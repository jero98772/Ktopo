#!/usr/bin/env python3
"""
Ktopo — AI brain for the Raspberry Pi 5
=======================================
 - Camera (Picamera2) + YOLO (Ultralytics, NCNN format) to recognize people and objects.
 - Modes:
     MANUAL   joystick from your phone (web page)
     FOLLOW   chases a person while keeping its distance
     MINER    looks for an object (a bottle by default), drives up to it and "drills" it (show)
     PATROL   explores, avoids obstacles and says hello when it sees someone
 - Web page with live video:  http://ktopo.local:8000   (or http://<pi-ip-address>:8000)
 - Talks to the ESP32 over USB. The ESP32 holds the safety reflexes: even if this
   program crashes, the rover stops by itself.

Usage:
  python3 ktopo_ai.py                       # on the rover
  python3 ktopo_ai.py --sim                 # on your PC, no hardware (simulated world)
  python3 ktopo_ai.py --port /dev/ttyUSB0 --model yolo11n_ncnn_model --rotate180
"""
import argparse
import glob
import math
import random
import threading
import time
from dataclasses import dataclass, asdict

import numpy as np

try:
    import cv2
except ImportError:  # without OpenCV there is no video on the web page, everything else works
    cv2 = None

# ---------------------------------------------------------------------------------------
# SETTINGS
# ---------------------------------------------------------------------------------------
WIDTH, HEIGHT = 640, 480        # camera resolution
AI_IMG = 320                    # resolution YOLO works at (smaller = faster)
CONFIDENCE = 0.45
BRAIN_HZ = 15                   # decisions per second
MAX_SPEED = 70                  # max % the AI uses for the wheels
STOP_DIST_MM = 330              # the miner stops at this distance (measured by the ToF sensors)
TARGETS = ["bottle", "cup", "sports ball", "teddy bear", "backpack", "chair", "cat", "dog", "person"]
FLAGS = {1: "obstacle", 2: "tilted", 4: "drill jammed", 8: "battery low", 16: "battery critical",
         32: "no link", 64: "EMERGENCY STOP", 128: "Bluetooth control", 256: "left ToF failed",
         512: "right ToF failed", 1024: "IMU failed", 2048: "show"}
MODES = ("idle", "manual", "follow", "miner", "patrol")


def clamp(v, lo=-100.0, hi=100.0):
    return max(lo, min(hi, v))


# ---------------------------------------------------------------------------------------
# DATA
# ---------------------------------------------------------------------------------------
@dataclass
class Telemetry:
    rpm_left: float = 0
    rpm_right: float = 0
    rpm_drill: float = 0
    dist_left: int = 8190
    dist_right: int = 8190
    pitch: float = 0
    roll: float = 0
    vbat: float = 0
    flags: int = 0
    out_left: float = 0
    out_right: float = 0
    out_drill: float = 0
    age: float = 99.0            # seconds since the last telemetry line

    def flag(self, b):
        return bool(self.flags & b)


@dataclass
class Detection:
    label: str
    conf: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def cx(self):
        return (self.x1 + self.x2) / 2

    @property
    def height(self):
        return self.y2 - self.y1

    @property
    def area(self):
        return (self.x2 - self.x1) * (self.y2 - self.y1)


# ---------------------------------------------------------------------------------------
# SIMULATED WORLD (only with --sim): a target placed around the rover
# ---------------------------------------------------------------------------------------
class SimWorld:
    FOV_H = math.radians(66)     # standard Camera Module 3

    def __init__(self):
        self.lock = threading.Lock()
        self.heading = 0.0       # rover heading (rad)
        self.x = 0.0
        self.y = 0.0
        self.t0 = time.time()
        self.label = "person"

    def target(self):
        if self.label != "person":
            return 2.2, 0.7                       # objects don't walk
        t = time.time() - self.t0
        # the person walks an oval around the point (2.5, 0)
        return 2.5 + 0.8 * math.cos(t * 0.15), 1.2 * math.sin(t * 0.22)

    def move(self, left, right, dt):
        with self.lock:
            v = (left + right) / 200 * 0.45       # m/s
            w = (left - right) / 200 * 2.4        # rad/s (positive = turns right)
            self.heading -= w * dt
            self.x += v * math.cos(self.heading) * dt
            self.y += v * math.sin(self.heading) * dt

    def relative(self):
        ox, oy = self.target()
        with self.lock:
            dx, dy = ox - self.x, oy - self.y
            d = math.hypot(dx, dy)
            ang = math.atan2(dy, dx) - self.heading
        ang = (ang + math.pi) % (2 * math.pi) - math.pi
        return d, -ang          # positive angle = right side of the image

    def detection(self):
        d, ang = self.relative()
        if abs(ang) > self.FOV_H / 2 or d < 0.15:
            return None
        cx = WIDTH / 2 + ang / (self.FOV_H / 2) * WIDTH / 2
        h = min(HEIGHT * 0.98, 600 * (0.9 if self.label == "person" else 0.25) / d)
        w = h * (0.4 if self.label == "person" else 0.45)
        y2 = HEIGHT / 2 + h * 0.55
        return Detection(self.label, 0.88, cx - w / 2, y2 - h, cx + w / 2, y2)


# ---------------------------------------------------------------------------------------
# LINK TO THE ESP32 (USB)
# ---------------------------------------------------------------------------------------
class ESP32Link:
    def __init__(self, port=None, sim: SimWorld = None):
        self.port = port
        self.sim = sim
        self.ser = None
        self.tel = Telemetry()
        self.t_tel = 0.0
        self.info = ""
        self.lock = threading.Lock()
        self._sim_tgt = [0.0, 0.0, 0.0]
        self._sim_out = [0.0, 0.0, 0.0]
        self._sim_flags = 0
        self._sim_last = 0.0
        threading.Thread(target=self._thread, daemon=True).start()

    # ---- sending ----
    def send(self, line):
        if self.sim:
            self._sim_receive(line.strip())
            return
        with self.lock:
            if self.ser:
                try:
                    self.ser.write((line.strip() + "\n").encode())
                except Exception:
                    self.ser = None

    def drive(self, left, right):
        self.send(f"V {clamp(left):.0f} {clamp(right):.0f}")

    def drill(self, v):
        self.send(f"T {clamp(v):.0f}")

    def lights(self, mode):
        self.send(f"L {int(mode)}")

    def beep(self, n=1):
        self.send(f"B {int(n)}")

    def emergency_stop(self):
        self.send("X")

    def reset(self):
        self.send("R")

    def show(self):
        self.send("1")

    def telemetry(self) -> Telemetry:
        t = Telemetry(**asdict(self.tel))
        t.age = time.time() - self.t_tel if self.t_tel else 99.0
        return t

    # ---- receiving ----
    def _parse(self, line):
        p = line.split()
        if not p:
            return
        if p[0] == "E" and len(p) >= 13:
            try:
                v = [float(x) for x in p[1:13]]
            except ValueError:
                return
            self.tel = Telemetry(v[0], v[1], v[2], int(v[3]), int(v[4]), v[5], v[6], v[7], int(v[8]),
                                 v[9], v[10], v[11])
            self.t_tel = time.time()
        elif p[0] == "I":
            self.info = line[2:]
            print("[ESP32]", self.info)

    def _find_port(self):
        if self.port:
            return self.port
        cands = sorted(glob.glob("/dev/serial/by-id/*")) + sorted(glob.glob("/dev/ttyUSB*")) + \
            sorted(glob.glob("/dev/ttyACM*"))
        return cands[0] if cands else None

    def _thread(self):
        if self.sim:
            self._sim_thread()
            return
        import serial
        while True:
            if self.ser is None:
                p = self._find_port()
                if not p:
                    time.sleep(2)
                    continue
                try:
                    s = serial.Serial(p, 115200, timeout=0.2)
                    time.sleep(0.3)
                    with self.lock:
                        self.ser = s
                    print("[ESP32] connected on", p)
                    self.send("?")
                except Exception as e:
                    print("[ESP32] could not open", p, e)
                    time.sleep(2)
                    continue
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
                if line:
                    self._parse(line)
            except Exception as e:
                print("[ESP32] disconnected:", e)
                with self.lock:
                    self.ser = None
                time.sleep(1)

    # ---- simulated ESP32 ----
    def _sim_receive(self, line):
        p = line.replace(",", " ").split()
        if not p:
            return
        c = p[0]
        try:
            if c == "V":
                self._sim_tgt[0], self._sim_tgt[1] = float(p[1]), float(p[2])
                self._sim_last = time.time()
            elif c == "T":
                self._sim_tgt[2] = float(p[1])
                self._sim_last = time.time()
            elif c == "P":
                self._sim_last = time.time()
            elif c == "X":
                self._sim_flags |= 64
            elif c == "R":
                self._sim_flags &= ~64
        except (IndexError, ValueError):
            pass

    def _sim_thread(self):
        dt = 0.05
        while True:
            if time.time() - self._sim_last > 0.6 or self._sim_flags & 64:
                self._sim_tgt = [0.0, 0.0, 0.0]
            d, ang = self.sim.relative()
            dist = int(d * 1000 - 330) if abs(ang) < math.radians(18) else 8190   # ToF measured from the front
            flags = self._sim_flags | (1 if 0 < dist < 260 else 0)
            tgt = list(self._sim_tgt)
            if flags & 1 and (tgt[0] + tgt[1]) / 2 > 0:
                fwd = (tgt[0] + tgt[1]) / 2
                tgt[0] -= fwd
                tgt[1] -= fwd
            for i, r in enumerate((250, 250, 70)):
                step = r * dt
                self._sim_out[i] = max(tgt[i], self._sim_out[i] - step) if tgt[i] < self._sim_out[i] \
                    else min(tgt[i], self._sim_out[i] + step)
            self.sim.move(self._sim_out[0], self._sim_out[1], dt)
            s = self._sim_out
            self._parse(f"E {s[0]:.0f} {s[1]:.0f} {s[2] * 2:.0f} {dist} {dist} 0.5 -0.3 11.8 {flags} "
                        f"{s[0]:.0f} {s[1]:.0f} {s[2]:.0f}")
            time.sleep(dt)


# ---------------------------------------------------------------------------------------
# VISION: camera + YOLO in its own thread
# ---------------------------------------------------------------------------------------
class Vision:
    def __init__(self, model, rotate180=False, sim: SimWorld = None, no_ai=False):
        self.sim = sim
        self.detections = []
        self.t_det = 0.0
        self.fps = 0.0
        self.jpg = None
        self.lock = threading.Lock()
        self.hud = {}
        self.cam = None
        self.cap = None
        self.model = None
        self.error = ""
        if not sim:
            self._open_camera(rotate180)
            if not no_ai:
                self._load_model(model)
        threading.Thread(target=self._thread, daemon=True).start()

    def _open_camera(self, rotate180):
        try:
            from picamera2 import Picamera2
            self.cam = Picamera2()
            kw = {}
            if rotate180:
                from libcamera import Transform
                kw["transform"] = Transform(hflip=1, vflip=1)
            cfg = self.cam.create_video_configuration(main={"size": (WIDTH, HEIGHT), "format": "RGB888"}, **kw)
            self.cam.configure(cfg)
            self.cam.start()
            try:
                self.cam.set_controls({"AfMode": 2})     # continuous autofocus (Camera Module 3)
            except Exception:
                pass
            print("[camera] Picamera2 ready")
        except Exception as e:
            print("[camera] Picamera2 not available:", e, "-> trying a USB camera")
            self.cam = None
            if cv2 is not None:
                self.cap = cv2.VideoCapture(0)
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
                if not self.cap.isOpened():
                    self.cap = None
                    self.error = "no camera"

    def _load_model(self, path):
        try:
            from ultralytics import YOLO
            self.model = YOLO(path, task="detect")
            print("[AI] model loaded:", path)
        except Exception as e:
            self.error = f"no AI model ({e})"
            print("[AI]", self.error)

    def _frame(self):
        if self.sim:
            img = np.full((HEIGHT, WIDTH, 3), (40, 34, 30), np.uint8)
            if cv2 is not None:
                for x in range(0, WIDTH, 40):
                    cv2.line(img, (x, HEIGHT // 2), (int(WIDTH / 2 + (x - WIDTH / 2) * 3), HEIGHT), (70, 60, 52), 1)
                det = self.sim.detection()
                if det:
                    cv2.rectangle(img, (int(det.x1), int(det.y1)), (int(det.x2), int(det.y2)), (90, 160, 230), -1)
            return img
        if self.cam is not None:
            return self.cam.capture_array()            # already in BGR order for OpenCV
        if self.cap is not None:
            ok, f = self.cap.read()
            return f if ok else None
        time.sleep(0.2)
        return None

    def _detect(self, img):
        if self.sim:
            d = self.sim.detection()
            return [d] if d else []
        if self.model is None:
            return []
        r = self.model.predict(img, imgsz=AI_IMG, conf=CONFIDENCE, verbose=False)[0]
        out = []
        for b in r.boxes:
            x1, y1, x2, y2 = [float(v) for v in b.xyxy[0].tolist()]
            out.append(Detection(r.names[int(b.cls[0])], float(b.conf[0]), x1, y1, x2, y2))
        return out

    def _draw(self, img, dets):
        if cv2 is None:
            return None
        for d in dets:
            col = (60, 220, 90) if d.label == self.hud.get("target") else (230, 160, 40)
            cv2.rectangle(img, (int(d.x1), int(d.y1)), (int(d.x2), int(d.y2)), col, 2)
            txt = f"{d.label} {d.conf:.0%}"
            cv2.putText(img, txt, (int(d.x1) + 3, max(16, int(d.y1) - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
        cv2.line(img, (WIDTH // 2, HEIGHT // 2 - 12), (WIDTH // 2, HEIGHT // 2 + 12), (255, 255, 255), 1)
        cv2.line(img, (WIDTH // 2 - 12, HEIGHT // 2), (WIDTH // 2 + 12, HEIGHT // 2), (255, 255, 255), 1)
        line = f"{self.hud.get('mode', '')}  {self.hud.get('status', '')}  {self.fps:.0f} fps"
        cv2.putText(img, line, (10, HEIGHT - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        return buf.tobytes() if ok else None

    def _thread(self):
        t_prev = time.time()
        while True:
            img = self._frame()
            if img is None:
                continue
            try:
                dets = self._detect(img)
            except Exception as e:
                self.error = f"AI error: {e}"
                dets = []
            jpg = self._draw(img, dets)
            now = time.time()
            self.fps = 0.9 * self.fps + 0.1 / max(1e-3, now - t_prev)
            t_prev = now
            with self.lock:
                self.detections = dets
                self.t_det = now
                if jpg:
                    self.jpg = jpg
            if self.sim:
                time.sleep(1 / 20)

    def latest(self, max_age=0.6):
        with self.lock:
            return list(self.detections) if time.time() - self.t_det < max_age else []


# ---------------------------------------------------------------------------------------
# BRAIN: decides what to do in each mode
# ---------------------------------------------------------------------------------------
class Brain:
    def __init__(self, esp: ESP32Link, vision: Vision):
        self.esp = esp
        self.vision = vision
        self.mode = "idle"
        self.target = "bottle"
        self.status = ""
        self.joy = (0.0, 0.0)
        self.t_joy = 0.0
        self.speed = 70
        self.manual_drill = 0.0
        self.light_mode = 1
        self.sub = ""             # sub-state of the autonomous modes
        self.t_sub = 0.0
        self.seen_t = 0.0
        self.seen_cx = WIDTH / 2
        self.turn_dir = 1
        self.t_hello = 0.0
        self.output = (0.0, 0.0, 0.0)
        self._dur = 1.0
        self._prev_lights = None
        self.lock = threading.Lock()
        threading.Thread(target=self._thread, daemon=True).start()

    # --- commands from the web page ---
    def command(self, d):
        with self.lock:
            if "mode" in d and d["mode"] in MODES:
                self.mode = d["mode"]
                self.sub, self.t_sub = "", time.time()
                self.manual_drill = 0.0
                self.esp.beep(1)
            if "target" in d and d["target"] in TARGETS:
                self.target = d["target"]
            if "joy" in d:
                self.joy = (clamp(float(d["joy"][0]), -1, 1), clamp(float(d["joy"][1]), -1, 1))
                self.t_joy = time.time()
            if "speed" in d:
                self.speed = clamp(float(d["speed"]), 10, 100)
            if "drill" in d:
                self.manual_drill = clamp(float(d["drill"]))
            if "lights" in d:
                self.light_mode = int(d["lights"])
            if d.get("estop"):
                self.mode = "idle"
                self.esp.emergency_stop()
            if d.get("reset"):
                self.esp.reset()
            if d.get("show"):
                self.esp.show()

    def _switch(self, sub):
        self.sub, self.t_sub = sub, time.time()

    def _elapsed(self):
        return time.time() - self.t_sub

    def _best(self, dets, label):
        c = [d for d in dets if d.label == label]
        return max(c, key=lambda d: d.area) if c else None

    def _aim(self, det, gain=55.0):
        """Returns the turn (%) needed to center the detection in the image."""
        err = (det.cx - WIDTH / 2) / (WIDTH / 2)
        self.seen_t, self.seen_cx = time.time(), det.cx
        self.turn_dir = 1 if err > 0 else -1
        return clamp(gain * err, -45, 45)

    # --- behaviors ---
    def _manual(self, tel):
        x, y = self.joy
        if time.time() - self.t_joy > 0.5:
            x = y = 0.0
        k = self.speed
        left = clamp(k * y + 0.6 * k * x)
        right = clamp(k * y - 0.6 * k * x)
        self.status = "joystick"
        return left, right, self.manual_drill, self.light_mode

    def _follow(self, tel, dets):
        p = self._best(dets, "person")
        if p:
            turn = self._aim(p)
            ratio = p.height / HEIGHT
            fwd = clamp(160 * (0.62 - ratio), -30, MAX_SPEED)
            if abs(turn) > 30:
                fwd *= 0.4             # turn first, then drive
            if self.sub != "following":
                self._switch("following")
                self.esp.beep(2)
            drill = 35 if self._elapsed() < 1.2 else 0      # waves the drill when it finds you
            self.status = f"following you ({ratio:.0%} of frame height)"
            return fwd + turn, fwd - turn, drill, 3
        lost = time.time() - self.seen_t
        if lost < 8:
            if self.sub != "searching":
                self._switch("searching")
            g = 28 * self.turn_dir
            self.status = "lost you, searching..."
            return g, -g, 0, 2
        self.status = "waiting for someone"
        return 0, 0, 0, 1

    def _miner(self, tel, dets):
        obj = self._best(dets, self.target)
        dist = min(tel.dist_left, tel.dist_right)
        name = self.target
        if self.sub in ("", "search"):
            if obj:
                self._switch("approach")
            else:
                self.sub = "search"
                self.status = f"looking for a {name}..."
                return 30, -30, 0, 2
        if self.sub == "approach":
            if obj:
                turn = self._aim(obj, 50)
                ratio = obj.height / HEIGHT
                close = (0 < dist < STOP_DIST_MM) or ratio > 0.55
                if close and abs(turn) < 12:
                    self._switch("drill")
                    self.esp.beep(3)
                else:
                    fwd = clamp(90 * (0.6 - ratio), 15, 55)
                    if abs(turn) > 25:
                        fwd = 0
                    self.status = f"approaching the {name} ({dist} mm)"
                    return fwd + turn, fwd - turn, 0, 3
            elif time.time() - self.seen_t > 1.5:
                self._switch("search")
                return 0, 0, 0, 2
            else:
                return 0, 0, 0, 3
        if self.sub == "drill":
            self.status = f"DRILLING THE {name.upper()}!"
            if self._elapsed() < 4.0:
                return 0, 0, 85, 4
            self._switch("back_off")
        if self.sub == "back_off":
            self.status = "backing off"
            if self._elapsed() < 1.2:
                return -35, -35, 0, 1
            self._switch("turn")
        if self.sub == "turn":
            self.status = "looking for another one"
            if self._elapsed() < 1.6:
                return 45, -45, 0, 2
            self._switch("search")
        return 0, 0, 0, 2

    def _patrol(self, tel, dets):
        dl, dr = tel.dist_left, tel.dist_right
        p = self._best(dets, "person")
        if p and p.height / HEIGHT > 0.25 and time.time() - self.t_hello > 12:
            self._switch("hello")
            self.t_hello = time.time()
            self.esp.beep(2)
        if self.sub == "hello":
            self.status = "hello, human!"
            if self._elapsed() < 2.0:
                w = 40 if int(self._elapsed() * 4) % 2 else -40
                return w, -w, 50, 4
            self._switch("")
        if self.sub == "avoid":
            self.status = "avoiding an obstacle"
            if self._elapsed() < 0.4:
                return -30, -30, 0, 2                       # back up a little first
            if self._elapsed() < 0.4 + self._dur:
                return 35 * self.turn_dir, -35 * self.turn_dir, 0, 2
            self._switch("")
        if min(dl, dr) < 450 or tel.flag(1):
            self.turn_dir = 1 if dl < dr else -1            # turn towards the clearer side (+1 = right)
            self._dur = random.uniform(0.7, 1.4)
            self._switch("avoid")
            return 0, 0, 0, 2
        self.status = "exploring"
        drift = 8 * math.sin(time.time() * 0.4)
        return 38 + drift, 38 - drift, 0, 1

    def _thread(self):
        period = 1.0 / BRAIN_HZ
        while True:
            t0 = time.time()
            tel = self.esp.telemetry()
            dets = self.vision.latest()
            with self.lock:
                mode = self.mode
                if self.esp.sim:   # in simulation, the world's object is whatever we are looking for
                    self.esp.sim.label = "person" if mode in ("follow", "patrol") else self.target
                try:
                    if tel.flag(64) or tel.flag(16) or tel.flag(2):
                        left = right = drill = 0
                        lights = 1
                        self.status = "stopped for safety: " + ", ".join(
                            n for b, n in FLAGS.items() if tel.flags & b and b in (2, 16, 64))
                    elif tel.flag(128):
                        left = right = drill = 0
                        lights = 1
                        self.status = "Bluetooth control (send 0 to hand control back)"
                    elif mode == "manual":
                        left, right, drill, lights = self._manual(tel)
                    elif mode == "follow":
                        left, right, drill, lights = self._follow(tel, dets)
                    elif mode == "miner":
                        left, right, drill, lights = self._miner(tel, dets)
                    elif mode == "patrol":
                        left, right, drill, lights = self._patrol(tel, dets)
                    else:
                        left = right = 0
                        drill = self.manual_drill
                        lights = self.light_mode
                        self.status = "ready"
                except Exception as e:  # never leave the rover moving because of a bug
                    left = right = drill = 0
                    lights = 1
                    self.status = f"error: {e}"
                lim = 100 if mode == "manual" else MAX_SPEED
                left = clamp(left, -lim, lim)
                right = clamp(right, -lim, lim)
                self.output = (left, right, drill)
            self.esp.drive(left, right)
            self.esp.drill(drill)
            if self._prev_lights != lights:
                self.esp.lights(lights)
                self._prev_lights = lights
            self.vision.hud = {"mode": mode.upper(), "status": self.status, "target":
                               "person" if mode in ("follow", "patrol") else self.target}
            time.sleep(max(0.0, period - (time.time() - t0)))

    def state_json(self):
        tel = self.esp.telemetry()
        dets = self.vision.latest()
        return {
            "mode": self.mode, "status": self.status, "target": self.target, "speed": self.speed,
            "output": [round(v) for v in self.output],
            "tel": asdict(tel), "flags": [n for b, n in FLAGS.items() if tel.flags & b],
            "fps": round(self.vision.fps, 1), "error": self.vision.error, "esp_info": self.esp.info,
            "detections": [{"label": d.label, "conf": round(d.conf, 2)} for d in dets],
            "targets": TARGETS,
        }


# ---------------------------------------------------------------------------------------
# WEB PAGE
# ---------------------------------------------------------------------------------------
PAGE = r"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Ktopo</title>
<style>
:root{--bg:#0f1216;--panel:#181d23;--line:#2a313a;--txt:#e8edf2;--dim:#8b96a3;--acc:#f08a3c;--ok:#48c774;--bad:#ff4d4d;--blue:#4aa3ff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--txt);font:15px/1.4 system-ui,sans-serif}
header{display:flex;align-items:center;gap:10px;padding:10px 14px;border-bottom:1px solid var(--line)}
header b{font-size:18px;letter-spacing:.06em}header span{color:var(--dim);font-size:13px}
.wrap{display:grid;grid-template-columns:minmax(0,1.5fr) minmax(280px,1fr);gap:12px;padding:12px;max-width:1200px;margin:auto}
@media(max-width:820px){.wrap{grid-template-columns:1fr}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:12px}
#video{width:100%;aspect-ratio:4/3;background:#000;border-radius:8px;display:block;object-fit:cover}
#status{margin-top:8px;font-weight:600;color:var(--acc);min-height:1.4em}
.modes{display:grid;grid-template-columns:repeat(5,1fr);gap:6px;margin-bottom:10px}
button{background:#232a33;color:var(--txt);border:1px solid var(--line);border-radius:9px;padding:10px 6px;font-weight:600;font-size:14px;cursor:pointer}
button.on{background:var(--acc);border-color:var(--acc);color:#1a0d02}
#stop{width:100%;background:var(--bad);border-color:var(--bad);font-size:20px;padding:16px;margin-top:10px;color:#fff}
.row{display:flex;gap:8px;align-items:center;margin:8px 0}.row label{color:var(--dim);min-width:74px;font-size:13px}
select,input[type=range]{flex:1}select{background:#232a33;color:var(--txt);border:1px solid var(--line);border-radius:8px;padding:8px}
#joy{width:210px;height:210px;border-radius:50%;background:radial-gradient(circle,#232a33 0,#151a20 70%);border:1px solid var(--line);margin:10px auto;position:relative;touch-action:none}
#nub{width:74px;height:74px;border-radius:50%;background:var(--acc);position:absolute;left:68px;top:68px;box-shadow:0 4px 18px #0008}
.grid{display:grid;grid-template-columns:repeat(2,1fr);gap:6px}.kv{background:#12161b;border-radius:8px;padding:8px}
.kv small{display:block;color:var(--dim);font-size:11px}.kv b{font-size:17px}
.chips span{display:inline-block;background:#3a1d1d;color:#ffb3b3;border-radius:20px;padding:2px 9px;margin:2px;font-size:12px}
.chips span.n{background:#1d2a3a;color:#b3d4ff}
</style></head><body>
<header><b>KTOPO</b><span id="info">connecting…</span></header>
<div class="wrap">
 <div class="card"><img id="video" src="/video.mjpg" alt="live video">
  <div id="status">…</div><div class="chips" id="dets"></div></div>
 <div class="card">
  <div class="modes">
   <button data-m="idle">Idle</button><button data-m="manual">Manual</button>
   <button data-m="follow">Follow</button><button data-m="miner">Miner</button><button data-m="patrol">Patrol</button></div>
  <div class="row"><label>Target</label><select id="target"></select></div>
  <div id="joy"><div id="nub"></div></div>
  <div class="row"><label>Speed</label><input type="range" id="speed" min="20" max="100" value="70"></div>
  <div class="row"><label>Drill</label><input type="range" id="drill" min="-100" max="100" value="0" step="5"><button id="drill0">0</button><button id="drillmax">MAX</button></div>
  <div class="row"><label>Lights</label><select id="lights"><option value="1">Breathe</option><option value="2">Searching</option><option value="3">Target</option><option value="4">Party</option><option value="5">Headlights</option><option value="0">Off</option></select><button id="show">Show</button></div>
  <button id="stop">EMERGENCY STOP</button>
  <div class="row"><button id="reset" style="flex:1">Clear faults</button></div>
  <div class="grid" style="margin-top:8px">
   <div class="kv"><small>Battery</small><b id="bat">–</b></div><div class="kv"><small>Distance left / right</small><b id="dist">–</b></div>
   <div class="kv"><small>Tilt</small><b id="tilt">–</b></div><div class="kv"><small>RPM left / right / drill</small><b id="rpm">–</b></div></div>
  <div class="chips" id="flags" style="margin-top:8px"></div>
 </div></div>
<script>
const $=s=>document.querySelector(s);
const post=d=>fetch('/api/cmd',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)}).catch(()=>{});
document.querySelectorAll('[data-m]').forEach(b=>b.onclick=()=>post({mode:b.dataset.m}));
$('#target').onchange=e=>post({target:e.target.value});
$('#speed').oninput=e=>post({speed:+e.target.value});
$('#drill').oninput=e=>post({drill:+e.target.value});
$('#drill0').onclick=()=>{$('#drill').value=0;post({drill:0})};
$('#drillmax').onclick=()=>{$('#drill').value=100;post({drill:100})};
$('#lights').onchange=e=>post({lights:+e.target.value});
$('#show').onclick=()=>post({show:1});
$('#stop').onclick=()=>post({estop:1});
$('#reset').onclick=()=>post({reset:1});
// joystick
const joy=$('#joy'),nub=$('#nub');let jx=0,jy=0,act=false;
function mv(e){const r=joy.getBoundingClientRect();let x=(e.clientX-r.left-r.width/2)/(r.width/2-37),y=-(e.clientY-r.top-r.height/2)/(r.height/2-37);
 const m=Math.hypot(x,y);if(m>1){x/=m;y/=m}jx=x;jy=y;nub.style.left=(68+x*68)+'px';nub.style.top=(68-y*68)+'px'}
joy.onpointerdown=e=>{act=true;joy.setPointerCapture(e.pointerId);mv(e)};
joy.onpointermove=e=>{if(act)mv(e)};
joy.onpointerup=joy.onpointercancel=()=>{act=false;jx=jy=0;nub.style.left='68px';nub.style.top='68px';post({joy:[0,0]})};
setInterval(()=>{if(act)post({joy:[jx,jy]})},100);
let targetsLoaded=false;
async function tick(){try{const s=await (await fetch('/api/state')).json();
 document.querySelectorAll('[data-m]').forEach(b=>b.classList.toggle('on',b.dataset.m==s.mode));
 if(!targetsLoaded){$('#target').innerHTML=s.targets.map(t=>`<option value="${t}">${t}</option>`).join('');targetsLoaded=true}
 $('#target').value=s.target;
 $('#status').textContent=s.status+(s.error?'  ·  '+s.error:'');
 const t=s.tel;const pct=Math.max(0,Math.min(100,(t.vbat-9.9)/(12.6-9.9)*100));
 $('#bat').textContent=t.age>2?'no data':`${t.vbat.toFixed(2)} V · ${pct.toFixed(0)}%`;
 const f=v=>v>=8190?'clear':(v/10).toFixed(0)+' cm';
 $('#dist').textContent=`${f(t.dist_left)} / ${f(t.dist_right)}`;
 $('#tilt').textContent=`${t.pitch.toFixed(0)}° / ${t.roll.toFixed(0)}°`;
 $('#rpm').textContent=`${t.rpm_left.toFixed(0)} / ${t.rpm_right.toFixed(0)} / ${t.rpm_drill.toFixed(0)}`;
 $('#flags').innerHTML=s.flags.map(b=>`<span>${b}</span>`).join('');
 $('#dets').innerHTML=s.detections.map(d=>`<span class="n">${d.label} ${(d.conf*100).toFixed(0)}%</span>`).join('');
 $('#info').textContent=`AI ${s.fps} fps · output ${s.output.join(' / ')}`;
}catch(e){$('#info').textContent='disconnected'}}
setInterval(tick,400);tick();
</script></body></html>"""


def create_web(brain: Brain, vision: Vision):
    from flask import Flask, Response, jsonify, request
    app = Flask(__name__)

    @app.get("/")
    def index():
        return PAGE

    @app.get("/api/state")
    def state():
        return jsonify(brain.state_json())

    @app.post("/api/cmd")
    def cmd():
        d = request.get_json(force=True, silent=True) or {}
        brain.command(d)
        return jsonify(ok=True)

    @app.get("/video.mjpg")
    def video():
        def gen():
            while True:
                with vision.lock:
                    jpg = vision.jpg
                if jpg:
                    yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
                time.sleep(1 / 15)
        return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

    return app


def main():
    ap = argparse.ArgumentParser(description="Ktopo — AI brain")
    ap.add_argument("--sim", action="store_true", help="no hardware: simulated world, camera and ESP32")
    ap.add_argument("--port", default=None, help="ESP32 serial port (auto-detected if omitted)")
    ap.add_argument("--model", default="yolo11n_ncnn_model", help="YOLO model (NCNN folder or .pt)")
    ap.add_argument("--rotate180", action="store_true", help="if the image is upside down")
    ap.add_argument("--no-ai", action="store_true", help="video and manual control only")
    ap.add_argument("--web", type=int, default=8000, help="web page port")
    a = ap.parse_args()
    sim = SimWorld() if a.sim else None
    esp = ESP32Link(a.port, sim=sim)
    vision = Vision(a.model, a.rotate180, sim=sim, no_ai=a.no_ai)
    brain = Brain(esp, vision)
    app = create_web(brain, vision)
    print(f"[web] open http://ktopo.local:{a.web}  (or the Raspberry Pi's IP address)")
    app.run(host="0.0.0.0", port=a.web, threaded=True, debug=False)


if __name__ == "__main__":
    main()
