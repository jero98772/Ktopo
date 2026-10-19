#!/usr/bin/env python3
"""
TOPO S14 — Cerebro con IA para Raspberry Pi 5
================================================
 - Cámara (Picamera2) + YOLO (Ultralytics, formato NCNN) para reconocer personas y objetos.
 - Modos:
     MANUAL    joystick desde el celular (página web)
     SEGUIR    persigue a una persona manteniendo distancia
     MINERO    busca un objeto (por defecto una botella), se acerca y lo "perfora" (show)
     PATRULLA  explora esquivando obstáculos y saluda cuando ve a alguien
 - Página web con video en vivo:  http://topo.local:8000   (o http://IP-de-la-pi:8000)
 - Habla con el ESP32 por USB. El ESP32 tiene los reflejos de seguridad: aunque este
   programa falle, el carro frena solo.

Uso:
  python3 topo_ia.py                       # en el carro
  python3 topo_ia.py --sim                 # en tu PC, sin hardware (mundo simulado)
  python3 topo_ia.py --puerto /dev/ttyUSB0 --modelo yolo11n_ncnn_model --rotar180
"""
import argparse
import glob
import json
import math
import random
import threading
import time
import unicodedata
from dataclasses import dataclass, field, asdict

import numpy as np

try:
    import cv2
except ImportError:  # sin OpenCV no hay video en la web, pero el resto funciona
    cv2 = None

# ---------------------------------------------------------------------------------------
# AJUSTES
# ---------------------------------------------------------------------------------------
ANCHO, ALTO = 640, 480          # resolución de la cámara
IMG_IA = 320                    # resolución que usa YOLO (más pequeño = más rápido)
CONFIANZA = 0.45
FREC_CEREBRO = 15               # decisiones por segundo
VEL_MAX = 70                    # % máximo que usa la IA para las ruedas
DIST_PARADA_MM = 330            # el minero se detiene a esta distancia (medida por los ToF)
OBJETIVOS = ["bottle", "cup", "sports ball", "teddy bear", "backpack", "chair", "cat", "dog", "person"]
NOMBRES_ES = {"person": "persona", "bottle": "botella", "cup": "taza", "sports ball": "balón",
              "teddy bear": "peluche", "backpack": "morral", "chair": "silla", "cat": "gato", "dog": "perro"}
BANDERAS = {1: "obstáculo", 2: "inclinado", 4: "taladro trabado", 8: "batería baja", 16: "batería crítica",
            32: "sin enlace", 64: "EMERGENCIA", 128: "control Bluetooth", 256: "ToF izq falla",
            512: "ToF der falla", 1024: "IMU falla", 2048: "show"}


def ascii(txt):
    """OpenCV no dibuja tildes: 'acercándome' -> 'acercandome'."""
    return unicodedata.normalize("NFKD", txt).encode("ascii", "ignore").decode()


def limitar(v, lo=-100.0, hi=100.0):
    return max(lo, min(hi, v))


# ---------------------------------------------------------------------------------------
# DATOS
# ---------------------------------------------------------------------------------------
@dataclass
class Telemetria:
    rpm_izq: float = 0
    rpm_der: float = 0
    rpm_tal: float = 0
    dist_izq: int = 8190
    dist_der: int = 8190
    pitch: float = 0
    roll: float = 0
    vbat: float = 0
    banderas: int = 0
    sal_izq: float = 0
    sal_der: float = 0
    sal_tal: float = 0
    edad: float = 99.0           # segundos desde la última telemetría

    def bandera(self, b):
        return bool(self.banderas & b)


@dataclass
class Deteccion:
    clase: str
    conf: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def cx(self):
        return (self.x1 + self.x2) / 2

    @property
    def alto(self):
        return self.y2 - self.y1

    @property
    def area(self):
        return (self.x2 - self.x1) * (self.y2 - self.y1)


# ---------------------------------------------------------------------------------------
# MUNDO SIMULADO (solo con --sim): un objetivo que se mueve alrededor del carro
# ---------------------------------------------------------------------------------------
class MundoSim:
    FOV_H = math.radians(66)     # Camera Module 3 normal

    def __init__(self):
        self.lock = threading.Lock()
        self.rumbo = 0.0         # rumbo del carro (rad)
        self.x = 0.0
        self.y = 0.0
        self.t0 = time.time()
        self.clase = "person"

    def objetivo(self):
        if self.clase != "person":
            return 2.2, 0.7                       # los objetos no caminan
        t = time.time() - self.t0
        # la persona camina en un óvalo alrededor del punto (2.5, 0)
        return 2.5 + 0.8 * math.cos(t * 0.15), 1.2 * math.sin(t * 0.22)

    def mover(self, izq, der, dt):
        with self.lock:
            v = (izq + der) / 200 * 0.45          # m/s
            w = (izq - der) / 200 * 2.4           # rad/s (positivo = gira a la derecha)
            self.rumbo -= w * dt
            self.x += v * math.cos(self.rumbo) * dt
            self.y += v * math.sin(self.rumbo) * dt

    def relativo(self):
        ox, oy = self.objetivo()
        with self.lock:
            dx, dy = ox - self.x, oy - self.y
            d = math.hypot(dx, dy)
            ang = math.atan2(dy, dx) - self.rumbo
        ang = (ang + math.pi) % (2 * math.pi) - math.pi
        return d, -ang          # ángulo positivo = a la derecha de la imagen

    def deteccion(self):
        d, ang = self.relativo()
        if abs(ang) > self.FOV_H / 2 or d < 0.15:
            return None
        cx = ANCHO / 2 + ang / (self.FOV_H / 2) * ANCHO / 2
        h = min(ALTO * 0.98, 600 * (0.9 if self.clase == "person" else 0.25) / d)
        w = h * (0.4 if self.clase == "person" else 0.45)
        y2 = ALTO / 2 + h * 0.55
        return Deteccion(self.clase, 0.88, cx - w / 2, y2 - h, cx + w / 2, y2)


# ---------------------------------------------------------------------------------------
# ENLACE CON EL ESP32 (USB)
# ---------------------------------------------------------------------------------------
class EnlaceESP32:
    def __init__(self, puerto=None, sim: MundoSim = None):
        self.puerto = puerto
        self.sim = sim
        self.ser = None
        self.tel = Telemetria()
        self.t_tel = 0.0
        self.info = ""
        self.lock = threading.Lock()
        self._sim_obj = [0.0, 0.0, 0.0]
        self._sim_sal = [0.0, 0.0, 0.0]
        self._sim_flags = 0
        self._sim_ultimo = 0.0
        threading.Thread(target=self._hilo, daemon=True).start()

    # ---- envío ----
    def enviar(self, linea):
        if self.sim:
            self._sim_recibir(linea.strip())
            return
        with self.lock:
            if self.ser:
                try:
                    self.ser.write((linea.strip() + "\n").encode())
                except Exception:
                    self.ser = None

    def mover(self, izq, der):
        self.enviar(f"V {limitar(izq):.0f} {limitar(der):.0f}")

    def taladro(self, v):
        self.enviar(f"T {limitar(v):.0f}")

    def luces(self, modo):
        self.enviar(f"L {int(modo)}")

    def pitar(self, n=1):
        self.enviar(f"B {int(n)}")

    def emergencia(self):
        self.enviar("X")

    def reset(self):
        self.enviar("R")

    def show(self):
        self.enviar("1")

    def telemetria(self) -> Telemetria:
        t = Telemetria(**{k: v for k, v in asdict(self.tel).items()})
        t.edad = time.time() - self.t_tel if self.t_tel else 99.0
        return t

    # ---- recepción ----
    def _parsear(self, linea):
        p = linea.split()
        if not p:
            return
        if p[0] == "E" and len(p) >= 13:
            try:
                v = [float(x) for x in p[1:13]]
            except ValueError:
                return
            self.tel = Telemetria(v[0], v[1], v[2], int(v[3]), int(v[4]), v[5], v[6], v[7], int(v[8]),
                                  v[9], v[10], v[11])
            self.t_tel = time.time()
        elif p[0] == "I":
            self.info = linea[2:]
            print("[ESP32]", self.info)

    def _buscar_puerto(self):
        if self.puerto:
            return self.puerto
        cands = sorted(glob.glob("/dev/serial/by-id/*")) + sorted(glob.glob("/dev/ttyUSB*")) + \
            sorted(glob.glob("/dev/ttyACM*"))
        return cands[0] if cands else None

    def _hilo(self):
        if self.sim:
            self._hilo_sim()
            return
        import serial
        while True:
            if self.ser is None:
                p = self._buscar_puerto()
                if not p:
                    time.sleep(2)
                    continue
                try:
                    s = serial.Serial(p, 115200, timeout=0.2)
                    time.sleep(0.3)
                    with self.lock:
                        self.ser = s
                    print("[ESP32] conectado en", p)
                    self.enviar("?")
                except Exception as e:
                    print("[ESP32] no se pudo abrir", p, e)
                    time.sleep(2)
                    continue
            try:
                linea = self.ser.readline().decode(errors="ignore").strip()
                if linea:
                    self._parsear(linea)
            except Exception as e:
                print("[ESP32] desconectado:", e)
                with self.lock:
                    self.ser = None
                time.sleep(1)

    # ---- ESP32 simulado ----
    def _sim_recibir(self, linea):
        p = linea.replace(",", " ").split()
        if not p:
            return
        c = p[0]
        try:
            if c == "V":
                self._sim_obj[0], self._sim_obj[1] = float(p[1]), float(p[2])
                self._sim_ultimo = time.time()
            elif c == "T":
                self._sim_obj[2] = float(p[1])
                self._sim_ultimo = time.time()
            elif c == "P":
                self._sim_ultimo = time.time()
            elif c == "X":
                self._sim_flags |= 64
            elif c == "R":
                self._sim_flags &= ~64
        except (IndexError, ValueError):
            pass

    def _hilo_sim(self):
        dt = 0.05
        while True:
            if time.time() - self._sim_ultimo > 0.6 or self._sim_flags & 64:
                self._sim_obj = [0.0, 0.0, 0.0]
            d, ang = self.sim.relativo()
            dist = int(d * 1000 - 330) if abs(ang) < math.radians(18) else 8190   # ToF desde el frente
            flags = self._sim_flags | (1 if 0 < dist < 260 else 0)
            obj = list(self._sim_obj)
            if flags & 1 and (obj[0] + obj[1]) / 2 > 0:
                av = (obj[0] + obj[1]) / 2
                obj[0] -= av
                obj[1] -= av
            for i, r in enumerate((250, 250, 70)):
                paso = r * dt
                self._sim_sal[i] = max(obj[i], self._sim_sal[i] - paso) if obj[i] < self._sim_sal[i] \
                    else min(obj[i], self._sim_sal[i] + paso)
            self.sim.mover(self._sim_sal[0], self._sim_sal[1], dt)
            s = self._sim_sal
            self._parsear(f"E {s[0]:.0f} {s[1]:.0f} {s[2] * 2:.0f} {dist} {dist} 0.5 -0.3 11.8 {flags} "
                          f"{s[0]:.0f} {s[1]:.0f} {s[2]:.0f}")
            time.sleep(dt)


# ---------------------------------------------------------------------------------------
# VISIÓN: cámara + YOLO en su propio hilo
# ---------------------------------------------------------------------------------------
class Vision:
    def __init__(self, modelo, rotar180=False, sim: MundoSim = None, sin_ia=False):
        self.sim = sim
        self.detecciones = []
        self.t_det = 0.0
        self.fps = 0.0
        self.jpg = None
        self.lock = threading.Lock()
        self.hud = {}
        self.cam = None
        self.cap = None
        self.modelo = None
        self.error = ""
        if not sim:
            self._abrir_camara(rotar180)
            if not sin_ia:
                self._cargar_modelo(modelo)
        threading.Thread(target=self._hilo, daemon=True).start()

    def _abrir_camara(self, rotar180):
        try:
            from picamera2 import Picamera2
            self.cam = Picamera2()
            kw = {}
            if rotar180:
                from libcamera import Transform
                kw["transform"] = Transform(hflip=1, vflip=1)
            cfg = self.cam.create_video_configuration(main={"size": (ANCHO, ALTO), "format": "RGB888"}, **kw)
            self.cam.configure(cfg)
            self.cam.start()
            try:
                self.cam.set_controls({"AfMode": 2})     # autofoco continuo (Camera Module 3)
            except Exception:
                pass
            print("[cámara] Picamera2 lista")
        except Exception as e:
            print("[cámara] Picamera2 no disponible:", e, "-> probando cámara USB")
            self.cam = None
            if cv2 is not None:
                self.cap = cv2.VideoCapture(0)
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, ANCHO)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ALTO)
                if not self.cap.isOpened():
                    self.cap = None
                    self.error = "sin cámara"

    def _cargar_modelo(self, ruta):
        try:
            from ultralytics import YOLO
            self.modelo = YOLO(ruta, task="detect")
            print("[IA] modelo cargado:", ruta)
        except Exception as e:
            self.error = f"sin modelo IA ({e})"
            print("[IA]", self.error)

    def _cuadro(self):
        if self.sim:
            img = np.full((ALTO, ANCHO, 3), (40, 34, 30), np.uint8)
            if cv2 is not None:
                for x in range(0, ANCHO, 40):
                    cv2.line(img, (x, ALTO // 2), (int(ANCHO / 2 + (x - ANCHO / 2) * 3), ALTO), (70, 60, 52), 1)
                det = self.sim.deteccion()
                if det:
                    cv2.rectangle(img, (int(det.x1), int(det.y1)), (int(det.x2), int(det.y2)), (90, 160, 230), -1)
            return img
        if self.cam is not None:
            return self.cam.capture_array()            # ya viene en orden BGR para OpenCV
        if self.cap is not None:
            ok, f = self.cap.read()
            return f if ok else None
        time.sleep(0.2)
        return None

    def _detectar(self, img):
        if self.sim:
            d = self.sim.deteccion()
            return [d] if d else []
        if self.modelo is None:
            return []
        r = self.modelo.predict(img, imgsz=IMG_IA, conf=CONFIANZA, verbose=False)[0]
        out = []
        for b in r.boxes:
            x1, y1, x2, y2 = [float(v) for v in b.xyxy[0].tolist()]
            out.append(Deteccion(r.names[int(b.cls[0])], float(b.conf[0]), x1, y1, x2, y2))
        return out

    def _dibujar(self, img, dets):
        if cv2 is None:
            return None
        for d in dets:
            col = (60, 220, 90) if d.clase == self.hud.get("objetivo") else (230, 160, 40)
            cv2.rectangle(img, (int(d.x1), int(d.y1)), (int(d.x2), int(d.y2)), col, 2)
            txt = ascii(f"{NOMBRES_ES.get(d.clase, d.clase)} {d.conf:.0%}")
            cv2.putText(img, txt, (int(d.x1) + 3, max(16, int(d.y1) - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
        cv2.line(img, (ANCHO // 2, ALTO // 2 - 12), (ANCHO // 2, ALTO // 2 + 12), (255, 255, 255), 1)
        cv2.line(img, (ANCHO // 2 - 12, ALTO // 2), (ANCHO // 2 + 12, ALTO // 2), (255, 255, 255), 1)
        linea = ascii(f"{self.hud.get('modo', '')}  {self.hud.get('estado', '')}  {self.fps:.0f} fps")
        cv2.putText(img, linea, (10, ALTO - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        return buf.tobytes() if ok else None

    def _hilo(self):
        t_prev = time.time()
        while True:
            img = self._cuadro()
            if img is None:
                continue
            try:
                dets = self._detectar(img)
            except Exception as e:
                self.error = f"error IA: {e}"
                dets = []
            jpg = self._dibujar(img, dets)
            ahora = time.time()
            self.fps = 0.9 * self.fps + 0.1 / max(1e-3, ahora - t_prev)
            t_prev = ahora
            with self.lock:
                self.detecciones = dets
                self.t_det = ahora
                if jpg:
                    self.jpg = jpg
            if self.sim:
                time.sleep(1 / 20)

    def ultimas(self, max_edad=0.6):
        with self.lock:
            return list(self.detecciones) if time.time() - self.t_det < max_edad else []


# ---------------------------------------------------------------------------------------
# CEREBRO: decide qué hacer según el modo
# ---------------------------------------------------------------------------------------
class Cerebro:
    def __init__(self, esp: EnlaceESP32, vision: Vision):
        self.esp = esp
        self.vision = vision
        self.modo = "quieto"
        self.objetivo = "bottle"
        self.estado = ""
        self.joy = (0.0, 0.0)
        self.t_joy = 0.0
        self.vel = 70
        self.taladro_manual = 0.0
        self.luces = 1
        self.sub = ""             # sub-estado de los modos automáticos
        self.t_sub = 0.0
        self.visto_t = 0.0
        self.visto_cx = ANCHO / 2
        self.giro_dir = 1
        self.t_saludo = 0.0
        self.salida = (0.0, 0.0, 0.0)
        self.lock = threading.Lock()
        threading.Thread(target=self._hilo, daemon=True).start()

    # --- órdenes desde la web ---
    def cmd(self, d):
        with self.lock:
            if "modo" in d and d["modo"] in ("quieto", "manual", "seguir", "minero", "patrulla"):
                self.modo = d["modo"]
                self.sub, self.t_sub = "", time.time()
                self.taladro_manual = 0.0
                self.esp.pitar(1)
            if "objetivo" in d and d["objetivo"] in OBJETIVOS:
                self.objetivo = d["objetivo"]
            if "joy" in d:
                self.joy = (limitar(float(d["joy"][0]), -1, 1), limitar(float(d["joy"][1]), -1, 1))
                self.t_joy = time.time()
            if "vel" in d:
                self.vel = limitar(float(d["vel"]), 10, 100)
            if "taladro" in d:
                self.taladro_manual = limitar(float(d["taladro"]))
            if "luces" in d:
                self.luces = int(d["luces"])
            if d.get("emergencia"):
                self.modo = "quieto"
                self.esp.emergencia()
            if d.get("reset"):
                self.esp.reset()
            if d.get("show"):
                self.esp.show()

    def _cambiar(self, sub):
        self.sub, self.t_sub = sub, time.time()

    def _tiempo(self):
        return time.time() - self.t_sub

    def _mejor(self, dets, clase):
        c = [d for d in dets if d.clase == clase]
        return max(c, key=lambda d: d.area) if c else None

    def _apuntar(self, det, ganancia=55.0):
        """Devuelve el giro (%) para centrar la detección en la imagen."""
        err = (det.cx - ANCHO / 2) / (ANCHO / 2)
        self.visto_t, self.visto_cx = time.time(), det.cx
        self.giro_dir = 1 if err > 0 else -1
        return limitar(ganancia * err, -45, 45)

    # --- comportamientos ---
    def _manual(self, tel):
        x, y = self.joy
        if time.time() - self.t_joy > 0.5:
            x = y = 0.0
        k = self.vel
        izq = limitar(k * y + 0.6 * k * x)
        der = limitar(k * y - 0.6 * k * x)
        self.estado = "joystick"
        return izq, der, self.taladro_manual, self.luces

    def _seguir(self, tel, dets):
        p = self._mejor(dets, "person")
        if p:
            giro = self._apuntar(p)
            ratio = p.alto / ALTO
            avance = limitar(160 * (0.62 - ratio), -30, VEL_MAX)
            if abs(giro) > 30:
                avance *= 0.4          # primero girar, luego avanzar
            if self.sub != "siguiendo":
                self._cambiar("siguiendo")
                self.esp.pitar(2)
            tal = 35 if self._tiempo() < 1.2 else 0      # saludo con el taladro al encontrarte
            self.estado = f"te sigo ({ratio:.0%} de alto)"
            return avance + giro, avance - giro, tal, 3
        perdido = time.time() - self.visto_t
        if perdido < 8:
            if self.sub != "buscando":
                self._cambiar("buscando")
            g = 28 * self.giro_dir
            self.estado = "te perdí, buscando..."
            return g, -g, 0, 2
        self.estado = "esperando a alguien"
        return 0, 0, 0, 1

    def _minero(self, tel, dets):
        obj = self._mejor(dets, self.objetivo)
        dist = min(tel.dist_izq, tel.dist_der)
        nombre = NOMBRES_ES.get(self.objetivo, self.objetivo)
        if self.sub in ("", "buscar"):
            if obj:
                self._cambiar("acercar")
            else:
                self.sub = "buscar"
                self.estado = f"buscando {nombre}..."
                return 30, -30, 0, 2
        if self.sub == "acercar":
            if obj:
                giro = self._apuntar(obj, 50)
                ratio = obj.alto / ALTO
                cerca = (0 < dist < DIST_PARADA_MM) or ratio > 0.55
                if cerca and abs(giro) < 12:
                    self._cambiar("perforar")
                    self.esp.pitar(3)
                else:
                    avance = limitar(90 * (0.6 - ratio), 15, 55)
                    if abs(giro) > 25:
                        avance = 0
                    self.estado = f"acercándome a {nombre} ({dist} mm)"
                    return avance + giro, avance - giro, 0, 3
            elif time.time() - self.visto_t > 1.5:
                self._cambiar("buscar")
                return 0, 0, 0, 2
            else:
                return 0, 0, 0, 3
        if self.sub == "perforar":
            self.estado = f"¡PERFORANDO {nombre.upper()}!"
            if self._tiempo() < 4.0:
                return 0, 0, 85, 4
            self._cambiar("retroceder")
        if self.sub == "retroceder":
            self.estado = "retrocediendo"
            if self._tiempo() < 1.2:
                return -35, -35, 0, 1
            self._cambiar("girar")
        if self.sub == "girar":
            self.estado = "buscando otro"
            if self._tiempo() < 1.6:
                return 45, -45, 0, 2
            self._cambiar("buscar")
        return 0, 0, 0, 2

    def _patrulla(self, tel, dets):
        izq_d, der_d = tel.dist_izq, tel.dist_der
        p = self._mejor(dets, "person")
        if p and p.alto / ALTO > 0.25 and time.time() - self.t_saludo > 12:
            self._cambiar("saludar")
            self.t_saludo = time.time()
            self.esp.pitar(2)
        if self.sub == "saludar":
            self.estado = "¡hola humano!"
            if self._tiempo() < 2.0:
                w = 40 if int(self._tiempo() * 4) % 2 else -40
                return w, -w, 50, 4
            self._cambiar("")
        if self.sub == "esquivar":
            self.estado = "esquivando"
            if self._tiempo() < 0.4:
                return -30, -30, 0, 2                       # primero retrocede un poco
            if self._tiempo() < 0.4 + self._dur:
                return 35 * self.giro_dir, -35 * self.giro_dir, 0, 2
            self._cambiar("")
        if min(izq_d, der_d) < 450 or tel.bandera(1):
            self.giro_dir = 1 if izq_d < der_d else -1     # gira hacia el lado más libre (+1 = derecha)
            self._dur = random.uniform(0.7, 1.4)
            self._cambiar("esquivar")
            return 0, 0, 0, 2
        self.estado = "explorando"
        deriva = 8 * math.sin(time.time() * 0.4)
        return 38 + deriva, 38 - deriva, 0, 1

    def _hilo(self):
        periodo = 1.0 / FREC_CEREBRO
        self._dur = 1.0
        while True:
            t0 = time.time()
            tel = self.esp.telemetria()
            dets = self.vision.ultimas()
            with self.lock:
                modo = self.modo
                if self.esp.sim:   # en simulación el "objeto" del mundo es lo que se busca
                    self.esp.sim.clase = "person" if modo in ("seguir", "patrulla") else self.objetivo
                try:
                    if tel.bandera(64) or tel.bandera(16) or tel.bandera(2):
                        izq = der = tal = 0
                        luces = 1
                        self.estado = "parado por seguridad: " + ", ".join(
                            n for b, n in BANDERAS.items() if tel.banderas & b and b in (2, 16, 64))
                    elif tel.bandera(128):
                        izq = der = tal = 0
                        luces = 1
                        self.estado = "control por Bluetooth (envía 0 para devolver el mando)"
                    elif modo == "manual":
                        izq, der, tal, luces = self._manual(tel)
                    elif modo == "seguir":
                        izq, der, tal, luces = self._seguir(tel, dets)
                    elif modo == "minero":
                        izq, der, tal, luces = self._minero(tel, dets)
                    elif modo == "patrulla":
                        izq, der, tal, luces = self._patrulla(tel, dets)
                    else:
                        izq = der = 0
                        tal = self.taladro_manual
                        luces = self.luces
                        self.estado = "listo"
                except Exception as e:  # nunca dejar el carro andando por un error
                    izq = der = tal = 0
                    luces = 1
                    self.estado = f"error: {e}"
                izq = limitar(izq, -VEL_MAX if modo != "manual" else -100, VEL_MAX if modo != "manual" else 100)
                der = limitar(der, -VEL_MAX if modo != "manual" else -100, VEL_MAX if modo != "manual" else 100)
                self.salida = (izq, der, tal)
            self.esp.mover(izq, der)
            self.esp.taladro(tal)
            if getattr(self, "_luces_prev", None) != luces:
                self.esp.luces(luces)
                self._luces_prev = luces
            self.vision.hud = {"modo": modo.upper(), "estado": self.estado, "objetivo":
                               "person" if modo in ("seguir", "patrulla") else self.objetivo}
            time.sleep(max(0.0, periodo - (time.time() - t0)))

    def estado_json(self):
        tel = self.esp.telemetria()
        dets = self.vision.ultimas()
        return {
            "modo": self.modo, "estado": self.estado, "objetivo": self.objetivo, "vel": self.vel,
            "salida": [round(v) for v in self.salida],
            "tel": asdict(tel), "banderas": [n for b, n in BANDERAS.items() if tel.banderas & b],
            "fps": round(self.vision.fps, 1), "error": self.vision.error, "esp_info": self.esp.info,
            "detecciones": [{"clase": NOMBRES_ES.get(d.clase, d.clase), "conf": round(d.conf, 2)} for d in dets],
            "objetivos": [{"id": o, "nombre": NOMBRES_ES.get(o, o)} for o in OBJETIVOS],
        }


# ---------------------------------------------------------------------------------------
# PÁGINA WEB
# ---------------------------------------------------------------------------------------
PAGINA = r"""<!doctype html><html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>TOPO S14</title>
<style>
:root{--bg:#0f1216;--panel:#181d23;--line:#2a313a;--txt:#e8edf2;--dim:#8b96a3;--acc:#f08a3c;--ok:#48c774;--bad:#ff4d4d;--blue:#4aa3ff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--txt);font:15px/1.4 system-ui,sans-serif}
header{display:flex;align-items:center;gap:10px;padding:10px 14px;border-bottom:1px solid var(--line)}
header b{font-size:18px;letter-spacing:.06em}header span{color:var(--dim);font-size:13px}
.wrap{display:grid;grid-template-columns:minmax(0,1.5fr) minmax(280px,1fr);gap:12px;padding:12px;max-width:1200px;margin:auto}
@media(max-width:820px){.wrap{grid-template-columns:1fr}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:12px}
#video{width:100%;aspect-ratio:4/3;background:#000;border-radius:8px;display:block;object-fit:cover}
#estado{margin-top:8px;font-weight:600;color:var(--acc);min-height:1.4em}
.modos{display:grid;grid-template-columns:repeat(5,1fr);gap:6px;margin-bottom:10px}
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
<header><b>TOPO S14</b><span id="info">conectando…</span></header>
<div class="wrap">
 <div class="card"><img id="video" src="/video.mjpg" alt="video">
  <div id="estado">…</div><div class="chips" id="dets"></div></div>
 <div class="card">
  <div class="modos">
   <button data-m="quieto">Quieto</button><button data-m="manual">Manual</button>
   <button data-m="seguir">Seguir</button><button data-m="minero">Minero</button><button data-m="patrulla">Patrulla</button></div>
  <div class="row"><label>Objetivo</label><select id="obj"></select></div>
  <div id="joy"><div id="nub"></div></div>
  <div class="row"><label>Velocidad</label><input type="range" id="vel" min="20" max="100" value="70"></div>
  <div class="row"><label>Taladro</label><input type="range" id="tal" min="-100" max="100" value="0" step="5"><button id="tal0">0</button><button id="talmax">MAX</button></div>
  <div class="row"><label>Luces</label><select id="luz"><option value="1">Respirar</option><option value="2">Buscando</option><option value="3">Objetivo</option><option value="4">Fiesta</option><option value="5">Faros</option><option value="0">Apagadas</option></select><button id="show">Show</button></div>
  <button id="stop">PARADA DE EMERGENCIA</button>
  <div class="row"><button id="reset" style="flex:1">Borrar fallas</button></div>
  <div class="grid" style="margin-top:8px">
   <div class="kv"><small>Batería</small><b id="bat">–</b></div><div class="kv"><small>Distancia izq / der</small><b id="dist">–</b></div>
   <div class="kv"><small>Inclinación</small><b id="inc">–</b></div><div class="kv"><small>RPM izq / der / taladro</small><b id="rpm">–</b></div></div>
  <div class="chips" id="flags" style="margin-top:8px"></div>
 </div></div>
<script>
const $=s=>document.querySelector(s);
const post=d=>fetch('/api/cmd',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)}).catch(()=>{});
document.querySelectorAll('[data-m]').forEach(b=>b.onclick=()=>post({modo:b.dataset.m}));
$('#obj').onchange=e=>post({objetivo:e.target.value});
$('#vel').oninput=e=>post({vel:+e.target.value});
$('#tal').oninput=e=>post({taladro:+e.target.value});
$('#tal0').onclick=()=>{$('#tal').value=0;post({taladro:0})};
$('#talmax').onclick=()=>{$('#tal').value=100;post({taladro:100})};
$('#luz').onchange=e=>post({luces:+e.target.value});
$('#show').onclick=()=>post({show:1});
$('#stop').onclick=()=>post({emergencia:1});
$('#reset').onclick=()=>post({reset:1});
// joystick
const joy=$('#joy'),nub=$('#nub');let jx=0,jy=0,act=false;
function mv(e){const r=joy.getBoundingClientRect();let x=(e.clientX-r.left-r.width/2)/(r.width/2-37),y=-(e.clientY-r.top-r.height/2)/(r.height/2-37);
 const m=Math.hypot(x,y);if(m>1){x/=m;y/=m}jx=x;jy=y;nub.style.left=(68+x*68)+'px';nub.style.top=(68-y*68)+'px'}
joy.onpointerdown=e=>{act=true;joy.setPointerCapture(e.pointerId);mv(e)};
joy.onpointermove=e=>{if(act)mv(e)};
joy.onpointerup=joy.onpointercancel=()=>{act=false;jx=jy=0;nub.style.left='68px';nub.style.top='68px';post({joy:[0,0]})};
setInterval(()=>{if(act)post({joy:[jx,jy]})},100);
let objCargado=false;
async function tick(){try{const s=await (await fetch('/api/estado')).json();
 document.querySelectorAll('[data-m]').forEach(b=>b.classList.toggle('on',b.dataset.m==s.modo));
 if(!objCargado){$('#obj').innerHTML=s.objetivos.map(o=>`<option value="${o.id}">${o.nombre}</option>`).join('');objCargado=true}
 $('#obj').value=s.objetivo;
 $('#estado').textContent=s.estado+(s.error?'  ·  '+s.error:'');
 const t=s.tel;const pct=Math.max(0,Math.min(100,(t.vbat-9.9)/(12.6-9.9)*100));
 $('#bat').textContent=t.edad>2?'sin datos':`${t.vbat.toFixed(2)} V · ${pct.toFixed(0)}%`;
 const f=v=>v>=8190?'libre':(v/10).toFixed(0)+' cm';
 $('#dist').textContent=`${f(t.dist_izq)} / ${f(t.dist_der)}`;
 $('#inc').textContent=`${t.pitch.toFixed(0)}° / ${t.roll.toFixed(0)}°`;
 $('#rpm').textContent=`${t.rpm_izq.toFixed(0)} / ${t.rpm_der.toFixed(0)} / ${t.rpm_tal.toFixed(0)}`;
 $('#flags').innerHTML=s.banderas.map(b=>`<span>${b}</span>`).join('');
 $('#dets').innerHTML=s.detecciones.map(d=>`<span class="n">${d.clase} ${(d.conf*100).toFixed(0)}%</span>`).join('');
 $('#info').textContent=`IA ${s.fps} fps · salida ${s.salida.join(' / ')}`;
}catch(e){$('#info').textContent='sin conexión'}}
setInterval(tick,400);tick();
</script></body></html>"""


def crear_web(cerebro: Cerebro, vision: Vision):
    from flask import Flask, Response, jsonify, request
    app = Flask(__name__)

    @app.get("/")
    def index():
        return PAGINA

    @app.get("/api/estado")
    def estado():
        return jsonify(cerebro.estado_json())

    @app.post("/api/cmd")
    def comando():
        d = request.get_json(force=True, silent=True) or {}
        cerebro.cmd(d)
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
    ap = argparse.ArgumentParser(description="TOPO S14 — cerebro con IA")
    ap.add_argument("--sim", action="store_true", help="sin hardware: mundo, cámara y ESP32 simulados")
    ap.add_argument("--puerto", default=None, help="puerto del ESP32 (auto si se omite)")
    ap.add_argument("--modelo", default="yolo11n_ncnn_model", help="modelo YOLO (carpeta NCNN o .pt)")
    ap.add_argument("--rotar180", action="store_true", help="si la imagen sale al revés")
    ap.add_argument("--sin-ia", action="store_true", help="solo video y control manual")
    ap.add_argument("--web", type=int, default=8000, help="puerto de la página web")
    a = ap.parse_args()
    sim = MundoSim() if a.sim else None
    esp = EnlaceESP32(a.puerto, sim=sim)
    vision = Vision(a.modelo, a.rotar180, sim=sim, sin_ia=a.sin_ia)
    cerebro = Cerebro(esp, vision)
    app = crear_web(cerebro, vision)
    print(f"[web] abre http://topo.local:{a.web}  (o la IP de la Raspberry)")
    app.run(host="0.0.0.0", port=a.web, threaded=True, debug=False)


if __name__ == "__main__":
    main()
