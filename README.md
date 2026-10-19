# Ktopo — 3D-printed drilling rover with AI

Everything you need to build it: 3D parts, ESP32 firmware and vision software for the Raspberry Pi 5.
The full guide (3D viewer, electronics, wiring diagram and assembly order) is on the project page.

## Folders

| Folder | Contents |
|---|---|
| `cad/stl/` | 15 parts ready to print, already oriented, no supports needed (they fit a 220 × 220 mm bed) |
| `cad/step/` | The same parts as STEP files to edit in Fusion 360, FreeCAD or Onshape, plus `Ktopo_assembly.step` |
| `cad/ktopo_cad.py` | Parametric CadQuery model. Change the parameters at the top and run `python3 ktopo_cad.py` |
| `cad/Ktopo.glb` | Full assembly to open in any 3D viewer |
| `firmware/ktopo_esp32/` | ESP32 firmware (Arduino IDE) |
| `firmware/pc_simulator/` | Firmware tests on your PC: `g++ -std=gnu++17 -DSIM -I. sim.cpp -o sim && ./sim` |
| `raspberry/` | AI brain (`ktopo_ai.py`), installer and start-up service |
| `drawings/` | Wiring diagram, drill mechanism and software architecture (PNG and SVG), plus renders |

## How many to print

| Part | Qty | Material |
|---|---|---|
| chassis_front, chassis_rear | 1 + 1 | PETG |
| hull_front, hull_rear, cab | 1 + 1 + 1 | PLA or PETG |
| collar_carrier, rear_cutter, front_cone | 1 + 1 + 1 | PETG |
| gear_A | 1 | PETG or nylon, 100 % infill |
| spider_gear | 4 | PETG or nylon, 100 % infill |
| shaft_spacer, led_diffuser, electronics_deck | 1 + 1 + 1 | PLA / clear PETG / PLA |
| wheel_rim | 4 | PETG |
| wheel_tire_TPU | 4 | TPU 95A |

Approximate total filament: 1.2 kg.

## Quick start

1. **ESP32**: in the Arduino IDE install the *esp32* core (Espressif) and the **VL53L0X** (Pololu) and **Adafruit NeoPixel** libraries.
   Board: *ESP32 Dev Module*. Upload `firmware/ktopo_esp32/ktopo_esp32.ino`.
   Test from the Serial Monitor (115200): `V 30 30` turns the wheels, `T 40` the drill, `X` is the emergency stop and `R` clears it.
2. **Raspberry Pi 5**: flash Raspberry Pi OS 64-bit with the hostname `ktopo`, copy this folder over and run
   `bash raspberry/install.sh`. After the reboot, open `http://ktopo.local:8000` on your phone.
3. **No hardware**: `python3 raspberry/ktopo_ai.py --sim` and open `http://localhost:8000` (needs `pip install flask opencv-python numpy`).

## Bluetooth

Pair your phone with **Ktopo**. It accepts the same letters as the old app:
`A` forward, `a` backward, `i` / `d` pivot left / right, `I` / `D` spin in place, `S` stop the wheels,
`M` / `m` drill at max / off, `1` demo show, `0` everything off and control back to the AI.

## Firmware settings you may need to change

- `INVERT_LEFT`, `INVERT_RIGHT`, `INVERT_DRILL`: if a side or the drill turns the wrong way.
- `DRIVE_GEAR_RATIO`, `DRILL_GEAR_RATIO`: the gear ratio of your motors (printed on the label).
- `USE_DRILL_ENCODER = false` if you don't wire the drill encoder.
- `OBST_MM`: braking distance measured from the front of the chassis (the drill tip sticks out about 18 cm).

## Safety

The drill grabs fingers and cables even though it is plastic: turn it off before touching it. Charge the battery
outside the rover and under supervision. Do the first test with the fuse in and the wheels off the ground.
