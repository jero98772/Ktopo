# TOPO S14 — carro perforador impreso en 3D con IA

Todo lo necesario para construirlo: piezas 3D, firmware del ESP32 y software de visión para la Raspberry Pi 5.
La guía completa (visor 3D, electrónica, diagrama de conexiones y orden de armado) está en la página del proyecto.

## Carpetas

| Carpeta | Contenido |
|---|---|
| `cad/stl/` | 15 piezas listas para imprimir, ya orientadas y sin soportes (caben en 220 × 220 mm) |
| `cad/step/` | Las mismas piezas en STEP para editarlas en Fusion 360, FreeCAD u Onshape, más `TOPO_S14_ensamble.step` |
| `cad/topo_cad.py` | Modelo paramétrico en CadQuery. Cambia los parámetros del inicio y ejecuta `python3 topo_cad.py` |
| `cad/TOPO_S14.glb` | Ensamblaje completo para verlo en cualquier visor 3D |
| `firmware/topo_esp32/` | Firmware del ESP32 (Arduino IDE) |
| `firmware/simulador_pc/` | Pruebas del firmware en el PC: `g++ -std=gnu++17 -DSIM -I. sim.cpp -o sim && ./sim` |
| `raspberry/` | Cerebro con IA (`topo_ia.py`), instalador y servicio de arranque |

## Cantidades a imprimir

| Pieza | Cant. | Material |
|---|---|---|
| chasis_delantero, chasis_trasero | 1 + 1 | PETG |
| casco_delantero, casco_trasero, cabina | 1 + 1 + 1 | PLA o PETG |
| collar_portasatelites, cortador_trasero, cono_delantero | 1 + 1 + 1 | PETG |
| engranaje_A | 1 | PETG o nylon, 100 % relleno |
| satelite | 4 | PETG o nylon, 100 % relleno |
| espaciador_eje, difusor_led, bandeja_electronica | 1 + 1 + 1 | PLA / PETG transparente / PLA |
| rueda_rin | 4 | PETG |
| rueda_llanta_TPU | 4 | TPU 95A |

Filamento total aproximado: 1.1 kg.

## Arranque rápido

1. **ESP32**: en Arduino IDE instala el núcleo *esp32* (Espressif) y las librerías **VL53L0X** (Pololu) y **Adafruit NeoPixel**.
   Placa: *ESP32 Dev Module*. Sube `firmware/topo_esp32/topo_esp32.ino`.
   Prueba en el monitor serie (115200): `V 30 30` mueve las ruedas, `T 40` el taladro, `X` es parada de emergencia y `R` la quita.
2. **Raspberry Pi 5**: graba Raspberry Pi OS de 64 bits con el nombre de equipo `topo`, copia esta carpeta y ejecuta
   `bash raspberry/instalar.sh`. Al reiniciar abre `http://topo.local:8000` en el celular.
3. **Sin hardware**: `python3 raspberry/topo_ia.py --sim` y abre `http://localhost:8000` (necesita `pip install flask opencv-python numpy`).

## Ajustes que quizá debas cambiar en el firmware

- `INVERTIR_IZQ`, `INVERTIR_DER`, `INVERTIR_TAL`: si un lado o el taladro gira al revés.
- `RELACION_TRACCION`, `RELACION_TALADRO`: la relación de la caja de tus motores (viene en la etiqueta).
- `USAR_ENCODER_TALADRO = false` si no conectas el encoder del taladro.
- `OBST_MM`: distancia de frenado medida desde el frente del chasis (la punta sobresale unos 18 cm).

## Seguridad

El taladro atrapa dedos y cables aunque sea de plástico: apágalo antes de tocarlo. Carga la batería fuera del carro
y vigilada. Haz la primera prueba con el fusible puesto y las ruedas en el aire.
