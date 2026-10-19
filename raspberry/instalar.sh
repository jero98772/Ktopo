#!/bin/bash
# =====================================================================
#  TOPO S14 — instalación en Raspberry Pi 5 (Raspberry Pi OS Bookworm 64 bits)
#  Uso:  cd ~/topo/raspberry && bash instalar.sh
#  Tarda 15-30 minutos (descarga PyTorch y exporta el modelo YOLO).
# =====================================================================
set -e
cd "$(dirname "$0")"
DIR="$(pwd)"

echo ">> Paquetes del sistema (cámara, OpenCV, Flask, puerto serie)"
sudo apt update
sudo apt install -y python3-picamera2 python3-opencv python3-flask python3-serial python3-venv python3-pip

echo ">> Entorno de Python con acceso a picamera2"
python3 -m venv --system-site-packages ~/topo-venv
~/topo-venv/bin/pip install --upgrade pip
~/topo-venv/bin/pip install ultralytics ncnn

echo ">> Exportando YOLO11n a NCNN (el formato más rápido en la Pi)"
~/topo-venv/bin/yolo export model=yolo11n.pt format=ncnn imgsz=320

echo ">> Permiso para usar el puerto USB del ESP32"
sudo usermod -aG dialout "$USER"

echo ">> Servicio para que arranque solo al encender"
sed "s|__USUARIO__|$USER|g; s|__DIR__|$DIR|g; s|__PY__|$HOME/topo-venv/bin/python|g" topo-ia.service \
  | sudo tee /etc/systemd/system/topo-ia.service > /dev/null
sudo systemctl daemon-reload
sudo systemctl enable topo-ia

echo ""
echo "Listo. Reinicia con:  sudo reboot"
echo "Después abre en el celular:  http://$(hostname).local:8000"
echo "Ver mensajes:  journalctl -u topo-ia -f"
