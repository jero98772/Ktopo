#!/bin/bash
# =====================================================================
#  Ktopo — install on a Raspberry Pi 5 (Raspberry Pi OS Bookworm, 64-bit)
#  Usage:  cd ~/ktopo/raspberry && bash install.sh
#  Takes 15-30 minutes (downloads PyTorch and exports the YOLO model).
# =====================================================================
set -e
cd "$(dirname "$0")"
DIR="$(pwd)"

echo ">> System packages (camera, OpenCV, Flask, serial port)"
sudo apt update
sudo apt install -y python3-picamera2 python3-opencv python3-flask python3-serial python3-venv python3-pip

echo ">> Python environment with access to picamera2"
python3 -m venv --system-site-packages ~/ktopo-venv
~/ktopo-venv/bin/pip install --upgrade pip
~/ktopo-venv/bin/pip install ultralytics ncnn

echo ">> Exporting YOLO11n to NCNN (the fastest format on the Pi)"
~/ktopo-venv/bin/yolo export model=yolo11n.pt format=ncnn imgsz=320

echo ">> Permission to use the ESP32's USB port"
sudo usermod -aG dialout "$USER"

echo ">> Service so it starts on power-up"
sed "s|__USER__|$USER|g; s|__DIR__|$DIR|g; s|__PY__|$HOME/ktopo-venv/bin/python|g" ktopo-ai.service \
  | sudo tee /etc/systemd/system/ktopo-ai.service > /dev/null
sudo systemctl daemon-reload
sudo systemctl enable ktopo-ai

echo ""
echo "Done. Reboot with:  sudo reboot"
echo "Then open on your phone:  http://$(hostname).local:8000"
echo "See the logs:  journalctl -u ktopo-ai -f"
