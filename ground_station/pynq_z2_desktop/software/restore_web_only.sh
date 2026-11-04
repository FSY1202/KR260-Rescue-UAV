#!/bin/bash
set -eu
desktop_project=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
if [ "$(id -u)" -ne 0 ]; then exec sudo bash "$0" "$@"; fi
systemctl disable --now pynq-desktop.target
systemctl stop gesture-web pynq-desktop-camera pynq-desktop-session pynq-desktop-xorg pynq-desktop-hardware
rm -f /etc/systemd/system/gesture-web.service.d/desktop.conf
if [ -s "$desktop_project/backups/Gesture-Recognition.desktop" ]; then
    install -o xilinx -g xilinx -m755 "$desktop_project/backups/Gesture-Recognition.desktop" /home/xilinx/Desktop/Gesture-Recognition.desktop
fi
systemctl daemon-reload
systemctl start getty@tty1.service
systemctl restart gesture-web
echo ORIGINAL_GESTURE_WEB_RESTORED
