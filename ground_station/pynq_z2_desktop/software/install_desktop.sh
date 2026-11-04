#!/bin/bash
set -eu
desktop_project=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
if [ "$(id -u)" -ne 0 ]; then exec sudo bash "$0" "$@"; fi
test -s "$desktop_project/output/desktop_gesture.bit"
test -s "$desktop_project/output/desktop_gesture.hwh"
test -s "$desktop_project/kernel/pynq_hdmi_drm.ko"
test "$(modinfo -F vermagic "$desktop_project/kernel/pynq_hdmi_drm.ko" | cut -d ' ' -f1)" = "$(uname -r)"
test -s /home/xilinx/pynq_z2_gesture/software/gesture-web.service
bash "$desktop_project/software/build_native.sh"
bash "$desktop_project/software/build_flight_native.sh"
mkdir -p "$desktop_project/backups" /etc/systemd/system/gesture-web.service.d
cp -pn /home/xilinx/Desktop/Gesture-Recognition.desktop "$desktop_project/backups/" 2>/dev/null || true
systemctl stop pynq-desktop.target 2>/dev/null || true
systemctl stop gesture-web.service pynq-hdmi-session.service pynq-hdmi-xorg.service 2>/dev/null || true
# The hardware owner's rmmod may still be finishing after its unit exits.
# Wait for all DRM users/references instead of racing a second unload.
for desktop_attempt in {1..25}; do
    if ! grep -q '^pynq_hdmi_drm ' /proc/modules; then break; fi
    rmmod pynq_hdmi_drm 2>/dev/null || true
    sleep .2
done
if grep -q '^pynq_hdmi_drm ' /proc/modules; then
    echo 'HDMI DRM is still in use; stop its clients before replacing the FPGA image' >&2
    exit 1
fi
systemctl stop pynq-hdmi-test.service 2>/dev/null || true
for desktop_unit in pynq-desktop-hardware.service pynq-desktop-xorg.service pynq-desktop-session.service pynq-desktop-camera.service map-display.service flight-status.service pynq-desktop.target; do
    install -m644 "$desktop_project/software/$desktop_unit" "/etc/systemd/system/$desktop_unit"
done
install -m644 "$desktop_project/software/gesture-hdmi.conf" /etc/systemd/system/gesture-web.service.d/desktop.conf
install -o xilinx -g xilinx -m755 "$desktop_project/software/Gesture-Recognition.desktop" /home/xilinx/Desktop/Gesture-Recognition.desktop
# Deploy map data files (map.svg and trajectory.json) to board data directory.
install -d -o xilinx -g xilinx -m755 /home/xilinx/pynq_z2_desktop/data
if [ -f "$desktop_project/data/trajectory.json" ]; then
    if [ "$(readlink -f "$desktop_project/data/trajectory.json")" != /home/xilinx/pynq_z2_desktop/data/trajectory.json ]; then
        install -o xilinx -g xilinx -m644 "$desktop_project/data/trajectory.json" /home/xilinx/pynq_z2_desktop/data/trajectory.json
    fi
fi
if [ -f "$desktop_project/data/map.svg" ]; then
    if [ "$(readlink -f "$desktop_project/data/map.svg")" != /home/xilinx/pynq_z2_desktop/data/map.svg ]; then
        install -o xilinx -g xilinx -m644 "$desktop_project/data/map.svg" /home/xilinx/pynq_z2_desktop/data/map.svg
    fi
fi
if [ -d "$desktop_project/data/flight_status" ]; then
    if [ "$(readlink -f "$desktop_project/data/flight_status")" != /home/xilinx/pynq_z2_desktop/data/flight_status ]; then
        install -d -o xilinx -g xilinx -m755 /home/xilinx/pynq_z2_desktop/data/flight_status
        cp -a "$desktop_project/data/flight_status/." /home/xilinx/pynq_z2_desktop/data/flight_status/
    fi
fi
if [ -f "$desktop_project/data/flight_demo.json" ]; then
    if [ "$(readlink -f "$desktop_project/data/flight_demo.json")" != /home/xilinx/pynq_z2_desktop/data/flight_demo.json ]; then
        install -o xilinx -g xilinx -m644 "$desktop_project/data/flight_demo.json" /home/xilinx/pynq_z2_desktop/data/flight_demo.json
    fi
fi
install -m644 /home/xilinx/pynq_z2_gesture/software/gesture-web.service /etc/systemd/system/gesture-web.service
systemctl daemon-reload
systemctl reset-failed
systemctl start pynq-desktop.target
systemctl enable pynq-desktop.target
echo DESKTOP_INSTALLED
systemctl is-active pynq-desktop-hardware pynq-desktop-xorg pynq-desktop-session gesture-web map-display flight-status
