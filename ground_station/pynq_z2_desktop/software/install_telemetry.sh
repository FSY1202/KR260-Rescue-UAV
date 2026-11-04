#!/bin/bash
# Optional mode switch. Does not load PL or change the HDMI hardware owner.
set -eu
mode=${1:-}
case "$mode" in live|demo) ;; *) echo 'Usage: sudo bash install_telemetry.sh live|demo' >&2; exit 2;; esac
if [ "$(id -u)" -ne 0 ]; then echo 'Run this installer with sudo' >&2; exit 1; fi
software=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
if [ "$mode" = live ]; then
    if [ ! -f /etc/default/pynq-telemetry ]; then
        echo 'First configure /etc/default/pynq-telemetry using telemetry.env.example' >&2
        exit 1
    fi
    install -m644 "$software/telemetry-receiver.service" /etc/systemd/system/telemetry-receiver.service
    for unit in map-display flight-status; do
        install -d -m755 "/etc/systemd/system/$unit.service.d"
        install -m644 "$software/telemetry-ui.conf" "/etc/systemd/system/$unit.service.d/telemetry.conf"
    done
    systemctl daemon-reload
    systemctl enable --now telemetry-receiver.service
else
    for unit in map-display flight-status; do
        rm -f "/etc/systemd/system/$unit.service.d/telemetry.conf"
    done
    systemctl disable --now telemetry-receiver.service 2>/dev/null || true
    systemctl daemon-reload
fi
systemctl restart map-display.service flight-status.service
echo "TELEMETRY_MODE=$mode"
