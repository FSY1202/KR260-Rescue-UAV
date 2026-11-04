#!/bin/sh
# Raw camera preview only. No YOLO, ROS, arming, or FC messages.
set -eu
camera_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
unit=kr260-camera-preview.service
action=${1:-status}
if [ "$#" -gt 0 ]; then shift; fi
if [ "$(id -u)" -ne 0 ] && { [ "$action" = start ] || [ "$action" = stop ]; }; then
    exec sudo -- "$0" "$action" "$@"
fi
check_owner() {
    command=$(systemctl show "$unit" -p ExecStart --value 2>/dev/null || true)
    case "$command" in
        ''|*"$camera_dir/camera_preview.py"*) ;;
        *) echo 'Refusing to manage a preview service owned by another application' >&2; exit 1 ;;
    esac
}
case "$action" in
    start)
        check_owner
        if ! systemctl is-active --quiet "$unit"; then
            # Adding CPU/image-copy load to an active flight has not been tested.
            for flight in kr260-position kr260-ego kr260-mission; do
                if systemctl is-active --quiet "$flight"; then
                    echo "Refusing camera preview setup during active $flight; keep flight services stopped for this bench version" >&2
                    exit 1
                fi
            done
            "$camera_dir/camera.sh" start --seconds 1 >/dev/null
            systemctl reset-failed "$unit" 2>/dev/null || true
            systemd-run --quiet --collect --unit="$unit" --property=Restart=no --property=TimeoutStopSec=20 \
                /usr/bin/python3 "$camera_dir/camera_preview.py" serve --host 0.0.0.0 --port 8080 --fps 3
        fi
        attempt=0
        while [ "$attempt" -lt 40 ]; do
            if python3 -c 'import json,urllib.request,sys; s=json.load(urllib.request.urlopen("http://127.0.0.1:8080/status.json",timeout=.4)); sys.exit(0 if s.get("state")=="READY" and not s.get("yolo_enabled") else 1)' 2>/dev/null; then
                echo 'CAMERA_READY: http://<board-ip>:8080'
                echo 'Raw camera only; YOLO is disabled. SSH disconnect does not stop preview.'
                exit 0
            fi
            if ! systemctl is-active --quiet "$unit"; then break; fi
            attempt=$((attempt+1)); sleep 0.5
        done
        journalctl -u "$unit" -n 15 --no-pager
        echo 'Camera preview did not reach READY' >&2; exit 1
        ;;
    status)
        systemctl show "$unit" -p ActiveState -p MainPID
        python3 -c 'import json,urllib.request; print(json.dumps(json.load(urllib.request.urlopen("http://127.0.0.1:8080/status.json",timeout=2)),indent=2))'
        ;;
    logs) journalctl -u "$unit" -n 50 --no-pager ;;
    stop) check_owner; systemctl stop "$unit"; echo 'Preview stopped; camera PL/UART and other services were not stopped' ;;
    snapshot)
        out=${1:-"$camera_dir/capture_$(date +%Y%m%d_%H%M%S).jpg"}
        if [ -e "$out" ]; then echo "Refusing to overwrite $out" >&2; exit 1; fi
        curl -f --max-time 5 http://127.0.0.1:8080/raw.jpg -o "$out"
        echo "$out"
        ;;
    *) echo 'Usage: preview.sh {start|status|logs|stop|snapshot [file.jpg]}' >&2; exit 2 ;;
esac
