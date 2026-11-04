#!/bin/sh
# Independent visual preview. Original flight/vision files are not replaced.
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
original=$(CDPATH= cd -- "$root/../yolo_standard" && pwd)
unit=kr260-yolo-preview.service
action=${1:-status}
if [ "$(id -u)" -ne 0 ] && { [ "$action" = start ] || [ "$action" = stop ]; }; then
    exec sudo -- "$0" "$action"
fi
command=$(systemctl show "$unit" -p ExecStart --value 2>/dev/null || true)
case "$action" in
    start)
        for flight in kr260-position kr260-ego kr260-mission kr260-drop kr260-drop-io kr260-rescue kr260-rescue-io kr260-rescue-servo; do
            if systemctl is-active --quiet "$flight"; then
                echo "Stop the active flight/localization task on ground before fast preview: $flight" >&2
                exit 1
            fi
        done
        case "$command" in
            ''|*"$root/ncnn_preview.py"*|*"$original/ncnn_preview.py"*) ;;
            *) echo 'Detection service belongs to another entry; refusing to replace it' >&2; exit 1 ;;
        esac
        "$root/../camera/deploy/preview.sh" start
        case "$command" in
            *"$root/ncnn_preview.py"*) ;;
            *)
                if systemctl is-active --quiet "$unit"; then systemctl stop "$unit"; fi
                systemctl reset-failed "$unit" 2>/dev/null || true
                systemd-run --quiet --collect --unit="$unit" --uid=ubuntu \
                    --property=Restart=no --property=TimeoutStopSec=20 \
                    --working-directory="$root" /usr/bin/env \
                    OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
                    "$original/.venv/bin/python" "$root/ncnn_preview.py" \
                    --port 8081 --threads 2 --confidence 0.25 --fast-preview
                ;;
        esac
        attempt=0
        while [ "$attempt" -lt 40 ]; do
            if python3 -c 'import json,urllib.request,sys; s=json.load(urllib.request.urlopen("http://127.0.0.1:8081/status.json",timeout=.5)); sys.exit(0 if s.get("state")=="READY" and s.get("imgsz")==384 and s.get("max_angle_passes")==1 else 1)' 2>/dev/null; then
                echo 'YOLO_FAST_READY: 384 input / one inference per frame / http://<board-ip>:8081/'
                exit 0
            fi
            if ! systemctl is-active --quiet "$unit"; then break; fi
            attempt=$((attempt+1)); sleep .5
        done
        journalctl -u "$unit" -n 15 --no-pager
        echo 'Fast preview did not reach READY' >&2; exit 1
        ;;
    stop)
        case "$command" in
            *"$root/ncnn_preview.py"*) systemctl stop "$unit" ;;
            '') ;;
            *) echo 'Another visual entry is running; nothing stopped' >&2; exit 1 ;;
        esac
        echo 'Fast YOLO stopped; camera unchanged'
        ;;
    status)
        systemctl show "$unit" -p ActiveState -p MainPID -p ExecStart
        curl -fsS --max-time 3 http://127.0.0.1:8081/status.json; echo
        ;;
    logs) journalctl -u "$unit" -n 25 --no-pager ;;
    *) echo 'Usage: fast.sh {start|stop|status|logs}' >&2; exit 2 ;;
esac
