#!/bin/sh
# YOLO26n NCNN HTTP preview only. Never arms or transmits flight commands.
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
unit=kr260-yolo-preview.service
action=${1:-status}
if [ "$(id -u)" -ne 0 ] && { [ "$action" = start ] || [ "$action" = stop ]; }; then
    exec sudo -- "$0" "$action"
fi
check_owner() {
    command=$(systemctl show "$unit" -p ExecStart --value 2>/dev/null || true)
    case "$command" in
        ''|*"$root/ncnn_preview.py"*) ;;
        *) echo 'Refusing to manage a detection unit owned by another entry' >&2; exit 1 ;;
    esac
}
case "$action" in
    start)
        check_owner
        for flight in kr260-position kr260-ego kr260-mission; do
            if systemctl is-active --quiet "$flight"; then
                echo "Refusing unvalidated vision load while $flight is active" >&2; exit 1
            fi
        done
        "$root/../camera/deploy/preview.sh" start
        if ! systemctl is-active --quiet "$unit"; then
            systemd-run --quiet --collect --unit="$unit" --uid=ubuntu \
                --property=Restart=no --property=TimeoutStopSec=20 \
                --working-directory="$root" /usr/bin/env \
                OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
                "$root/.venv/bin/python" "$root/ncnn_preview.py" --port 8081 --threads 2 --confidence 0.25
        fi
        attempt=0
        while [ "$attempt" -lt 40 ]; do
            if python3 -c 'import urllib.request,json,sys; s=json.load(urllib.request.urlopen("http://127.0.0.1:8081/status.json",timeout=.5)); sys.exit(0 if s.get("state")=="READY" and s.get("backend")=="NCNN CPU FP32" else 1)' 2>/dev/null; then
                echo 'YOLO26_NCNN_READY: detection page on port 8081; raw camera on 8080'
                exit 0
            fi
            if ! systemctl is-active --quiet "$unit"; then break; fi
            attempt=$((attempt+1)); sleep .5
        done
        journalctl -u "$unit" -n 20 --no-pager
        echo 'NCNN preview did not reach READY' >&2; exit 1
        ;;
    status)
        systemctl show "$unit" -p ActiveState -p MainPID -p ExecStart
        curl -fsS --max-time 3 http://127.0.0.1:8081/status.json; echo
        ;;
    logs) journalctl -u "$unit" -n 30 --no-pager ;;
    stop) check_owner; systemctl stop "$unit"; echo 'Detection stopped; raw camera unchanged' ;;
    *) echo 'Usage: vision.sh {start|status|logs|stop}' >&2; exit 2 ;;
esac
