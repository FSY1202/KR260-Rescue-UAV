#!/bin/sh
# Camera only. No flight processes or UART transmissions are started.
set -eu
if [ "$(id -u)" -ne 0 ]; then
    exec sudo -- "$0" "$@"
fi
camera_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
action=${1:-status}
if [ "$#" -gt 0 ]; then shift; fi
case "$action" in
    start)
        if ! printf '%s  %s\n' \
          4d7d287668a878fb61d96be5412cb4eee19ba3a8ef319aa25d4ac8f21b44a7e9 \
          /lib/firmware/xilinx/kr260_ov5640/kr260_ov5640.bit.bin | sha256sum -c --status; then
            echo 'Camera firmware checksum mismatch; refusing initialization' >&2
            exit 1
        fi
        found=0
        for addr in /sys/class/uio/uio*/maps/map0/addr; do
            [ -f "$addr" ] || continue
            if [ "$(cat "$addr")" = 0x00000000a0000000 ] || [ "$(cat "$addr")" = 0xa0000000 ]; then
                found=1
            fi
        done
        if [ "$found" -eq 0 ]; then
            for service in kr260-position kr260-ego kr260-mission; do
                if systemctl is-active --quiet "$service"; then
                    echo "Refusing PL reload while $service is active" >&2
                    exit 1
                fi
            done
            if fuser /dev/ttyS0 >/dev/null 2>&1; then
                echo 'Refusing PL reload: UART is in use' >&2
                exit 1
            fi
            active=$(xmutil listapps | awk '$NF ~ /^0,?$/ { print $1 }')
            case "$active" in
                k26-starter-kits|kr260_ov5640) xmutil unloadapp ;;
                '') ;; # No active accelerator; load without attempting unload.
                *) echo "Refusing to replace unknown accelerator: $active" >&2; exit 1 ;;
            esac
            xmutil loadapp kr260_ov5640
        fi
        exec python3 "$camera_dir/ov5640_camera.py" init "$@"
        ;;
    status|watch|stop|probe)
        exec python3 "$camera_dir/ov5640_camera.py" "$action" "$@"
        ;;
    *) echo 'Usage: camera.sh {start|status|watch|stop|probe} [camera options]' >&2; exit 2 ;;
esac
