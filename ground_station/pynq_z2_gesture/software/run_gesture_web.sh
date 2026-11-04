#!/bin/sh
set -eu
script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec bash "$script_dir/run_gesture.sh" --headless \
    --web-host "${GESTURE_WEB_HOST:-192.168.137.2}" --web-port 8080 \
    --web-fps 5 --snapshot-interval 0 "$@"
