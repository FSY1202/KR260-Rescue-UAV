#!/bin/sh
set -eu
servo_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
# Initialization remains disabled. Motion requires the explicit "move" action.
if [ "$#" -eq 0 ]; then set -- status; fi
if [ "$1" = move ]; then
    shift
    exec python3 "$servo_dir/servo_move.py" "$@"
fi
if [ "$1" = load ]; then
    shift
    exec python3 "$servo_dir/load_servo.py" "$@"
fi
exec python3 "$servo_dir/servo_init.py" "$@"
