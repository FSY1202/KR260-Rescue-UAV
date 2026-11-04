#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
exec sudo env XILINX_XRT=/usr /usr/local/share/pynq-venv/bin/python3 "$root/software/gesture_fast.py" "$@"
