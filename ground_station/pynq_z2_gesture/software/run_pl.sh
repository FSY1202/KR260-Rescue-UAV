#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
exec sudo env XILINX_XRT=/usr DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}" \
    /usr/local/share/pynq-venv/bin/python3 "$root/software/gesture.py" \
    --backend pl --bitfile "$root/output/gesture.bit" "$@"
