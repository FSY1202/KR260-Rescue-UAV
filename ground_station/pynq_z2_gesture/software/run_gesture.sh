#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
# Keep the PL render node accessible even inside the XRDP mount namespace.
exec sudo nsenter -t 1 -m -- env XILINX_XRT=/usr DISPLAY="${DISPLAY:-}" \
    XAUTHORITY="${XAUTHORITY:-${HOME:-/home/xilinx}/.Xauthority}" \
    /usr/local/share/pynq-venv/bin/python3 "$root/software/gesture_demo.py" \
    --status-file /home/xilinx/gesture-status.json \
    --snapshot /home/xilinx/gesture-latest.jpg \
    --report /home/xilinx/gesture-demo-report.json "$@"
