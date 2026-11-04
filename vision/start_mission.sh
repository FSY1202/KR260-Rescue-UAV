#!/bin/sh
# Start raw camera first, then one 320-input inference thread. No flight commands.
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
bash "$root/camera/deploy/preview.sh" start
export KR260_VISION_PROFILE=mission
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
exec "$root/yolo_standard/.venv/bin/python" "$root/yolo_fast/ncnn_preview.py" --port 8081 --threads 1 --confidence .25
