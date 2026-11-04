#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
python3 -m venv --system-site-packages "$root/yolo_standard/.venv"
"$root/yolo_standard/.venv/bin/python" -m pip install 'ncnn==1.0.20261020'
# OpenCV and numpy are provided by python3-opencv and python3-numpy on Ubuntu.
