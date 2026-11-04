#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cc -O2 -fPIC -shared -Wall -Wextra "$root/camera_mmio.c" -o "$root/libcamera_mmio.so"
