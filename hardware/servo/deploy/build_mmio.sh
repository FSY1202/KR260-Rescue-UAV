#!/bin/sh
set -eu
servo_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
# Builds only this helper. No device access, PL load, ROS or service changes.
gcc -O2 -std=c11 -Wall -Wextra -Werror -fPIC -shared \
    "$servo_dir/servo_mmio.c" -o "$servo_dir/libservo_mmio.so"
