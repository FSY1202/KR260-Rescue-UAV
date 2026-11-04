#!/bin/bash
set -eu
desktop_software=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
gesture_software=$(CDPATH= cd -- "$desktop_software/../../pynq_z2_gesture/software" && pwd)
native_flags=()
case "$(uname -m)" in armv7*) native_flags=(-mfpu=neon);; esac
gcc -O3 -shared -fPIC -ffp-contract=off -Wall -Wextra "${native_flags[@]}" \
    -o "$gesture_software/_hand_geometry.new.so" "$gesture_software/hand_native.c"
mv -f "$gesture_software/_hand_geometry.new.so" "$gesture_software/_hand_geometry.so"
gcc -O3 -shared -fPIC -Wall -Wextra "${native_flags[@]}" \
    -o "$desktop_software/_live_x11.new.so" "$desktop_software/live_x11.c" \
    $(pkg-config --cflags --libs libswscale libavutil x11 xext)
mv -f "$desktop_software/_live_x11.new.so" "$desktop_software/_live_x11.so"
echo NATIVE_LIBRARIES_BUILT
