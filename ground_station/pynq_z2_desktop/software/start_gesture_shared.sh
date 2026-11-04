#!/bin/bash
set -eu
if [ ! -e /dev/video0 ]; then
    echo 'HDMI desktop ready; waiting for USB camera /dev/video0'
    while [ ! -e /dev/video0 ]; do sleep 2; done
fi
exec /bin/bash /home/xilinx/pynq_z2_gesture/software/run_gesture_web.sh \
    --bitfile /home/xilinx/pynq_z2_desktop/output/desktop_gesture.bit --reuse-overlay \
    --capture-scale 2 --web-jpeg-quality 90 \
    --local-preview /run/pynq-desktop/camera-live.frame --lazy-web-preview
