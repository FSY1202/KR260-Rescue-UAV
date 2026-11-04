"""Unprivileged camera viewer; the recognition service owns camera and PL."""
import argparse
import json
from pathlib import Path
import subprocess
import time
from urllib.request import urlopen
import cv2
import numpy as np

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--button-control', type=Path)
    args = parser.parse_args()
    cv2.setNumThreads(1)
    visible = args.button_control is None
    last_sequence = 0
    window = False
    opened_at = 0
    next_wake = 0
    title = 'Gesture Recognition'
    status = Path('/run/pynq-xfce/camera-control-status.json')

    def record():
        if args.button_control:
            status.write_text(json.dumps({'visible': visible, 'sequence': last_sequence}))
            print('CAMERA_VIEW '+('SHOW' if visible else 'HIDE')+
                  ' sequence='+str(last_sequence), flush=True)

    record()
    try:
        while True:
            if args.button_control:
                try:
                    sequence = json.loads(args.button_control.read_text())['sequence']
                except (OSError, ValueError, KeyError):
                    sequence = last_sequence
                if sequence < last_sequence:
                    visible = False
                elif (sequence-last_sequence) % 2:
                    visible = not visible
                if sequence != last_sequence:
                    last_sequence = sequence
                    record()
            if not visible:
                if window:
                    cv2.destroyAllWindows()
                    cv2.waitKey(1)
                    window = False
                time.sleep(.05)
                continue
            if not window:
                cv2.namedWindow(title, cv2.WINDOW_NORMAL)
                if args.button_control:
                    cv2.setWindowProperty(title, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                window = True
                opened_at = time.monotonic()
                next_wake = 0
            if time.monotonic() >= next_wake:
                # Board buttons are not X input events: explicitly wake a
                # monitor that entered DPMS and keep the camera view awake.
                for command in (['/usr/bin/xset', 's', 'reset'],
                                ['/usr/bin/xset', 'dpms', 'force', 'on']):
                    subprocess.run(command, stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL, check=False)
                next_wake = time.monotonic()+60
            try:
                with urlopen('http://192.168.137.2:8080/frame.jpg', timeout=.5) as response:
                    frame = cv2.imdecode(np.frombuffer(response.read(), np.uint8), 1)
                if frame is None:
                    raise RuntimeError('No preview image')
            except Exception:
                frame = np.zeros((240, 320, 3), np.uint8)
                cv2.putText(frame, 'Waiting for camera...', (12, 115), 0, .6,
                            (255, 255, 255), 1)
            if args.button_control:
                frame = cv2.resize(frame, (800, 600), interpolation=cv2.INTER_LINEAR)
            cv2.imshow(title, frame)
            # The preview endpoint publishes at 5 FPS. Avoid repainting the
            # same JPEG at 20 FPS and competing with recognition on the ARM.
            key = cv2.waitKey(150) & 255
            # GTK builds can report VISIBLE=-1 (unsupported), and a new
            # fullscreen window may briefly report zero while being mapped.
            closed = (args.button_control is None and time.monotonic()-opened_at > 2
                      and cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) == 0)
            if key in (27, ord('q')) or closed:
                if not args.button_control:
                    break
                visible = False
                record()
    finally:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
