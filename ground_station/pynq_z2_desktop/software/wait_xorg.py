import os
import subprocess
import time
for attempt in range(40):
    check = subprocess.run(['/usr/bin/xrandr', '--query'], stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL)
    if check.returncode == 0:
        break
    time.sleep(.5)
else:
    raise SystemExit('HDMI X server did not become ready')
