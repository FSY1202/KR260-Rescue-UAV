"""Known-good HDMI pipeline test, before connecting Linux DRM scanout."""
import json
from pathlib import Path
import signal
import time
import faulthandler
import cv2
import numpy as np
from pynq import Overlay
from pynq.lib.video import VideoMode, PIXEL_BGR

stop = False
def finish(signum, frame):
    global stop
    stop = True
signal.signal(signal.SIGTERM, finish)
signal.signal(signal.SIGINT, finish)
faulthandler.dump_traceback_later(15, repeat=True)
print('LOADING_BASE', flush=True)
base = Overlay('/usr/local/share/pynq-venv/lib/python3.10/site-packages/pynq/overlays/base/base.bit')
print('BASE_LOADED', flush=True)
hdmi = base.video.hdmi_out
try:
    hdmi.configure(VideoMode(800, 600, 32))
    print('HDMI_CONFIGURED', flush=True)
    hdmi.color_convert.colorspace = PIXEL_BGR.out_color
    hdmi.start()
    print('DMA_STARTED', flush=True)
    image = hdmi.newframe()
    image[:] = 0
    colors = [(255,255,255),(0,255,255),(255,255,0),(0,255,0),
              (255,0,255),(0,0,255),(255,0,0),(0,0,0)]
    for i,color in enumerate(colors):
        image[:,i*100:(i+1)*100,:3] = color
    text = np.ascontiguousarray(image[:,:,:3])
    cv2.rectangle(text,(0,220),(799,375),(25,25,25),-1)
    cv2.putText(text,'PYNQ-Z2 HDMI OUTPUT OK',(38,285),0,1.35,(255,255,255),2)
    cv2.putText(text,'800 x 600 | Linux desktop setup',(55,335),0,.85,(255,255,255),2)
    image[:,:,:3] = text
    hdmi.writeframe(image)
    print('FRAME_SUBMITTED', flush=True)
    faulthandler.cancel_dump_traceback_later()
    info={key:{'phys_addr':v['phys_addr'],'addr_range':v['addr_range']} for key,v in base.ip_dict.items() if any(s in key for s in ['vdma','vtc_out','dynclk'])}
    Path('/home/xilinx/hdmi_setup/video_ip.json').write_text(json.dumps(info,indent=2))
    print('HDMI_TEST_RUNNING '+json.dumps(info),flush=True)
    while not stop:
        time.sleep(.2)
finally:
    hdmi.close()
