#!/usr/bin/env python3
"""Raw DVP/PL camera preview; inference uses the independent YOLO26 service."""
import argparse
import http.server
import json
import os
from pathlib import Path
import pwd
import signal
import threading
import time
import uuid
import cv2
import numpy as np
from frame_grabber import FrameGrabber
from capture_pump import CapturePump
from vision_core import rgb565_to_bgr

PAGE = b"""<!doctype html><meta charset=utf-8><title>KR260 Camera</title>
<h1>KR260 OV5640 / DVP-PL-PS</h1><img src='/stream.mjpg' width='640'>
<p><a href='/raw.jpg'>Raw image</a> | <a href='/status.json'>Capture status</a></p>"""


def drop_root():
    if os.getuid() == 0:
        user = pwd.getpwnam('ubuntu')
        os.setgroups([]); os.setgid(user.pw_gid); os.setuid(user.pw_uid)


class Preview:
    def __init__(self, grabber, detector, fps):
        self.grabber, self.detector, self.fps = grabber, detector, fps
        self.cv = threading.Condition()
        self.stop = threading.Event()
        self.jpeg, self.raw = None, None
        self.serial = 0
        self.session = uuid.uuid4().hex
        self.capture_pump = None
        self.updated = 0
        self.info = dict(state='STARTING',yolo_enabled=detector is not None,detections=[])

    def produce(self):
        pump = CapturePump(self.grabber, self.stop)
        self.capture_pump = pump
        pump.start()
        try:
            self.consume(pump)
        finally:
            pump.close()

    def consume(self, pump):
        previous = None
        while not self.stop.is_set():
            start = time.monotonic()
            try:
                captured = pump.get()
                if captured is None:
                    break
                pixels, info = captured
                convert_started = time.monotonic()
                image = rgb565_to_bgr(pixels)
                convert_ms = (time.monotonic()-convert_started)*1000
                detections, inference_ms = [], 0
                display = image
                encode_started = time.monotonic()
                raw_ok, raw = cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,90])
                if self.detector:
                    ok, jpg = cv2.imencode('.jpg',display,[cv2.IMWRITE_JPEG_QUALITY,85])
                else:
                    ok, jpg = raw_ok, raw
                if not ok or not raw_ok:
                    raise RuntimeError('JPEG encoding failed')
                raw_bytes = raw.tobytes()
                jpg_bytes = jpg.tobytes() if self.detector else raw_bytes
                now = time.monotonic()
                info.update(state='READY',yolo_enabled=self.detector is not None,
                            detections=detections,inference_ms=inference_ms,
                            fps=1/(now-previous) if previous else 0,error=None,
                            convert_ms=convert_ms,encode_ms=(now-encode_started)*1000,
                            prefetch_frames=1)
                previous = now
                with self.cv:
                    self.jpeg, self.raw = jpg_bytes, raw_bytes
                    self.updated=info['capture_completed_monotonic_ns']/1e9
                    self.serial+=1; info['preview_frames']=self.serial
                    self.info=info; self.cv.notify_all()
            except Exception as exc:
                with self.cv:
                    self.info.update(state='ERROR',error=str(exc),detections=[])
                    self.jpeg=None; self.raw=None; self.cv.notify_all()
                print(json.dumps(dict(event='capture_error',reason=str(exc))),flush=True)
                self.stop.wait(0.25)
            self.stop.wait(max(0,1/self.fps-(time.monotonic()-start)))

    def status(self):
        with self.cv:
            result=dict(self.info)
            result['age_ms']=(time.monotonic()-self.updated)*1000 if self.updated else None
            return result


def handler_for(preview):
    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def respond(self, code, content_type, body, metadata=None):
            self.send_response(code); self.send_header('Content-Type',content_type)
            self.send_header('Cache-Control','no-store'); self.send_header('Content-Length',str(len(body)))
            if metadata:
                self.send_header('X-Camera-Session', preview.session)
                self.send_header('X-Camera-Frame-Seq', str(metadata['frame_seq']))
                self.send_header('X-Camera-Monotonic-Ns', str(metadata['capture_completed_monotonic_ns']))
            self.end_headers(); self.wfile.write(body)

        def do_GET(self):
            try:
                if self.path=='/':
                    self.respond(200,'text/html; charset=utf-8',PAGE)
                elif self.path=='/status.json':
                    self.respond(200,'application/json',json.dumps(preview.status()).encode())
                elif self.path in ('/snapshot.jpg','/raw.jpg'):
                    with preview.cv:
                        data = preview.raw if self.path=='/raw.jpg' else preview.jpeg
                        metadata = dict(preview.info)
                        fresh = preview.updated and time.monotonic()-preview.updated<5
                    if data is None or not fresh:
                        self.respond(503,'text/plain',b'No fresh camera image')
                    else:
                        self.respond(200,'image/jpeg',data,metadata)
                elif self.path=='/stream.mjpg':
                    self.send_response(200)
                    self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame')
                    self.send_header('Cache-Control','no-store'); self.end_headers()
                    last=-1
                    while not preview.stop.is_set():
                        with preview.cv:
                            preview.cv.wait_for(lambda: preview.serial!=last or preview.stop.is_set(),timeout=2)
                            if preview.stop.is_set(): break
                            if preview.serial==last or preview.jpeg is None:
                                if preview.updated and time.monotonic()-preview.updated>5: break
                                continue
                            last=preview.serial; data=preview.jpeg
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(data)).encode()+b'\r\n\r\n'+data+b'\r\n')
                        self.wfile.flush()
                else:
                    self.respond(404,'text/plain',b'Not found')
            except (BrokenPipeError,ConnectionResetError,TimeoutError):
                pass
    return Handler


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['grab','serve'])
    parser.add_argument('--host',default='0.0.0.0'); parser.add_argument('--port',type=int,default=8080)
    parser.add_argument('--fps',type=float,default=5)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if not 0<args.fps<=30 or not 1024<=args.port<=65535:
        parser.error('fps 0..30 and unprivileged port 1024..65535 required')
    cv2.setNumThreads(1)
    grabber=FrameGrabber()
    detector=None
    drop_root()  # Network service runs as ubuntu after device mapping, never as root.
    if args.action=='grab':
        try:
            if args.output is None: parser.error('grab needs --output')
            pixels,info=grabber.capture(); image=rgb565_to_bgr(pixels)
            args.output.parent.mkdir(parents=True,exist_ok=True)
            if not cv2.imwrite(str(args.output),image): raise RuntimeError('Cannot save image')
            args.output.with_suffix(args.output.suffix+'.json').write_text(json.dumps(info,indent=2)+'\n')
            print(json.dumps(info),flush=True)
        finally:
            grabber.close()
        return
    preview=Preview(grabber,detector,args.fps)
    server=http.server.ThreadingHTTPServer((args.host,args.port),handler_for(preview))
    server.daemon_threads=True
    producer=threading.Thread(target=preview.produce,daemon=True); producer.start()
    def stop_signal(*_):
        preview.stop.set()
        with preview.cv: preview.cv.notify_all()
        threading.Thread(target=server.shutdown,daemon=True).start()
    signal.signal(signal.SIGTERM,stop_signal); signal.signal(signal.SIGINT,stop_signal)
    print(json.dumps(dict(event='listening',host=args.host,port=args.port,yolo=False,uid=os.getuid())),flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    finally:
        preview.stop.set(); server.server_close(); producer.join(timeout=10)
        pump = preview.capture_pump
        if not producer.is_alive() and (pump is None or not pump.thread.is_alive()):
            grabber.close()


if __name__=='__main__':
    main()
