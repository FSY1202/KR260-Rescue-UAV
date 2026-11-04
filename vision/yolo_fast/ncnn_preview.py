"""HTTP-only YOLO26n NCNN preview; no ROS, PL, serial or actuator access."""
import os
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
os.environ.setdefault('OMP_NUM_THREADS','1')
import argparse
from pathlib import Path
import http.server
import json
import signal
import threading
import time
import uuid
import urllib.request
import cv2
import numpy as np
from ncnn_detector import Detector, AngleSearch, annotate

ROOT=Path(os.environ.get('KR260_YOLO_ROOT',str(Path(__file__).resolve().parent.parent/'yolo_standard')))


def inference_budget(path=Path('/run/kr260-rescue/vision_budget.json'),clock=time.monotonic):
    """Allow only pinned profiles, from an atomic same-host monotonic lease."""
    try:
        if path.stat().st_size>2048:return (2,640)
        value=json.loads(path.read_text())
        when=value.get('updated_monotonic')
        rid=value.get('run_id')
        size=value.get('input_size',640)  # Old single-pass lease remains compatible.
        if (type(when) in (int,float) and isinstance(rid,str) and len(rid)==32
                and uuid.UUID(hex=rid).hex==rid and type(value.get('max_angle_passes')) is int
                and value['max_angle_passes']==1 and type(size) is int and size in (320,384,480,640)
                and 0<=clock()-when<=3):return (1,size)
    except (OSError,ValueError,TypeError,AttributeError):pass
    return (2,640)


def angle_pass_budget(path=Path('/run/kr260-rescue/vision_budget.json'),clock=time.monotonic):
    return inference_budget(path,clock)[0]


def select_preview_profile(fast_preview=False):
    """Keep rescue leases authoritative; accelerate only the explicit preview mode."""
    budget,size=inference_budget()
    if os.environ.get('KR260_VISION_PROFILE')=='mission':return (1,320,.5)
    if fast_preview and budget==2:return (1,384,.2)
    return (budget,size,.5)


class DetectorPool:
    """Lazy, fixed-path exports, loaded BEFORE fetching the next image."""
    def __init__(self,threads,factory=Detector):
        self.threads,self.factory,self.loaded=threads,factory,{}
    def get(self,size):
        if type(size) is not int or size not in (320,384,480,640):raise ValueError('Unknown input profile')
        if size not in self.loaded:
            model_dir=ROOT/(f'models/yolo26n_ncnn_{size}_model' if size!=640 else 'models/yolo26n_ncnn_model')
            receipt=ROOT/(f'export_receipt_{size}.json' if size!=640 else 'export_receipt.json')
            model=self.factory(model_dir,receipt,self.threads)
            if model.size!=size:raise RuntimeError('Export receipt/profile mismatch')
            self.loaded[size]=AngleSearch(model)
        return self.loaded[size]


PAGE=b"""<!doctype html><meta charset=utf-8><title>KR260 YOLO26</title>
<h1>YOLO26n / NCNN</h1><img src='/stream.mjpg' width='640'>
<p><a href='/status.json'>Detection status</a> | <a href='/snapshot.jpg'>Snapshot</a></p>"""


def source_metadata(headers):
    """Never fabricate frame identities for a legacy source; preview still works."""
    try:
        session = headers['X-Camera-Session']
        if not isinstance(session,str) or len(session)!=32 or uuid.UUID(hex=session).hex!=session:
            raise ValueError('Invalid camera session')
        seq = int(headers['X-Camera-Frame-Seq'])
        stamp = int(headers['X-Camera-Monotonic-Ns'])
        if seq<0 or stamp<=0 or stamp>time.monotonic_ns():
            raise ValueError('Invalid camera frame sequence/time')
        return dict(source_camera_session_id=session,source_frame_seq=seq,
                    source_frame_monotonic_ns=stamp,source_metadata_error=None)
    except (KeyError,TypeError,ValueError,AttributeError) as exc:
        return dict(source_camera_session_id=None,source_frame_seq=None,
                    source_frame_monotonic_ns=None,source_metadata_error=str(exc))


class Preview:
    def __init__(self,threads=2,confidence=.25,fast_preview=False):
        self.cv=threading.Condition()
        self.stop=threading.Event()
        self.jpeg=None
        self.session_id=uuid.uuid4().hex
        self.version=0
        self.received=0.
        self.threads=threads
        self.fast_preview=fast_preview
        self.info=dict(state='STARTING',model='YOLO26n',backend='NCNN CPU FP32',imgsz=640,
                       confidence=confidence,count=None,detections=[],error=None,
                       detection_session_id=self.session_id,result_seq=0,
                       source_camera_session_id=None,source_frame_seq=None,
                       source_frame_monotonic_ns=None,source_metadata_error='No source frame yet',
                       source='http://127.0.0.1:8080/raw.jpg',coordinate_frame='original_camera_image')

    def status(self):
        with self.cv:
            out=dict(self.info)
            age=(time.monotonic()-self.received)*1000 if self.received else None
            out['result_age_ms']=age
            stamp=out.get('source_frame_monotonic_ns')
            out['source_frame_age_ms']=(time.monotonic_ns()-stamp)/1e6 if stamp else None
            if out['state']=='READY' and age is not None and age>5000:
                out.update(state='STALE',count=None,detections=[],error='No fresh result')
            return out

    def produce(self):
        pool=DetectorPool(self.threads)
        previous=None
        while not self.stop.is_set():
            started=time.monotonic()
            interval=.5
            try:
                budget,size,interval=select_preview_profile(self.fast_preview)
                detector=pool.get(size)  # Cold loading must not age an already captured frame.
                with urllib.request.urlopen(self.info['source'],timeout=3) as response:
                    data=response.read(4*1024*1024+1)
                    metadata=source_metadata(response.headers)
                received=time.monotonic()
                if len(data)>4*1024*1024:
                    raise RuntimeError('Camera JPEG too large')
                image=cv2.imdecode(np.frombuffer(data,dtype=np.uint8),cv2.IMREAD_COLOR)
                if image is None:
                    raise RuntimeError('Camera JPEG decode failed')
                if budget==1:items,ms,direction=detector.detect(image,self.info['confidence'],max_angle_passes=1)
                else:items,ms,direction=detector.detect(image,self.info['confidence'])
                ok,jpeg=cv2.imencode('.jpg',annotate(image,items),[cv2.IMWRITE_JPEG_QUALITY,85])
                if not ok:
                    raise RuntimeError('JPEG encoding failed')
                now=time.monotonic()
                if now-received>5:
                    raise RuntimeError('Result over five seconds old; discarded')
                with self.cv:
                    self.jpeg=jpeg.tobytes()
                    self.received=received
                    self.version+=1
                    self.info.update(state='READY',error=None,count=len(items),detections=items,
                                     result_seq=self.version,max_angle_passes=budget,imgsz=size,**metadata,
                                     processing_ms=(now-received)*1000,inference_ms=ms,
                                     fps=1/(now-previous) if previous else 0,
                                     frame_size=[image.shape[1],image.shape[0]],**direction)
                    self.cv.notify_all()
                previous=now
                print(json.dumps(dict(event='detection',count=len(items),processing_ms=round(ms,1),imgsz=size,**direction)),flush=True)
            except Exception as exc:
                with self.cv:
                    self.jpeg=None
                    self.info.update(state='ERROR',error=str(exc),count=None,detections=[])
                    self.cv.notify_all()
                print(json.dumps(dict(event='capture_or_inference_error',error=str(exc))),flush=True)
                self.stop.wait(1)
            self.stop.wait(max(0,interval-(time.monotonic()-started)))


def handler_for(preview):
    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self,*_): pass

        def send(self,code,kind,body):
            self.send_response(code)
            self.send_header('Content-Type',kind)
            self.send_header('Cache-Control','no-store')
            self.send_header('Content-Length',str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            try:
                if self.path=='/': self.send(200,'text/html; charset=utf-8',PAGE)
                elif self.path=='/status.json': self.send(200,'application/json',json.dumps(preview.status()).encode())
                elif self.path=='/snapshot.jpg':
                    with preview.cv:
                        jpeg=preview.jpeg if preview.status()['state']=='READY' else None
                    self.send(200 if jpeg else 503,'image/jpeg' if jpeg else 'text/plain',jpeg or b'No fresh detection image')
                elif self.path.startswith('/stream.mjpg'):
                    self.send_response(200)
                    self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame')
                    self.send_header('Cache-Control','no-store')
                    self.end_headers()
                    version=-1
                    while not preview.stop.is_set():
                        with preview.cv:
                            preview.cv.wait_for(lambda: (preview.version!=version and preview.jpeg is not None)
                                                or preview.stop.is_set(),timeout=2)
                            if preview.stop.is_set(): break
                            state=preview.status()['state']
                            if state=='STARTING': continue
                            if state!='READY': break
                            if version==preview.version: continue
                            version,jpeg=preview.version,preview.jpeg
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(jpeg)).encode()+b'\r\n\r\n'+jpeg+b'\r\n')
                        self.wfile.flush()
                else: self.send(404,'text/plain',b'Not found')
            except (BrokenPipeError,ConnectionResetError,TimeoutError): pass
    return Handler


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',default='0.0.0.0')
    parser.add_argument('--port',type=int,default=8082)
    parser.add_argument('--threads',type=int,choices=(1,2),default=2)
    parser.add_argument('--confidence',type=float,default=.25)
    parser.add_argument('--fast-preview',action='store_true',
                        help='Use 384 single-pass preview at up to 5 Hz when no rescue lease is active')
    args=parser.parse_args()
    if not 0<args.confidence<1: parser.error('Confidence must be 0..1')
    if hasattr(os,'sched_getaffinity'):
        os.sched_setaffinity(0,set(sorted(os.sched_getaffinity(0))[-args.threads:]))
    cv2.setNumThreads(1)
    preview=Preview(args.threads,args.confidence,fast_preview=args.fast_preview)
    server=http.server.ThreadingHTTPServer((args.host,args.port),handler_for(preview))
    server.daemon_threads=True
    worker=threading.Thread(target=preview.produce,daemon=True);worker.start()
    def stop(*_):
        preview.stop.set()
        with preview.cv: preview.cv.notify_all()
        threading.Thread(target=server.shutdown,daemon=True).start()
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    try: server.serve_forever(poll_interval=.2)
    finally:
        preview.stop.set()
        with preview.cv: preview.cv.notify_all()
        server.server_close();worker.join(timeout=10)


if __name__=='__main__': main()
