"""V2: raw YUYV capture, cacheable ROI DMA, six PL morphology stages."""
import argparse
import fcntl
from collections import Counter,deque
import json
import math
import os
from pathlib import Path
import threading
import struct
import time
import cv2
import numpy as np
from classify_fast import classify_clean
from gesture import DEFAULT_BOUNDS

KERNEL=np.ones((3,3),np.uint8)
VERSION=0x47535432

def reference(raw,bounds=DEFAULT_BOUNDS,y_min=40,decimate2=False):
    h,w,_=raw.shape
    pairs=raw.reshape(h,w//2,4)
    chroma=(pairs[:,:,1]>=bounds[0])&(pairs[:,:,1]<=bounds[1])&(pairs[:,:,3]>=bounds[2])&(pairs[:,:,3]<=bounds[3])
    if decimate2:mask=((pairs[::2,:,0]>=y_min)&chroma[::2]).astype(np.uint8)*255
    else:mask=((raw[:,:,0]>=y_min)&np.repeat(chroma,2,axis=1)).astype(np.uint8)*255
    opened=cv2.morphologyEx(mask,cv2.MORPH_OPEN,KERNEL)
    return cv2.morphologyEx(opened,cv2.MORPH_CLOSE,KERNEL,iterations=2)

class Processor:
    def __init__(self,bitfile,shape,bounds,y_min,decimate2=False,download=True):
        from pynq import Overlay,allocate
        path=Path(bitfile)
        if not path.is_file() or not path.with_suffix('.hwh').is_file():raise RuntimeError('V2 .bit/.hwh pair required')
        self.overlay=Overlay(str(path),download=download)
        if not self.overlay.is_loaded():raise RuntimeError('Requested overlay is not loaded; start the desktop hardware service first')
        if 'processing_config' not in self.overlay.ip_dict:raise RuntimeError('V1 overlay supplied: V2 processing_config is missing')
        config=self.overlay.processing_config.mmio
        if config.read(8)!=VERSION:raise RuntimeError('Incompatible PL version')
        h,w=shape
        if not 2<=w<=384 or w%2 or not 1<=h<=384:raise ValueError('ROI must have even width 2..384 and height 1..384')
        self.decimate2=decimate2;self.input_shape=(h*2,w*2) if decimate2 else shape
        config.write(0,(h<<16)|w|(0x80000000 if decimate2 else 0))
        self.overlay.skin_bounds.mmio.write(0,sum(int(v)<<(8*i) for i,v in enumerate(bounds)))
        self.overlay.skin_bounds.mmio.write(8,y_min)
        self.dma=self.overlay.axi_dma_0
        self.tx=[allocate(shape=(*self.input_shape,2),dtype=np.uint8,cacheable=True) for _ in range(2)]
        self.rx=[allocate(shape=(h,w),dtype=np.uint8,cacheable=True) for _ in range(2)]
        self.slot=0
    def process(self,raw):
        slot=self.slot;self.slot^=1
        start=time.perf_counter();np.copyto(self.tx[slot],raw);filled=time.perf_counter()
        try:
            self.dma.recvchannel.transfer(self.rx[slot]);self.dma.sendchannel.transfer(self.tx[slot])
            submitted=time.perf_counter();deadline=submitted+2
            while not (self.dma.recvchannel.idle and self.dma.sendchannel.idle):
                if self.dma.mmio.read(4)&0x70 or self.dma.mmio.read(0x34)&0x70:raise RuntimeError('DMA error')
                if time.perf_counter()>deadline:raise TimeoutError('V2 DMA timeout; reload overlay')
                time.sleep(0.00005)
            self.dma.sendchannel.wait();self.dma.recvchannel.wait()
            finished=time.perf_counter()
            return self.rx[slot].view(np.ndarray),{'copy_ms':(filled-start)*1000,'submit_cache_ms':(submitted-filled)*1000,'dma_receive_ms':(finished-submitted)*1000}
        except Exception:
            self.dma.sendchannel.stop();self.dma.recvchannel.stop();raise
    def close(self):
        for buffer in self.tx+self.rx:buffer.freebuffer()

class Capture:
    def __init__(self,device,width,height,fps,exposure_us=20000,native_scale=1):
        if native_scale not in (1,2):raise ValueError('Native capture scale must be 1 or 2')
        self.device=device;self.original_controls={}
        self.cap=cv2.VideoCapture(device,cv2.CAP_V4L2)
        if not self.cap.isOpened():raise RuntimeError('Camera unavailable; check video group permissions')
        self.cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'YUYV'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,width*native_scale);self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,height*native_scale)
        self.cap.set(cv2.CAP_PROP_FPS,fps);self.cap.set(cv2.CAP_PROP_CONVERT_RGB,0)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE,2)
        actual=(int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        allowed=[(width,height),(width*2,height*2)] if native_scale==1 else [(width*2,height*2)]
        if actual not in allowed or int(self.cap.get(cv2.CAP_PROP_FOURCC))!=cv2.VideoWriter_fourcc(*'YUYV'):
            self.cap.release();raise RuntimeError(f'Camera did not negotiate YUYV {width*native_scale}x{height*native_scale}: {actual}')
        self.scale=actual[0]//width
        with open(device,'rb',buffering=0) as control:
            for cid in [0x009a0901,0x009a0902]:
                value=bytearray(struct.pack('<Ii',cid,0));fcntl.ioctl(control,0xc008561b,value,True)
                self.original_controls[cid]=struct.unpack_from('<i',value,4)[0]
            fcntl.ioctl(control,0xc008561c,struct.pack('<Ii',0x009a0901,1))
            fcntl.ioctl(control,0xc008561c,struct.pack('<Ii',0x009a0902,exposure_us//100))
        self.settings={'format':'YUYV','native_width':actual[0],'native_height':actual[1],'processing_width':width,'processing_height':height,'decimation':self.scale,'requested_fps':fps,'reported_fps':self.cap.get(cv2.CAP_PROP_FPS),'buffers':self.cap.get(cv2.CAP_PROP_BUFFERSIZE)}
        self.settings['exposure_us']=exposure_us
        self.width=actual[0];self.height=actual[1];self.condition=threading.Condition();self.latest=None
        self.count=0;self.dropped=0;self.error=None;self.stop=threading.Event()
        self.thread=threading.Thread(target=self._run,daemon=True)
    def start(self):self.thread.start()
    def _run(self):
        try:
            while not self.stop.is_set():
                before=time.perf_counter();ok,raw=self.cap.read();after=time.perf_counter()
                if not ok:raise RuntimeError('Camera read failed')
                if raw.dtype!=np.uint8 or raw.size!=self.height*self.width*2:raise RuntimeError(f'Unexpected raw frame: {raw.shape} {raw.dtype}')
                raw=raw.reshape(self.height,self.width,2)
                with self.condition:
                    self.count+=1
                    if self.latest is not None:self.dropped+=1
                    self.latest=(self.count,raw,after,(after-before)*1000);self.condition.notify()
        except Exception as exc:
            with self.condition:self.error=exc;self.condition.notify_all()
        finally:self.cap.release()
    def get(self):
        with self.condition:
            if not self.condition.wait_for(lambda:self.latest is not None or self.error is not None,timeout=3):raise TimeoutError('Camera stalled')
            if self.error:raise self.error
            frame=self.latest;self.latest=None;return frame
    def close(self):
        self.stop.set()
        if self.thread.ident is None:self.cap.release()
        else:
            self.thread.join(timeout=3)
            if self.thread.is_alive():raise RuntimeError('Capture thread did not stop')
        try:
            with open(self.device,'rb',buffering=0) as control:
                # Restore absolute exposure while manual, then the prior mode.
                for cid in [0x009a0902,0x009a0901]:fcntl.ioctl(control,0xc008561c,struct.pack('<Ii',cid,self.original_controls[cid]))
        except OSError as exc:print('Camera control restore failed: '+str(exc),flush=True)

def fixtures():
    masks=[]
    circle=np.zeros((180,180),np.uint8);cv2.circle(circle,(90,90),35,255,-1);masks.append((circle,'FIST'))
    hand=np.zeros((180,180),np.uint8);cv2.rectangle(hand,(35,85),(135,145),255,-1)
    for x,top in [(35,45),(57,20),(79,10),(101,20),(123,45)]:cv2.rectangle(hand,(x,top),(x+12,100),255,-1)
    masks.append((hand,'OPEN'));v=np.zeros_like(hand)
    cv2.fillPoly(v,[np.array([[65,145],[115,145],[110,80],[145,30],[128,20],[92,68],[60,20],[42,30],[68,80]],np.int32)],255);masks.append((v,'V'))
    return masks

def self_test(processor,shape):
    import gesture
    for mask,label in fixtures():
        for rotation in range(4):
            mask_r=np.ascontiguousarray(np.rot90(mask,rotation))
            clean=cv2.morphologyEx(cv2.morphologyEx(mask_r,cv2.MORPH_OPEN,KERNEL),cv2.MORPH_CLOSE,KERNEL,iterations=2)
            fast,_=classify_clean(clean);old=gesture.classify(mask_r)[0]
            assert fast==old,(label,rotation,fast,old)
    assert classify_clean(np.zeros(shape,np.uint8))[0]['gesture']=='NONE'
    rng=np.random.default_rng(7020)
    for i in range(100):
        raw=rng.integers(0,256,(*(processor.input_shape if processor else shape),2),dtype=np.uint8)
        if i%4==0:raw[:,:,0]=100;raw[:,::2,1]=100;raw[:,1::2,1]=150
        elif i%4==1:raw[:]=0
        elif i%4==2:
            raw[:,:,0]=100;raw[:,::2,1]=100;raw[:,1::2,1]=150
            raw[20:30,20:30,0]=0;raw[:3,:,0]=0
        expected=reference(raw,decimate2=processor.decimate2 if processor else False)
        if processor:
            actual,_=processor.process(raw)
            if not np.array_equal(actual,expected):raise AssertionError(f'Frame {i}: {np.count_nonzero(actual!=expected)} pixel mismatches')
    print(json.dumps({'self_test':'PASS','frames':100,'rotation_cases':12,'pl_checked':processor is not None}),flush=True)

def snapshot(path,raw,roi,contour,label,fps):
    frame=cv2.cvtColor(raw,cv2.COLOR_YUV2BGR_YUY2)
    if frame.shape[:2]!=(240,320):frame=cv2.resize(frame,(320,240))
    x,y,w,h=roi;cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),1)
    if contour is not None:cv2.drawContours(frame,[contour],-1,(0,255,255),1,offset=(x,y))
    cv2.putText(frame,f'{label} | V2 | {fps:.1f} FPS',(5,18),cv2.FONT_HERSHEY_SIMPLEX,0.45,(0,255,0),1)
    if not cv2.imwrite(str(path),frame):raise RuntimeError('Snapshot write failed')

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--backend',choices=['cpu','pl'],default='pl');ap.add_argument('--camera',default='/dev/video0')
    ap.add_argument('--fps',type=int,default=30);ap.add_argument('--duration',type=float,default=120)
    ap.add_argument('--exposure-us',type=int,default=20000)
    ap.add_argument('--warmup',type=float,default=5);ap.add_argument('--frames',type=int,default=0)
    ap.add_argument('--bitfile',default=str(Path(__file__).resolve().parents[1]/'output/gesture_v2.bit'))
    ap.add_argument('--report',default='gesture-v2-report.json');ap.add_argument('--snapshot')
    ap.add_argument('--self-test',action='store_true');ap.add_argument('--bounds',type=int,nargs=4,default=DEFAULT_BOUNDS)
    ap.add_argument('--y-min',type=int,default=40)
    args=ap.parse_args()
    if args.duration<=0 or args.warmup<0 or args.fps<=0 or args.frames<0:ap.error('Invalid run duration/rate')
    if not 100<=args.exposure_us<=min(500000,1000000//args.fps) or args.exposure_us%100:ap.error('Exposure must be a multiple of 100 us within one frame period')
    if not all(0<=v<=255 for v in [*args.bounds,args.y_min]) or args.bounds[0]>args.bounds[1] or args.bounds[2]>args.bounds[3]:ap.error('Invalid thresholds')
    roi=(64,24,192,192);shape=(192,192)
    processor=None;capture=None
    try:
        if args.self_test:
            if args.backend=='pl':processor=Processor(args.bitfile,shape,args.bounds,args.y_min,decimate2=True)
            self_test(processor,shape);return
        capture=Capture(args.camera,320,240,args.fps,args.exposure_us)
        if args.backend=='pl':processor=Processor(args.bitfile,shape,args.bounds,args.y_min,decimate2=capture.scale==2)
        capture.start()
        print(json.dumps({'camera':capture.settings,'backend':args.backend,'roi':roi}),flush=True)
        warm_end=time.perf_counter()+args.warmup
        while time.perf_counter()<warm_end:capture.get()
        history=deque(maxlen=5);times=[];classes=Counter();windows=Counter();completed=0
        capture_base=capture.count;drop_base=capture.dropped;start=time.perf_counter();next_log=start+1;last=None
        while time.perf_counter()-start<args.duration and (not args.frames or completed<args.frames):
            before=time.perf_counter();frame_id,raw,acquired,capture_ms=capture.get();obtained=time.perf_counter()
            x,y,w,h=roi;scale=capture.scale;input_roi=raw[y*scale:(y+h)*scale,x*scale:(x+w)*scale]
            if processor:clean,stages=processor.process(input_roi)
            else:
                ref_start=time.perf_counter();clean=reference(input_roi,args.bounds,args.y_min,decimate2=scale==2)
                stages={'cpu_preprocess_ms':(time.perf_counter()-ref_start)*1000}
            classify_start=time.perf_counter();info,contour=classify_clean(clean);done=time.perf_counter()
            history.append(info['gesture']);label,votes=Counter(history).most_common(1)[0];label=label if votes>=3 else 'UNKNOWN'
            completed+=1;classes[label]+=1;windows[int(done-start)]+=1
            times.append(dict(capture_read_ms=capture_ms,queue_wait_ms=(obtained-before)*1000,classify_ms=(done-classify_start)*1000,frame_age_ms=(done-acquired)*1000,**stages))
            last=(raw,contour,label)
            if done>=next_log:
                print(json.dumps({**info,'frame':completed,'source_frame':frame_id,'raw_gesture':info['gesture'],'gesture':label,'fps':round(completed/(done-start),2),'capture_drops':capture.dropped-drop_base}),flush=True)
                next_log=done+1
        elapsed=time.perf_counter()-start
        captured=capture.count-capture_base;dropped=capture.dropped-drop_base
        full_seconds=int(elapsed);counts=[windows[i] for i in range(full_seconds)]
        fraction=sum(n>=29 for n in counts)/len(counts) if counts else 0
        fps=completed/elapsed
        stages={key:{'mean':float(np.mean([r[key] for r in times])),'p95':float(np.percentile([r[key] for r in times],95))} for key in times[0]} if times else {}
        result=dict(backend=args.backend,camera=capture.settings,roi=roi,processed=completed,captured=captured,dropped=dropped,elapsed_seconds=elapsed,fps=fps,one_second_counts=counts,windows_ge_29_fraction=fraction,labels=dict(classes),stages_ms=stages,dma_errors=0,
                    acceptance_pass=elapsed>=120 and fps>=29.5 and fraction>=0.95)
        Path(args.report).write_text(json.dumps(result,indent=2),encoding='utf-8')
        if args.snapshot and last:snapshot(args.snapshot,*[last[0],roi,last[1],last[2],fps])
        print('RESULT '+json.dumps(result),flush=True)
    finally:
        if capture:capture.close()
        if processor:processor.close()

if __name__=='__main__':main()
