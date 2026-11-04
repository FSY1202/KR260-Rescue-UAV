"""PYNQ-Z2 gesture demo: CPU reference or AXI DMA skin segmentation.

Controlled background, one hand in the ROI. This is a contour heuristic,
not a trained model; UNKNOWN is intentional when evidence is insufficient.
"""
import argparse
from collections import Counter, deque
import json
from pathlib import Path
import time

import cv2
import numpy as np

DEFAULT_BOUNDS = (77, 127, 133, 173)

def segment_cpu(bgr, bounds=DEFAULT_BOUNDS, y_min=40):
    b, g, r = (bgr[:, :, i].astype(np.int32) for i in range(3))
    y = ((77*r + 150*g + 29*b) >> 8).astype(np.uint8)
    cb = (32768 - 43*r - 85*g + 128*b) >> 8
    cr = (32768 + 128*r - 107*g - 21*b) >> 8
    lo_cb, hi_cb, lo_cr, hi_cr = bounds
    mask = ((y >= y_min) & (cb >= lo_cb) & (cb <= hi_cb) &
            (cr >= lo_cr) & (cr <= hi_cr)).astype(np.uint8)*255
    return mask, y

class PLProcessor:
    def __init__(self, bitfile, shape, bounds, y_min, timeout=2.0):
        from pynq import Overlay, allocate
        bitfile = Path(bitfile)
        if not bitfile.is_file() or not bitfile.with_suffix('.hwh').is_file():
            raise RuntimeError('Matching .bit and .hwh files are required')
        self.overlay = Overlay(str(bitfile))
        self.dma = self.overlay.axi_dma_0
        self.overlay.skin_bounds.mmio.write(0, sum(v << (8*i) for i, v in enumerate(bounds)))
        self.overlay.skin_bounds.mmio.write(8, y_min)
        self.tx = allocate(shape=shape, dtype=np.uint32)
        self.rx = allocate(shape=shape, dtype=np.uint32)
        self.timeout = timeout

    def process(self, frame):
        if frame.shape[:2] != self.tx.shape:
            raise ValueError('Frame size must match allocated DMA buffers')
        pixels = self.tx.view(np.uint8).reshape(*self.tx.shape, 4)
        pixels[:, :, :3] = frame
        pixels[:, :, 3] = 0
        try:
            self.dma.recvchannel.transfer(self.rx)
            self.dma.sendchannel.transfer(self.tx)
            deadline = time.monotonic()+self.timeout
            while not (self.dma.recvchannel.idle and self.dma.sendchannel.idle):
                if self.dma.mmio.read(4) & 0x70 or self.dma.mmio.read(0x34) & 0x70:
                    raise RuntimeError('AXI DMA error; inspect length/alignment/TLAST')
                if time.monotonic() > deadline:
                    raise TimeoutError('DMA timeout; reload overlay before retrying')
                time.sleep(0.0001)
            self.dma.sendchannel.wait()
            self.dma.recvchannel.wait()
            return (self.rx & 255).astype(np.uint8), ((self.rx >> 8) & 255).astype(np.uint8)
        except Exception:
            self.dma.sendchannel.stop()
            self.dma.recvchannel.stop()
            raise

    def close(self):
        self.tx.freebuffer()
        self.rx.freebuffer()

def classify(mask):
    kernel = np.ones((3, 3), np.uint8)
    clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    clean = cv2.morphologyEx(clean, cv2.MORPH_CLOSE, kernel, iterations=2)
    contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    info = {'gesture': 'NONE', 'gaps': 0, 'solidity': 0.0}
    if not contours:
        return info, clean, None
    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    if area < max(250, mask.size*0.025):
        return info, clean, None
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    if hull_area <= 0:
        return info, clean, None
    x, y, w, h = cv2.boundingRect(contour)
    distance = cv2.distanceTransform(clean, cv2.DIST_L2, 5)
    # Measure palm radius inside this contour rather than another component.
    component = np.zeros_like(clean)
    cv2.drawContours(component, [contour], -1, 255, -1)
    _, radius, _, palm = cv2.minMaxLoc(distance, mask=component)
    # Average a peak plateau so rotating a silhouette does not select a
    # different end of an equally thick palm/wrist region.
    peak_y,peak_x=np.nonzero((distance>=radius*0.98)&(component>0))
    if len(peak_x): palm=(float(peak_x.mean()),float(peak_y.mean()))
    gaps = 0
    indices = cv2.convexHull(contour, returnPoints=False)
    if indices is not None and len(indices) >= 4 and len(contour) >= 4:
        try:
            defects = cv2.convexityDefects(contour, indices)
        except cv2.error:
            defects = None
        if defects is not None:
            for s, e, f, depth in defects.reshape(-1,4):
                start, end, valley = contour[s, 0], contour[e, 0], contour[f, 0]
                a, b = start-valley, end-valley
                norm = float(np.linalg.norm(a)*np.linalg.norm(b))
                if norm == 0:
                    continue
                angle = np.degrees(np.arccos(np.clip(np.dot(a, b)/norm, -1, 1)))
                start_radius=float(np.linalg.norm(start-np.array(palm)))
                end_radius=float(np.linalg.norm(end-np.array(palm)))
                valley_radius=float(np.linalg.norm(valley-np.array(palm)))
                if (angle < 100 and depth/256 > max(8, radius*0.35) and
                        min(start_radius,end_radius)>radius*1.4 and
                        valley_radius<radius*2.3 and
                        valley_radius<min(start_radius,end_radius)*0.85):
                    gaps += 1
    solidity = area/hull_area
    extent = area/(w*h)
    if gaps >= 3:
        label = 'OPEN'
    elif gaps == 1 and solidity < 0.90:
        label = 'V'
    elif gaps == 0 and solidity >= 0.82 and extent >= 0.50 and max(w/h, h/w) < 1.65:
        label = 'FIST'
    else:
        label = 'UNKNOWN'
    info.update(gesture=label, gaps=gaps, solidity=round(solidity, 3), bbox=[x,y,w,h])
    return info, clean, contour

def self_test(processor=None):
    frame = np.array([[[80,120,180], [255,0,0], [0,0,0], [100,100,100]]], np.uint8)
    mask, _ = segment_cpu(frame)
    assert mask.tolist() == [[255,0,0,0]]
    assert classify(np.zeros((100,100), np.uint8))[0]['gesture'] == 'NONE'
    circle = np.zeros((100,100), np.uint8)
    cv2.circle(circle, (50,50), 25, 255, -1)
    assert classify(circle)[0]['gesture'] == 'FIST'
    hand = np.zeros((180,180), np.uint8)
    cv2.rectangle(hand,(35,85),(135,145),255,-1)
    for x, top in [(35,45),(57,20),(79,10),(101,20),(123,45)]:
        cv2.rectangle(hand,(x,top),(x+12,100),255,-1)
    assert classify(hand)[0]['gesture'] == 'OPEN'
    v_hand = np.zeros((180,180), np.uint8)
    cv2.fillPoly(v_hand,[np.array([[65,145],[115,145],[110,80],[145,30],[128,20],[92,68],[60,20],[42,30],[68,80]],np.int32)],255)
    assert classify(v_hand)[0]['gesture'] == 'V'
    for fixture,label in [(circle,'FIST'),(hand,'OPEN'),(v_hand,'V')]:
        for rotation in range(4):
            assert classify(np.ascontiguousarray(np.rot90(fixture,rotation)))[0]['gesture']==label, (label,rotation)
    if processor:
        rng = np.random.default_rng(7020)
        for _ in range(4):
            pixels = rng.integers(0,256,(*processor.tx.shape,3),dtype=np.uint8)
            expected = segment_cpu(pixels)
            actual = processor.process(pixels)
            assert all(np.array_equal(a,b) for a,b in zip(expected,actual)), 'CPU/PL mismatch'
    print('PASS: CPU numerical reference and contour smoke tests' + ('; four CPU/PL frame comparisons' if processor else ''))

def benchmark(shape, bounds, y_min, processor):
    pixels=np.random.default_rng(7020).integers(0,256,(*shape,3),dtype=np.uint8)
    result={'kind':'synthetic_preprocessing_only','width':shape[1],'height':shape[0]}
    for name,fn in [('cpu',lambda: segment_cpu(pixels,bounds,y_min))] + ([('pl',lambda: processor.process(pixels))] if processor else []):
        for _ in range(3): fn()
        start=time.perf_counter()
        for _ in range(20): fn()
        result[name+'_segment_ms']=(time.perf_counter()-start)*50
    if processor: result['preprocessing_speedup']=result['cpu_segment_ms']/result['pl_segment_ms']
    print(json.dumps(result),flush=True)

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--camera', default='/dev/video0')
    ap.add_argument('--width', type=int, default=320)
    ap.add_argument('--height', type=int, default=240)
    ap.add_argument('--backend', choices=['cpu','pl'], default='cpu')
    ap.add_argument('--bitfile', default='../output/gesture.bit')
    ap.add_argument('--bounds', type=int, nargs=4, default=DEFAULT_BOUNDS,
                    metavar=('CB_MIN','CB_MAX','CR_MIN','CR_MAX'))
    ap.add_argument('--y-min', type=int, default=40)
    ap.add_argument('--headless', action='store_true')
    ap.add_argument('--frames', type=int, default=0, help='0 runs until interrupted')
    ap.add_argument('--snapshot', help='Save the latest annotated frame to this JPEG path')
    ap.add_argument('--self-test', action='store_true')
    ap.add_argument('--benchmark', action='store_true',help='Synthetic preprocessing benchmark; no camera FPS claim')
    args = ap.parse_args()
    if not all(0 <= v <= 255 for v in [*args.bounds,args.y_min]):
        ap.error('Thresholds must be in 0..255')
    if args.bounds[0]>args.bounds[1] or args.bounds[2]>args.bounds[3]:
        ap.error('Minimum thresholds must not exceed maximum thresholds')
    if args.width<=0 or args.height<=0 or args.width*args.height*4 >= 2**26:
        ap.error('Invalid image size for 26-bit AXI DMA transfer length')
    processor = PLProcessor(args.bitfile,(args.height,args.width),args.bounds,args.y_min) if args.backend=='pl' else None
    if args.self_test:
        try: self_test(processor)
        finally:
            if processor: processor.close()
        return
    if args.benchmark:
        try: benchmark((args.height,args.width),args.bounds,args.y_min,processor)
        finally:
            if processor: processor.close()
        return
    source = int(args.camera) if args.camera.isdigit() else args.camera
    cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
    if not cap.isOpened():
        if processor: processor.close()
        raise RuntimeError(f'Cannot open camera {args.camera}; check /dev/video* and USB cable')
    cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,args.height)
    cap.set(cv2.CAP_PROP_FPS,15)
    history=deque(maxlen=5); timings=deque(maxlen=30); count=0
    try:
        while not args.frames or count<args.frames:
            started=time.perf_counter()
            ok, frame=cap.read()
            if not ok: raise RuntimeError('Camera frame read failed')
            frame=cv2.resize(frame,(args.width,args.height))
            frame=cv2.flip(frame,1)
            h,w=frame.shape[:2]
            left,top,right,bottom=w//5,h//10,4*w//5,9*h//10
            segment_start=time.perf_counter()
            mask,_=processor.process(frame) if processor else segment_cpu(frame,args.bounds,args.y_min)
            segment_ms=(time.perf_counter()-segment_start)*1000
            roi=mask[top:bottom,left:right]
            info,clean,contour=classify(roi)
            history.append(info['gesture'])
            candidate,votes=Counter(history).most_common(1)[0]
            stable=candidate if votes>=3 else 'UNKNOWN'
            cv2.rectangle(frame,(left,top),(right,bottom),(0,255,0),1)
            if contour is not None:
                cv2.drawContours(frame,[contour],-1,(0,255,255),1,offset=(left,top))
            timings.append(time.perf_counter()-started)
            fps=len(timings)/sum(timings)
            cv2.putText(frame,f'{stable} | {args.backend.upper()} | {fps:.1f} FPS',(5,18),cv2.FONT_HERSHEY_SIMPLEX,0.45,(0,255,0),1)
            count+=1
            if count%15==0 or args.frames==1:
                print(json.dumps(dict(frame=count,stable=stable,backend=args.backend,fps=round(fps,2),segment_ms=round(segment_ms,3),**info)),flush=True)
            if args.snapshot and (count%15==0 or args.frames and count==args.frames):
                if not cv2.imwrite(args.snapshot,frame): raise RuntimeError('Snapshot write failed')
            if not args.headless:
                cv2.imshow('PYNQ-Z2 Gesture',frame)
                cv2.imshow('Hand mask',clean)
                if cv2.waitKey(1)&255 in (27,ord('q')): break
    finally:
        cap.release()
        if not args.headless: cv2.destroyAllWindows()
        if processor: processor.close()

if __name__=='__main__':
    main()
