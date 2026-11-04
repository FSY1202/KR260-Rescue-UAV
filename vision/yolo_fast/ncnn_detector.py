"""CPU FP32 YOLO26n NCNN detector, without importing PyTorch."""
from pathlib import Path
import hashlib
import json
import time
import cv2
import numpy as np
import ncnn


def unrotate_box(box, width, height, angle):
    x1, y1, x2, y2 = box
    if angle == 0:
        result = [x1, y1, x2, y2]
    elif angle == 90:
        result = [y1, height-x2, y2, height-x1]
    elif angle == 180:
        result = [width-x2, height-y2, width-x1, height-y1]
    elif angle == 270:
        result = [width-y2, x1, width-y1, x2]
    else:
        raise ValueError('Expected a multiple of 90 degrees')
    result = np.clip(result, [0, 0, 0, 0], [width, height, width, height])
    return result.tolist()


def merge_detections(items, confidence, iou=0.45):
    if not items:
        return []
    boxes = [[d['xyxy'][0], d['xyxy'][1], d['xyxy'][2]-d['xyxy'][0],
              d['xyxy'][3]-d['xyxy'][1]] for d in items]
    keep = cv2.dnn.NMSBoxes(boxes, [d['confidence'] for d in items], confidence, iou)
    return [items[int(i)] for i in np.asarray(keep).reshape(-1)]


class Detector:
    def __init__(self, model_dir, receipt, threads=1):
        model_dir = Path(model_dir)
        manifest = json.loads(Path(receipt).read_text(encoding='utf-8'))
        if manifest.get('source_sha256') != '9b09cc8bf347f0fc8a5f7657480587f25db09b34bf33b0652110fb03a8ad4fef':
            raise RuntimeError('Unexpected trained model weights')
        self.size = manifest.get('imgsz')
        if type(self.size) is not int or self.size not in (320,384,480,640) or manifest.get('quantize') != 32:
            raise RuntimeError('Expected pinned 320/480/640 FP32 export')
        for name in ('model.ncnn.param', 'model.ncnn.bin'):
            if hashlib.sha256((model_dir/name).read_bytes()).hexdigest() != manifest['files'][name]:
                raise RuntimeError('NCNN artifact checksum mismatch: '+name)
        self.net = ncnn.Net()
        self.net.opt.use_vulkan_compute = False
        self.net.opt.num_threads = threads
        self.net.opt.use_fp16_storage = False
        self.net.opt.use_fp16_arithmetic = False
        self.net.opt.use_fp16_packed = False
        if self.net.load_param(str(model_dir/'model.ncnn.param')) != 0:
            raise RuntimeError('NCNN parameter load failed')
        if self.net.load_model(str(model_dir/'model.ncnn.bin')) != 0:
            raise RuntimeError('NCNN weight load failed')
        assert len(self.net.input_names()) == 1 and len(self.net.output_names()) == 1

    def detect_one(self, image, confidence):
        height, width = image.shape[:2]
        scale = min(self.size/width, self.size/height)
        nw, nh = round(width*scale), round(height*scale)
        left = round((self.size-nw)/2-.1)
        top = round((self.size-nh)/2-.1)
        resized = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_LINEAR)
        padded = cv2.copyMakeBorder(resized, top, self.size-nh-top,
                                   left, self.size-nw-left, cv2.BORDER_CONSTANT, value=(114,114,114))
        rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
        mat = ncnn.Mat.from_pixels(rgb, ncnn.Mat.PixelType.PIXEL_RGB, self.size, self.size)
        mat.substract_mean_normalize([], [1/255.]*3)
        with self.net.create_extractor() as ex:
            if ex.input(self.net.input_names()[0], mat) != 0:
                raise RuntimeError('NCNN input failed')
            code, output = ex.extract(self.net.output_names()[0])
            if code != 0:
                raise RuntimeError('NCNN extraction failed')
            rows = np.array(output).copy()
        if rows.ndim == 3 and rows.shape[0] == 1:
            rows = rows[0]
        if rows.ndim == 2 and rows.shape[0] == 84:
            # The official NCNN export disables the end2end branch (TopK unsupported).
            # It emits [4+80,N] xywh/class scores and requires external NMS.
            raw = rows.T
            labels = raw[:,4:].argmax(axis=1)
            scores = raw[:,4:][np.arange(len(raw)), labels]
            selected = raw[(labels == 0) & (scores >= confidence)]
            selected_scores = scores[(labels == 0) & (scores >= confidence)]
            rows = np.zeros((len(selected),6), dtype=np.float32)
            rows[:,:2] = selected[:,:2] - selected[:,2:4]/2
            rows[:,2:4] = selected[:,:2] + selected[:,2:4]/2
            rows[:,4] = selected_scores
        if rows.ndim != 2 or rows.shape[1] != 6 or not np.isfinite(rows).all():
            raise RuntimeError('Unexpected NCNN output shape or nonfinite values: '+str(rows.shape))
        items = []
        for row in rows:
            if row[4] < confidence or int(round(float(row[5]))) != 0:
                continue
            x1, y1, x2, y2 = row[:4].astype(float)
            box = np.clip([(x1-left)/scale, (y1-top)/scale, (x2-left)/scale, (y2-top)/scale],
                          [0,0,0,0], [width,height,width,height]).tolist()
            if box[2] > box[0] and box[3] > box[1]:
                items.append(dict(class_id=0, label='person', confidence=float(row[4]), xyxy=box))
        return merge_detections(items, confidence, iou=.7)

    def detect(self, image, confidence=.25, angles=(0,90,180,270)):
        started = time.monotonic()
        height, width = image.shape[:2]
        rotations = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}
        items = []
        for angle in angles:
            rotated = image if angle == 0 else cv2.rotate(image, rotations[angle])
            for item in self.detect_one(rotated, confidence):
                item['xyxy'] = unrotate_box(item['xyxy'], width, height, angle)
                item['source_angle'] = angle
                items.append(item)
        return merge_detections(items, confidence), (time.monotonic()-started)*1000


def annotate(image, items):
    result = image.copy()
    for item in items:
        x1,y1,x2,y2 = [round(x) for x in item['xyxy']]
        cv2.rectangle(result, (x1,y1), (x2,y2), (30,220,30), 2)
        cv2.putText(result, f"person {item['confidence']:.2f}", (x1,max(16,y1-5)),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, (30,220,30), 1, cv2.LINE_AA)
    return result


class AngleSearch:
    """Retain a successful angle; probe one alternate on loss or every fifth frame.

    All merged boxes come from the SAME current image. No historical boxes are reused.
    Ground default costs one or two predictions. The flight budget uses one
    prediction per frame, holds a successful angle and rotates after a miss.
    """
    def __init__(self, detector):
        self.detector=detector
        self.angle=0
        self.probe=1
        self.frame=0
        self.last_budget=2
        self.flight_tracking=False
        self.flight_search_angle=0

    def detect(self, image, confidence=.25, max_angle_passes=2):
        if type(max_angle_passes) is not int or max_angle_passes not in (1,2):
            raise ValueError('max_angle_passes must be 1 or 2')
        started=time.monotonic()
        if max_angle_passes==1:
            # Same model, all four orientations, one CURRENT image prediction.
            # Retain successful orientation; rotate across frames after a miss.
            if self.last_budget!=1:self.flight_tracking=True
            primary=self.angle if self.flight_tracking else self.flight_search_angle
            items,_=self.detector.detect(image,confidence,angles=(primary,))
            if items:
                self.angle=primary;self.flight_tracking=True
            else:
                self.flight_tracking=False;self.flight_search_angle=(primary+90)%360
            self.last_budget=1;self.frame+=1
            return items,(time.monotonic()-started)*1000,dict(angles_used=[primary],held_angle=self.angle,
                        direction_state='TRACKING' if items else 'SEARCHING')
        self.last_budget=2
        primary=self.angle
        items,_=self.detector.detect(image,confidence,angles=(primary,))
        used=[primary]
        self.frame+=1
        if not items or self.frame%5==0:
            angles=(0,90,180,270)
            while angles[self.probe%4]==primary:
                self.probe+=1
            alternate=angles[self.probe%4]
            self.probe+=1
            extra,_=self.detector.detect(image,confidence,angles=(alternate,))
            used.append(alternate)
            if extra and (len(extra)>len(items) or
                          (len(extra)==len(items) and sum(x['confidence'] for x in extra)>
                           sum(x['confidence'] for x in items)+.05)):
                self.angle=alternate
            items=merge_detections(items+extra,confidence)
        return items,(time.monotonic()-started)*1000,dict(angles_used=used,held_angle=self.angle,
                    direction_state='TRACKING' if items else 'SEARCHING')
