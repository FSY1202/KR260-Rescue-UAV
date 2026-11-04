"""Live single-hand gesture recognition with the existing V2 PL preprocessing.

FIST / OPEN / V / NONE / UNKNOWN. This uses silhouette rules, not SqueezeNet.
GUI on a board desktop, or JSON status and snapshots over a serial connection.
"""
import argparse
from collections import Counter, defaultdict, deque
import json
import os
from pathlib import Path
import signal
import time

import cv2
import numpy as np
from classify_hand import CLASSIFIER_VERSION, classify_clean, _native
from gesture import DEFAULT_BOUNDS

ROI = (64, 24, 192, 192)
NAMES = {'FIST': 'FIST / ROCK', 'OPEN': 'OPEN / PALM', 'V': 'V / SCISSORS',
         'NONE': 'NO HAND', 'UNKNOWN': 'ADJUST HAND'}


class GestureVote:
    """Five-frame majority; removing the hand immediately clears old votes."""
    def __init__(self):
        self.history = deque(maxlen=5)

    def update(self, label):
        if label == 'NONE':
            self.history.clear()
            return 'NONE'
        self.history.append(label)
        label, votes = Counter(self.history).most_common(1)[0]
        return label if votes >= 3 else 'UNKNOWN'


def recognize(mask):
    info, contour = classify_clean(mask)
    info = dict(info)
    # A skin-coloured wall or overexposure filling the ROI is not a closed fist.
    if contour is not None:
        fraction = info.get('source_area_fraction', cv2.contourArea(contour) / mask.size)
        info['area_fraction'] = round(float(fraction), 3)
        if fraction > .75:
            info.update(gesture='UNKNOWN', reason='ROI_FILLED')
        else:
            info.setdefault('reason', 'OK' if info['gesture'] != 'UNKNOWN' else 'UNCLEAR_SHAPE')
    else:
        info.update(area_fraction=0.0, reason='NO_HAND')
    return info, contour


def atomic_json(path, record):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + '.tmp')
    temp.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding='utf-8')
    os.replace(temp, path)


def stage_summary(sums, recent, frames):
    """Update reporting statistics at log time, outside the per-frame work."""
    if not frames:
        return {}
    return {key: {'mean': value / frames,
                  'recent_p95': float(np.percentile([row[key] for row in recent], 95))}
            for key, value in sums.items()}


def picture(raw, clean, contour, label, backend, fps, frame_id, controls=True):
    frame = cv2.cvtColor(raw, cv2.COLOR_YUV2BGR_YUY2)
    # Preserve native 640x480 camera detail. Draw analysis coordinates only
    # after scaling to the preview canvas; never downsample via 320x240.
    if frame.shape[:2] != (480, 640):
        frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)
    x, y, w, h = ROI
    color = (0, 220, 0) if label in ('FIST', 'OPEN', 'V') else (0, 210, 255)
    cv2.rectangle(frame, (x*2, y*2), ((x+w-1)*2, (y+h-1)*2), (0, 255, 0), 2)
    if contour is not None:
        cv2.drawContours(frame, [contour*2], -1, color, 2, offset=(x*2, y*2))
    cv2.rectangle(frame, (0, 0), (640, 43), (25, 25, 25), -1)
    cv2.putText(frame, NAMES[label], (9, 29), cv2.FONT_HERSHEY_SIMPLEX, .75, color, 2)
    cv2.rectangle(frame, (0, 449), (640, 480), (25, 25, 25), -1)
    footer = f'{backend.upper()}  {fps:.1f} FPS  frame {frame_id}'
    if controls:
        footer += ' | Q: quit  S: save'
    cv2.putText(frame, footer,
                (8, 470), cv2.FONT_HERSHEY_SIMPLEX, .43, (255, 255, 255), 1)
    return frame


def save_snapshot(path, raw, clean, contour, label, backend, fps, frame_id):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    # Keep a reader from observing a partially written JPEG.
    temp = path.with_name(path.stem + '.tmp' + path.suffix)
    if not cv2.imwrite(str(temp), picture(raw, clean, contour, label, backend, fps, frame_id)):
        raise RuntimeError('Cannot write camera snapshot')
    os.replace(temp, path)
    mask_path = path.with_name(path.stem + '-mask.png')
    if not cv2.imwrite(str(mask_path), clean):
        raise RuntimeError('Cannot write mask snapshot')


def self_test():
    # Keep this portable: importing the camera/DMA module requires Linux fcntl.
    from gesture import self_test as old_self_test
    old_self_test()
    masks = []
    open_hand = np.zeros((180,180),np.uint8)
    cv2.rectangle(open_hand,(35,85),(135,145),255,-1)
    for x, top in [(35,45),(57,20),(79,10),(101,20),(123,45)]:
        cv2.rectangle(open_hand,(x,top),(x+12,100),255,-1)
    v_hand=np.zeros_like(open_hand)
    cv2.fillPoly(v_hand,[np.array([[65,145],[115,145],[110,80],[145,30],
        [128,20],[92,68],[60,20],[42,30],[68,80]],np.int32)],255)
    for mask,label in [(open_hand,'OPEN'),(v_hand,'V')]:
        for rotation in range(4):
            info,_=recognize(np.ascontiguousarray(np.rot90(mask,rotation)))
            assert info['gesture']==label,(label,rotation,info)
    circle = np.zeros((192, 192), np.uint8)
    cv2.circle(circle, (96, 96), 32, 255, -1)
    assert recognize(circle)[0]['gesture'] == 'FIST'
    assert recognize(np.zeros_like(circle))[0]['gesture'] == 'NONE'
    assert recognize(np.full_like(circle, 255))[0]['gesture'] == 'UNKNOWN'
    vote = GestureVote()
    assert [vote.update('OPEN') for _ in range(3)] == ['UNKNOWN', 'UNKNOWN', 'OPEN']
    assert vote.update('NONE') == 'NONE'
    assert vote.update('V') == 'UNKNOWN'  # No old OPEN votes survive removal.
    assert vote.update('V') == 'UNKNOWN'
    assert vote.update('V') == 'V'
    assert vote.update('NONE') == 'NONE'
    assert [vote.update('UNKNOWN') for _ in range(5)][-1] == 'UNKNOWN'
    print(json.dumps({'self_test': 'PASS', 'rotation_cases': 12,
                      'no_hand_reset': True, 'filled_roi_rejection': True}), flush=True)


def main():
    root = Path(__file__).resolve().parents[1]
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--backend', choices=['pl', 'cpu'], default='pl')
    ap.add_argument('--camera', default='/dev/video0')
    ap.add_argument('--bitfile', default=str(root/'output/gesture_v2.bit'))
    ap.add_argument('--reuse-overlay', action='store_true', help='Use the already loaded combined desktop overlay')
    ap.add_argument('--headless', action='store_true', help='Auto-enabled without DISPLAY')
    ap.add_argument('--duration', type=float, default=0, help='Seconds; 0 runs until interrupted')
    ap.add_argument('--frames', type=int, default=0, help='0 means no frame limit')
    ap.add_argument('--warmup', type=float, default=3)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--capture-scale', type=int, choices=(1,2), default=1,
                    help='Native YUYV capture: 1=320x240, 2=640x480; analysis stays 320x240')
    ap.add_argument('--local-preview', help='Bounded shared YUYV frame for the HDMI viewer')
    ap.add_argument('--display-fps', type=float, default=15)
    ap.add_argument('--web-host', default='127.0.0.1', help='Preview bind address')
    ap.add_argument('--web-port', type=int, default=0, help='HTTP preview port; 0 disables it')
    ap.add_argument('--web-fps', type=float, default=5, help='Preview rate; classification still runs every frame')
    ap.add_argument('--web-jpeg-quality', type=int, default=90)
    ap.add_argument('--lazy-web-preview',action='store_true',help='Encode JPEG only while HTTP image clients are active')
    ap.add_argument('--exposure-us', type=int, default=20000)
    ap.add_argument('--bounds', type=int, nargs=4, default=DEFAULT_BOUNDS,
                    metavar=('CB_MIN', 'CB_MAX', 'CR_MIN', 'CR_MAX'))
    ap.add_argument('--y-min', type=int, default=20,
                    help='Raw YUYV luminance floor; 20 retains backlit skin shadows')
    ap.add_argument('--status-file', default=str(Path.home()/'gesture-status.json'))
    ap.add_argument('--snapshot', default=str(Path.home()/'gesture-latest.jpg'))
    ap.add_argument('--snapshot-interval', type=float, default=10,
                    help='Periodic snapshot seconds; 0 disables periodic saves')
    ap.add_argument('--report', default=str(Path.home()/'gesture-demo-report.json'))
    ap.add_argument('--self-test', action='store_true')
    args = ap.parse_args()
    cv2.setNumThreads(1)
    if args.duration < 0 or args.frames < 0 or args.warmup < 0 or args.snapshot_interval < 0:
        ap.error('Time and frame limits must be nonnegative')
    if args.fps <= 0 or args.display_fps <= 0 or args.web_fps <= 0 or not 0 <= args.y_min <= 255:
        ap.error('Invalid FPS or luminance threshold')
    if not 0 <= args.web_port <= 65535:
        ap.error('Invalid preview port')
    if not 1 <= args.web_jpeg_quality <= 100:
        ap.error('JPEG quality must be 1..100')
    if (not all(0 <= b <= 255 for b in args.bounds)
            or args.bounds[0] > args.bounds[1] or args.bounds[2] > args.bounds[3]):
        ap.error('Invalid skin bounds')
    if not 100 <= args.exposure_us <= min(500000, 1000000//args.fps) or args.exposure_us % 100:
        ap.error('Exposure must fit one frame, in increments of 100 us')
    if args.self_test:
        self_test()
        return
    from gesture_fast import Capture, Processor, reference
    headless = args.headless or not bool(os.environ.get('DISPLAY'))
    capture = None
    processor = None
    preview = None
    local = None
    stop = [False]
    def on_signal(signum, frame):
        stop[0] = True
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    record = dict(status='starting', backend=args.backend, method='skin_mask_contour_rules',
                  classifier=CLASSIFIER_VERSION,
                  geometry_backend='native_c' if _native is not None else 'numpy',
                  segmentation={'y_min': args.y_min, 'bounds': list(args.bounds)},
                  trained_neural_network=False, headless=headless, roi=list(ROI))
    try:
        atomic_json(args.status_file, record)
        if args.local_preview:
            from local_preview import LocalWriter
            local = LocalWriter(args.local_preview)
        if args.web_port:
            from gesture_web import GestureWeb
            preview = GestureWeb(args.web_host, args.web_port, args.status_file,
                                 lambda *values: picture(*values, controls=False),
                                 jpeg_quality=args.web_jpeg_quality,lazy_jpeg=args.lazy_web_preview)
            record['preview_url'] = f'http://{args.web_host}:{args.web_port}/'
            print('WEB_PREVIEW '+record['preview_url'], flush=True)
        capture = Capture(args.camera, 320, 240, args.fps, args.exposure_us,
                          native_scale=args.capture_scale)
        if args.backend == 'pl':
            processor = Processor(args.bitfile, (192, 192), args.bounds, args.y_min,
                                  decimate2=capture.scale == 2, download=not args.reuse_overlay)
        capture.start()
        record['camera'] = capture.settings
        record['preview'] = {'width':640, 'height':480,
                             'jpeg_quality':args.web_jpeg_quality, 'fps_limit':args.web_fps,
                             'lazy_jpeg':args.lazy_web_preview}
        warm_end = time.perf_counter() + args.warmup
        while time.perf_counter() < warm_end and not stop[0]:
            capture.get()
        if stop[0]:
            record['status'] = 'stopped'
            return
        vote = GestureVote()
        sums = defaultdict(float)
        recent = deque(maxlen=300)
        labels = Counter()
        frames = 0
        windows=Counter()
        recent_frames = deque()
        base_captured = capture.count
        base_drops = capture.dropped
        start = time.perf_counter()
        next_log = start
        next_display = start
        next_web = start
        next_snapshot = start
        last = None
        while not stop[0]:
            frame_id, raw, acquired, read_ms = capture.get()
            pre = time.perf_counter()
            x, y, w, h = ROI
            scale = capture.scale
            raw_roi = raw[y*scale:(y+h)*scale, x*scale:(x+w)*scale]
            if processor:
                clean, stages = processor.process(raw_roi)
            else:
                clean = reference(raw_roi, args.bounds, args.y_min, decimate2=scale == 2)
                stages = {'cpu_preprocess_ms': (time.perf_counter()-pre)*1000}
            cls_start = time.perf_counter()
            info, contour = recognize(clean)
            label = vote.update(info['gesture'])
            cls_ms = (time.perf_counter()-cls_start)*1000
            now = time.perf_counter()
            fps = (frames+1)/max(now-start, 1e-9)
            recent_frames.append(now)
            while len(recent_frames)>2 and recent_frames[1] < now-1:
                recent_frames.popleft()
            live_fps = ((len(recent_frames)-1)/(recent_frames[-1]-recent_frames[0])
                        if len(recent_frames)>1 else 0.0)
            if local:
                local.publish(raw, contour, label, live_fps, frame_id, acquired)
            if preview and now >= next_web:
                preview.publish(raw, contour, label, args.backend, live_fps, frame_id,
                                clean=clean, raw_label=info['gesture'])
                next_web = now + 1/args.web_fps
            if not headless and now >= next_display:
                cv2.imshow('PYNQ-Z2 Gesture', picture(raw, clean, contour, label, args.backend, fps, frame_id))
                cv2.imshow('Gesture Mask', cv2.resize(clean, (384, 384), interpolation=cv2.INTER_NEAREST))
                key = cv2.waitKey(1) & 255
                if key in (ord('q'), 27):
                    stop[0] = True
                if key == ord('s'):
                    save_snapshot(args.snapshot, raw, clean, contour, label, args.backend, fps, frame_id)
                if cv2.getWindowProperty('PYNQ-Z2 Gesture', cv2.WND_PROP_VISIBLE) < 1:
                    stop[0] = True
                next_display = now + 1/args.display_fps
            if args.snapshot_interval and now >= next_snapshot:
                save_snapshot(args.snapshot, raw, clean, contour, label, args.backend, fps, frame_id)
                next_snapshot = now + args.snapshot_interval
            done = time.perf_counter()
            frames += 1
            windows[int(done-start)]+=1
            labels[label] += 1
            row = dict(capture_read_ms=read_ms, classify_ms=cls_ms,
                       frame_age_ms=(done-acquired)*1000, **stages)
            recent.append(row)
            for key, value in row.items():
                sums[key] += value
            record.update(status='running', frame=frames, source_frame=frame_id,
                          raw_gesture=info['gesture'], gesture=label, info=info,
                          fps=frames/(done-start), elapsed_seconds=done-start,
                          recent_fps=live_fps,
                          captured=capture.count-base_captured, dropped=capture.dropped-base_drops,
                          labels=dict(labels), updated_unix=time.time())
            if local:
                record['local_preview'] = {'published':local.published,'skipped':local.skipped}
            last = (raw, clean.copy(), contour, label, frame_id)
            if done >= next_log:
                end_window=int(done-start)
                first_window=max(0,end_window-180)
                record['one_second_counts_start']=first_window
                record['one_second_counts']=[windows[i] for i in range(first_window,end_window)]
                record['stages_ms'] = stage_summary(sums, recent, frames)
                atomic_json(args.status_file, record)
                print(json.dumps({key: record[key] for key in ['frame', 'source_frame', 'raw_gesture',
                                 'gesture', 'fps', 'dropped']}, ensure_ascii=False), flush=True)
                next_log = done + 1
            if args.frames and frames >= args.frames or args.duration and done-start >= args.duration:
                break
        if last:
            save_snapshot(args.snapshot, *last[:4], args.backend, record['fps'], last[4])
        record['stages_ms'] = stage_summary(sums, recent, frames)
        record.update(status='stopped', dma_errors=0)
        print('RESULT '+json.dumps(record, ensure_ascii=False), flush=True)
    except Exception as exc:
        record.update(status='failed', error=f'{type(exc).__name__}: {exc}')
        print(json.dumps(record, ensure_ascii=False), flush=True)
        raise
    finally:
        cleanup_errors = []
        for obj in (local, preview, capture, processor):
            if obj is not None:
                try:
                    obj.close()
                except Exception as exc:
                    cleanup_errors.append(str(exc))
        if cleanup_errors:
            record.update(status='failed', cleanup_errors=cleanup_errors)
        if not headless:
            cv2.destroyAllWindows()
        atomic_json(args.status_file, record)
        atomic_json(args.report, record)


if __name__ == '__main__':
    main()
