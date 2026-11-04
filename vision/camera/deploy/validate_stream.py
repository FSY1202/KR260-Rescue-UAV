#!/usr/bin/env python3
"""Camera-only evidence: frozen color bars or an existing HTTP preview bench."""
import argparse
import json
import os
from pathlib import Path
import threading
import time
import urllib.request


def bars(output):
    import cv2
    import numpy as np
    from frame_grabber import FrameGrabber
    from ov5640_camera import Sensor
    from vision_core import rgb565_to_bgr
    sensor, grabber = Sensor(), None
    try:
        sensor.write(0x503d, 0x80)
        time.sleep(.3)
        grabber = FrameGrabber()
        snapshots, banks, reference = [], set(), None
        for _ in range(15):
            pixels, info = grabber.capture()
            if reference is None:
                reference = pixels.copy()
            elif not np.array_equal(reference, pixels):
                raise RuntimeError('Color bars changed across frozen frame banks')
            banks.add(info['buffer_bank'])
            snapshots.append(info)
        if banks != {0, 1, 2}:
            raise RuntimeError(f'Test did not exercise every bank: {banks}')
        image = rgb565_to_bgr(reference)
        b, g, r = [image[:, :, n].astype(np.int16) for n in range(3)]
        counts = dict(red=int(((r >= 96) & (r-g >= 32) & (r-b >= 32)).sum()),
                      green=int(((g >= 80) & (g-r >= 24) & (g-b >= 24)).sum()),
                      blue=int(((b >= 80) & (b-r >= 24) & (b-g >= 24)).sum()))
        if counts != dict(red=39360, green=79680, blue=39360):
            raise RuntimeError(f'Color counts differ from known-good FRM1 bars: {counts}')
        if not cv2.imwrite(str(output.with_suffix('.png')), image):
            raise RuntimeError('Cannot save color bar evidence')
        result = dict(pass_test=True, banks=sorted(banks), frames=len(snapshots),
                      counts=counts, snapshots=snapshots)
        output.write_text(json.dumps(result, indent=2))
        print(json.dumps({k: v for k, v in result.items() if k != 'snapshots'}), flush=True)
    finally:
        if grabber is not None:
            grabber.close()
        try:
            sensor.write(0x503d, 0)
        finally:
            os.close(sensor.fd)


def bench(output, seconds):
    import cv2
    import numpy as np
    stopped = threading.Event()
    stream = dict(frames=0, jpeg_bytes=0, errors=[], geometry=None)

    def receive():
        try:
            with urllib.request.urlopen('http://127.0.0.1:8080/stream.mjpg', timeout=3) as response:
                while not stopped.is_set():
                    line = response.readline()
                    if not line:
                        raise RuntimeError('MJPEG stream closed unexpectedly')
                    if line.strip() != b'--frame':
                        continue
                    length = None
                    while True:
                        line = response.readline()
                        if not line:
                            raise RuntimeError('MJPEG header ended unexpectedly')
                        if line.lower().startswith(b'content-length:'):
                            length = int(line.split(b':', 1)[1])
                        if line in (b'\r\n', b'\n'):
                            break
                    if length is None or not 0 < length < 2000000:
                        raise RuntimeError('Invalid MJPEG frame length')
                    data = response.read(length)
                    if len(data) != length or not data.startswith(b'\xff\xd8') or not data.endswith(b'\xff\xd9'):
                        raise RuntimeError('Broken MJPEG image')
                    if stream['frames'] == 0:
                        image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
                        if image is None or image.shape != (480, 640, 3):
                            raise RuntimeError('Wrong JPEG geometry')
                        stream['geometry'] = list(image.shape)
                    stream['frames'] += 1
                    stream['jpeg_bytes'] += length
        except Exception as exc:
            if not stopped.is_set():
                stream['errors'].append(str(exc))

    reader = threading.Thread(target=receive, daemon=True)
    reader.start()
    samples = []
    started = time.monotonic()
    try:
        while time.monotonic() - started < seconds:
            with urllib.request.urlopen('http://127.0.0.1:8080/status.json', timeout=3) as response:
                sample = json.load(response)
            sample['observed_monotonic'] = time.monotonic()
            samples.append(sample)
            time.sleep(.5)
    finally:
        stopped.set()
        reader.join(timeout=4)
    elapsed = time.monotonic()-started
    first, last = samples[0], samples[-1]
    interval = last['observed_monotonic']-first['observed_monotonic']
    age = sorted(s['age_ms'] for s in samples)
    summary = dict(seconds=elapsed, samples=len(samples),
                   all_ready=all(s['state'] == 'READY' for s in samples),
                   actual_preview_fps=(last['preview_frames']-first['preview_frames'])/interval,
                   sensor_fps=(last['sensor_frames']-first['sensor_frames'])/interval,
                   mjpeg_received_fps=stream['frames']/elapsed,
                   copy_ms_mean=sum(s['copy_ms'] for s in samples)/len(samples),
                   convert_ms_mean=sum(s['convert_ms'] for s in samples)/len(samples),
                   encode_ms_mean=sum(s['encode_ms'] for s in samples)/len(samples),
                   age_p95_ms=age[min(len(age)-1, int(.95*(len(age)-1)))],
                   errors_first=first['capture_errors'], errors_last=last['capture_errors'],
                   overflows_first=first['fifo_overflows'], overflows_last=last['fifo_overflows'],
                   skipped_first=first['skipped_frames'], skipped_last=last['skipped_frames'],
                   stream=stream)
    output.write_text(json.dumps(dict(summary=summary, samples=samples), indent=2))
    print(json.dumps(summary), flush=True)
    if (not summary['all_ready'] or summary['errors_first'] != summary['errors_last'] or
            summary['overflows_first'] != summary['overflows_last'] or stream['errors'] or
            not stream['frames']):
        raise RuntimeError('Preview regression; inspect evidence JSON')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['bars', 'bench'])
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 600:
        parser.error('seconds must be between 1 and 600')
    if args.output.exists() or (args.action == 'bars' and args.output.with_suffix('.png').exists()):
        parser.error('refusing to overwrite existing evidence')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.action == 'bars':
        bars(args.output)
    else:
        bench(args.output, args.seconds)
