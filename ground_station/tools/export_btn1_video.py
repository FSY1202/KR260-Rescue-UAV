"""Export the deployed BTN1 renderer and resources as a portable H.264 video."""
import json
from pathlib import Path
import subprocess
import sys

import cv2
import imageio_ffmpeg
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / 'pynq_z2_desktop'
sys.path.insert(0, str(PROJECT / 'software'))
from point_cloud_map import PointCloudMap, Polyline, draw_route, draw_quadcopter


def main():
    cv2.setNumThreads(1)
    metadata = json.loads((PROJECT / 'data/trajectory.json').read_text())
    duration = float(metadata['duration_seconds'])
    path = Polyline(metadata['points'], duration)
    cloud = PointCloudMap(PROJECT / 'data/map_cloud.npz', path)
    output = ROOT / 'preview-output/BTN1_map_60s.mp4'
    output.parent.mkdir(exist_ok=True)
    fps = 30
    process = subprocess.Popen([
        imageio_ffmpeg.get_ffmpeg_exe(), '-y', '-loglevel', 'error',
        '-f', 'rawvideo', '-pixel_format', 'bgra', '-video_size', '800x600',
        '-framerate', str(fps), '-i', '-', '-an', '-c:v', 'libx264',
        '-preset', 'medium', '-crf', '18', '-pix_fmt', 'yuv420p',
        '-movflags', '+faststart', str(output)], stdin=subprocess.PIPE)
    frame = np.empty((600, 800, 4), np.uint8)
    try:
        count = round(duration * fps)
        for index in range(count):
            # Include both endpoints while retaining exactly 60 s at 30 fps.
            elapsed = duration * index / (count - 1)
            cloud.draw(frame, elapsed, path)
            draw_route(frame, path, elapsed)
            draw_quadcopter(frame, *path.at(elapsed))
            process.stdin.write(frame.tobytes())
            if index in (0, count // 2, count - 1):
                cv2.imwrite(str(ROOT / f'preview-output/BTN1_video_check_{index:04d}.png'), frame)
            if index % 300 == 0:
                print(f'Exported {index}/{count} frames', flush=True)
    finally:
        process.stdin.close()
    if process.wait() != 0:
        raise RuntimeError('Video encoder failed')
    reader = cv2.VideoCapture(str(output))
    info = {'file': str(output), 'frames': int(reader.get(cv2.CAP_PROP_FRAME_COUNT)),
            'fps': reader.get(cv2.CAP_PROP_FPS),
            'width': int(reader.get(cv2.CAP_PROP_FRAME_WIDTH)),
            'height': int(reader.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            'source': 'offline export of deployed BTN1 renderer; not board capture'}
    for index in (0, count // 2, count - 1):
        reader.set(cv2.CAP_PROP_POS_FRAMES, index)
        ok, decoded = reader.read()
        if not ok or decoded.shape != (600, 800, 3):
            raise RuntimeError(f'Cannot decode frame {index}')
    reader.release()
    assert info['frames'] == count and abs(info['fps'] - fps) < .01
    (ROOT / 'preview-output/BTN1_video_info.json').write_text(json.dumps(info, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(info, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
