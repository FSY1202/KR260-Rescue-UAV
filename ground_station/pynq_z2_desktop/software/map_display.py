"""BTN1-triggered map display with animated flight trajectory.

Loads an SVG or a preset 2-D point cloud plus a JSON route, then plays a
60-second animation showing the aircraft moving along the path.
The animation starts on first display and continues while hidden.
This is a local prerecorded trajectory, not live aircraft telemetry.

Usage (on board):
    python3 map_display.py \
        --map      /home/xilinx/pynq_z2_desktop/data/map.svg \
        --trajectory /home/xilinx/pynq_z2_desktop/data/trajectory.json \
        --button-control /run/pynq-desktop/map-button.json \
        --duration 60
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
import tempfile
from collections import deque
from pathlib import Path

import cv2
import numpy as np
from point_cloud_map import Polyline, PointCloudMap, draw_quadcopter, draw_route, draw_cloud_hud

# Limit OpenCV thread pool to avoid competing with real-time PL work.
cv2.setNumThreads(1)

DISPLAY_W = 800
DISPLAY_H = 600

# Colours (BGRA)
COLOUR_SKY        = (40,  30,  20, 255)   # very dark background
COLOUR_TRAIL      = (0,  180, 255, 255)   # orange trail
COLOUR_TRAIL_PAST = (0,  100, 160, 255)   # dimmer older trail
COLOUR_PLANE_BODY = (255, 255, 255, 255)  # white aircraft
COLOUR_PLANE_WING = (180, 220, 255, 255)  # light blue wings
COLOUR_PLANE_OUT  = (0,    60, 180, 255)  # dark blue outline
COLOUR_TEXT       = (255, 255, 255, 255)
COLOUR_HUD_BG     = (0,    0,   0, 160)   # semi-transparent header

FONT      = cv2.FONT_HERSHEY_SIMPLEX
FONT_BOLD = cv2.FONT_HERSHEY_DUPLEX

# ── SVG rendering ────────────────────────────────────────────────────────────

def render_svg(svg_path: str, width: int, height: int) -> np.ndarray:
    """Render once, using optional CairoSVG or installed system SVG loaders."""
    png_bytes = None
    errors = []
    try:
        import cairosvg
        png_bytes = cairosvg.svg2png(url=svg_path, output_width=width,
                                    output_height=height)
        print('MAP_RENDERER cairosvg', flush=True)
    except Exception as exc:
        errors.append(str(exc))
    if png_bytes is None:
        with tempfile.TemporaryDirectory(prefix='pynq-map-') as directory:
            target = str(Path(directory)/'map.png')
            commands = [
                ['rsvg-convert', '-w', str(width), '-h', str(height),
                 '-o', target, svg_path],
                ['/usr/bin/python3', '-c',
                 'import gi,sys;gi.require_version("GdkPixbuf","2.0");'
                 'from gi.repository import GdkPixbuf;'
                 'p=GdkPixbuf.Pixbuf.new_from_file_at_scale(sys.argv[1],'
                 'int(sys.argv[3]),int(sys.argv[4]),False);'
                 'p.savev(sys.argv[2],"png",[],[])',
                 svg_path, target, str(width), str(height)],
            ]
            for command in commands:
                try:
                    subprocess.run(command, check=True, capture_output=True, timeout=20)
                    png_bytes = Path(target).read_bytes()
                    print('MAP_RENDERER '+command[0], flush=True)
                    break
                except Exception as exc:
                    errors.append(str(exc))
    if png_bytes is None:
        raise RuntimeError('Cannot render SVG: '+'; '.join(errors))
    img = cv2.imdecode(np.frombuffer(png_bytes, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if img is None:
        raise ValueError('SVG renderer did not produce a valid image')
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGRA)
    elif img.shape[2] == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2BGRA)
    return cv2.resize(img, (width, height)).astype(np.uint8)


# ── Trajectory helpers ───────────────────────────────────────────────────────

def load_trajectory(path: str, duration_override=None):
    data = json.loads(Path(path).read_text())
    duration = float(data.get('duration_seconds', 60) if duration_override is None
                     else duration_override)
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError('Trajectory duration must be finite and positive')
    if data.get('coordinate_system', 'pixel') != 'pixel':
        raise ValueError('Only prerecorded pixel trajectories are supported')
    size = np.asarray(data.get('map_size', [DISPLAY_W, DISPLAY_H]), dtype=float)
    pts = np.asarray(data['points'], dtype=float)
    if (size.shape != (2,) or not np.isfinite(size).all() or np.any(size <= 0)
            or pts.ndim != 2 or pts.shape[1:] != (2,) or len(pts) < 2
            or not np.isfinite(pts).all()):
        raise ValueError('Invalid map size or trajectory points')
    pts = pts * np.array([DISPLAY_W, DISPLAY_H]) / size
    if np.any(pts < 0) or np.any(pts > [DISPLAY_W-1, DISPLAY_H-1]):
        raise ValueError('Trajectory leaves the map boundaries')
    return [tuple(p) for p in pts], duration


class AnimationClock:
    def __init__(self, duration):
        self.duration = duration
        self.started = None

    def start(self, now):
        if self.started is None:
            self.started = now

    def elapsed(self, now):
        return 0.0 if self.started is None else min(self.duration, max(0.0, now-self.started))


def interpolate_position(pts, t: float, duration: float):
    x, y, heading = Polyline(pts, duration).at(t)
    return round(x), round(y), heading


def draw_aircraft(frame, cx, cy, angle_deg, scale=1.):
    draw_quadcopter(frame, cx, cy, angle_deg, scale)


# ── HUD overlay ──────────────────────────────────────────────────────────────

def draw_hud(frame, elapsed: float, duration: float, visible_pts_count: int):
    # No visible timer, percentage, or progress bar; time is internal only.
    cv2.rectangle(frame, (0, 0), (799, 36), (21, 17, 10, 255), -1)
    cv2.putText(frame, 'PRESET MAP', (22, 24), FONT, .6,
                (238, 220, 173, 255), 1, cv2.LINE_AA)


# ── Main loop ────────────────────────────────────────────────────────────────

def read_sequence(path: Path):
    try:
        return int(json.loads(path.read_text())['sequence'])
    except Exception:
        return 0


def main():
    ap = argparse.ArgumentParser()
    source = ap.add_mutually_exclusive_group(required=True)
    source.add_argument('--map')
    source.add_argument('--point-cloud')
    ap.add_argument('--trajectory', required=True)
    ap.add_argument('--button-control', type=Path)
    ap.add_argument('--selection-control', type=Path)
    ap.add_argument('--duration', type=float, default=None)
    ap.add_argument('--render-check', type=Path)
    ap.add_argument('--preview-time', type=float, default=0.)
    ap.add_argument('--data-source', choices=['demo','live'], default=os.environ.get('GROUND_STATION_SOURCE','demo'))
    ap.add_argument('--telemetry-state',type=Path,default=Path(os.environ.get('TELEMETRY_STATE','/run/pynq-telemetry/state.json')))
    ap.add_argument('--map-calibration',type=Path,default=Path(__file__).resolve().parents[1]/'data/telemetry_map.json')
    args = ap.parse_args()
    traj, duration = load_trajectory(args.trajectory, args.duration)
    path = Polyline(traj, duration)
    cloud = PointCloudMap(args.point_cloud, path) if args.point_cloud else None
    base_frame = render_svg(args.map, DISPLAY_W, DISPLAY_H) if cloud is None else None
    live_map = None
    if args.data_source == 'live':
        from telemetry import TelemetryReader
        from telemetry_view import LiveMapView, MapCalibration
        background = np.empty((DISPLAY_H,DISPLAY_W,4),np.uint8)
        if cloud is not None:
            cloud.draw(background,duration,path)
        else:
            np.copyto(background,base_frame)
        cv2.polylines(background,[np.rint(path.points).astype(np.int32)],False,(75,70,55,255),1,cv2.LINE_8)
        live_map = LiveMapView(background,TelemetryReader(args.telemetry_state),
                              MapCalibration(json.loads(args.map_calibration.read_text())))

    def render(frame, elapsed):
        if live_map is not None:
            live_map.draw(frame)
            return
        if cloud is None:
            np.copyto(frame, base_frame)
            count = 0
        else:
            count = cloud.draw(frame, elapsed, path)
        draw_route(frame, path, elapsed)
        draw_quadcopter(frame, *path.at(elapsed))
        if cloud is None:
            draw_hud(frame, elapsed, duration, count)
    print(f'MAP_DISPLAY loaded {len(traj)} points, {duration}s; source={args.data_source}', flush=True)
    if args.render_check:
        preview = np.empty((DISPLAY_H, DISPLAY_W, 4), np.uint8)
        render(preview, min(duration, max(0., args.preview_time)))
        if not cv2.imwrite(str(args.render_check), preview):
            raise RuntimeError('Cannot write map preview')
        return

    from fast_surface import FastSurface
    stop = [False]
    requested = [None]
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGUSR1, lambda *_: requested.__setitem__(0, True))
    signal.signal(signal.SIGUSR2, lambda *_: requested.__setitem__(0, False))
    clock = AnimationClock(duration)
    sequence = read_sequence(args.button_control) if args.button_control else 0
    selection_token = None
    visible = False
    surface = None
    frames = 0
    rendered = deque()
    last_render_elapsed = None
    next_status = next_screen_wake = 0.0
    visible_seconds = 0.0
    last_loop = time.monotonic()
    visible_at_last_loop = False
    render_seconds = 0.0
    status = Path(os.environ.get('XDG_RUNTIME_DIR', '/run/pynq-xfce'))/'map-control-status.json'

    def record(now):
        while rendered and now-rendered[0] > 1:
            rendered.popleft()
        rate = ((len(rendered)-1)/(rendered[-1]-rendered[0])
                if len(rendered) > 1 else 0.0)
        value = {'pid': os.getpid(), 'visible': visible, 'sequence': sequence,
                 'elapsed_seconds': round(clock.elapsed(now), 3),
                 'duration_seconds': duration, 'started': clock.started is not None,
                 'complete': live_map is None and clock.elapsed(now) >= duration,
                 'display_fps': round(rate, 2), 'display_frames': frames,
                 'updated_monotonic': now, 'source': 'udp_telemetry' if live_map else ('preset_2d_point_cloud' if cloud else 'prerecorded_demo'),
                 'telemetry_connected': live_map.reader.read(now)[1] if live_map else False,
                 'visible_points': cloud.count if cloud else 0,
                 'total_points': len(cloud.points) if cloud else 0,
                 'visible_playback_seconds': round(visible_seconds, 4),
                 'mean_render_ms': round(1000*render_seconds/frames, 3) if frames else 0.}
        temporary = status.with_suffix('.tmp')
        temporary.write_text(json.dumps(value))
        temporary.replace(status)

    try:
        while not stop[0]:
            now = time.monotonic()
            if visible_at_last_loop and clock.started is not None:
                visible_seconds += max(0., min(now, clock.started+duration)-last_loop)
            last_loop = now
            selection = None
            if args.selection_control:
                try:
                    selection = json.loads(args.selection_control.read_text())
                    token = (selection['revision'], selection['view'])
                    if token != selection_token:
                        visible = selection['view'] == 'map'
                        selection_token = token
                except (OSError, ValueError, KeyError):
                    selection = None
            if args.button_control:
                new_seq = read_sequence(args.button_control)
                if selection is None:
                    if new_seq < sequence:
                        visible = False
                    elif (new_seq-sequence) % 2:
                        visible = not visible
                sequence = new_seq
            if requested[0] is not None:
                visible = requested[0]
                requested[0] = None
            if visible and surface is None:
                surface = FastSurface(source_width=DISPLAY_W, source_height=DISPLAY_H,
                                      target_width=DISPLAY_W, target_height=DISPLAY_H,
                                      fullscreen=True)
                clock.start(now)
                last_render_elapsed = None
                next_status = next_screen_wake = 0
                print('MAP_DISPLAY window opened', flush=True)
            if surface is not None and (not visible or surface.closed()):
                surface.close()
                surface = None
                visible = False
                next_status = 0
                rendered.clear()
                print('MAP_DISPLAY window closed', flush=True)
            elapsed = clock.elapsed(now)
            if surface is not None and (live_map is not None or elapsed < duration or last_render_elapsed != elapsed):
                index = surface.acquire()
                if index >= 0:
                    frame = surface.images[index]
                    render_start = time.perf_counter()
                    render(frame, elapsed)
                    render_seconds += time.perf_counter()-render_start
                    surface.present(index)
                    last_render_elapsed = elapsed
                    frames += 1
                    rendered.append(time.monotonic())
            if surface is not None and now >= next_screen_wake:
                for command in (['/usr/bin/xset', 's', 'reset'],
                                ['/usr/bin/xset', 'dpms', 'force', 'on']):
                    subprocess.run(command, capture_output=True, timeout=2, check=False)
                next_screen_wake = now+60
            if now >= next_status:
                record(now)
                next_status = now+1
            visible_at_last_loop = visible
            budget = 1/30 if visible and (live_map is not None or elapsed < duration) else 0.05
            time.sleep(max(0.001, budget-(time.monotonic()-now)))
    finally:
        if surface is not None:
            surface.close()
        visible = False
        record(time.monotonic())
        print('MAP_DISPLAY exiting', flush=True)


if __name__ == '__main__':
    main()
