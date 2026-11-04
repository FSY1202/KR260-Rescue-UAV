"""BTN2 QGC-inspired demo instruments. Contains no vehicle control or telemetry I/O."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import cv2
import numpy as np
from flight_demo import DemoClock, DemoFlight
from flight_native import HorizonBlitter

cv2.setNumThreads(1)
ROOT = Path(__file__).resolve().parents[1]


def load_image(path):
    image = cv2.imdecode(np.frombuffer(Path(path).read_bytes(), np.uint8), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError('Invalid flight UI image: '+str(path))
    if image.ndim != 3 or image.shape[2] not in (3, 4):
        raise ValueError('Flight UI image must be RGB/RGBA')
    return cv2.cvtColor(image, cv2.COLOR_BGR2BGRA) if image.shape[2] == 3 else image


def horizon_matrix(roll, pitch):
    matrix = cv2.getRotationMatrix2D((320, 320), roll, 1.)
    angle = math.radians(roll)
    matrix[0, 2] += 206-320+math.sin(angle)*pitch*3
    matrix[1, 2] += 140-320+math.cos(angle)*pitch*3
    return matrix


def compass_left(heading):
    return round(((heading % 360)+360)*3)-206


class FlightRenderer:
    def __init__(self, assets):
        assets = Path(assets)
        self.layout = json.loads((assets/'layout.json').read_text())
        if self.layout.get('format') != 'FLIGHT_UI_1':
            raise ValueError('Incompatible flight UI assets')
        self.background = load_image(assets/'background.png')
        self.horizon = load_image(assets/'horizon.png')
        self.compass = load_image(assets/'compass.png')
        overlay = load_image(assets/'overlay.png')
        self.sprites = load_image(assets/'states.png')
        self.digits_large = load_image(assets/'digits_large.png')
        self.digits_small = load_image(assets/'digits_small.png')
        if (self.background.shape != (600,800,4) or self.horizon.shape != (640,640,4)
                or self.compass.shape != (48,3240,4) or overlay.shape != (280,412,4)
                or self.sprites.shape != (210,180,4)):
            raise ValueError('Incorrect flight UI asset sizes')
        self.oy, self.ox = np.where(overlay[:,:,3] != 0)
        self.overlay = overlay[self.oy,self.ox]
        self.canvas = self.background.copy()
        self.tick = None
        self.values = [None]*6
        self.header = None
        self.blitter = HorizonBlitter()

    def digits(self, frame, value, x, y, small=False):
        atlas = self.digits_small if small else self.digits_large
        cw,ch = (17,25) if small else (24,36)
        for i,char in enumerate(value):
            index='0123456789+-.'.index(char)
            frame[y:y+ch,x+i*cw:x+(i+1)*cw] = atlas[:,index*cw:(index+1)*cw]

    def update_values(self, state, elapsed):
        tick = int(elapsed*10)
        if self.tick == tick:
            return
        self.tick = tick
        values = [f'{state.relative_altitude_m:.2f}', f'{state.ground_speed_mps:.2f}',
                  f'{state.climb_rate_mps:+.2f}', f'{state.voltage_v:.2f}',
                  f'{state.current_a:.1f}', f'{state.battery_percent:.0f}']
        if state.source.startswith('udp_'):
            # Large real-world values must fit the existing fixed-width cards.
            numbers=[state.relative_altitude_m,state.ground_speed_mps,state.climb_rate_mps,
                     state.voltage_v,state.current_a,state.battery_percent]
            for i,value in enumerate(values):
                if len(value)>5:
                    values[i]=f'{numbers[i]:.1f}'
                if len(values[i])>5:
                    values[i]=f'{numbers[i]:.0f}'
        for index,((x,y,_,_), value) in enumerate(zip(self.layout['cards'], values)):
            if value == self.values[index]:
                continue
            self.values[index] = value
            region = np.s_[y+43:y+91,x+10:x+142]
            self.canvas[region] = self.background[region]
            self.digits(self.canvas,value,x+14,y+49)
        if self.header == (state.mode,state.armed,state.phase):
            return
        self.header = (state.mode,state.armed,state.phase)
        self.canvas[65:96,106:238] = self.background[65:96,106:238]
        cv2.putText(self.canvas,state.mode,(108,88),0,.51,(224,232,219,255),1,cv2.LINE_AA)
        # Preserve opaque display pixels across OpenCV antialiasing versions.
        self.canvas[65:96,106:238,3] = 255
        armed_index = 6 if state.armed else 5
        self.canvas[65:95,240:420] = self.sprites[armed_index*30:armed_index*30+30]
        phase_index = ['standby','takeoff','cruise','hover','landing'].index(state.phase)
        self.canvas[65:95,440:530] = self.sprites[phase_index*30:phase_index*30+30,:90]

    def draw(self, frame, state, elapsed):
        if frame.shape != (600,800,4) or frame.dtype != np.uint8:
            raise ValueError('Flight surface must be 800x600 BGRA')
        self.update_values(state,elapsed)
        np.copyto(frame,self.canvas)
        roi = frame[120:400,28:440]
        self.blitter.draw(roi,self.horizon,horizon_matrix(state.roll_deg,state.pitch_deg))
        roi[self.oy,self.ox] = self.overlay
        left = compass_left(state.heading_deg)
        frame[420:468,28:440] = self.compass[:,left:left+412]
        cv2.fillConvexPoly(frame,np.array([[234,425],[229,419],[239,419]],np.int32),
                          (95,220,255,255),cv2.LINE_AA)
        for x,value in ((30,state.heading_deg),(174,state.roll_deg),(311,state.pitch_deg)):
            self.digits(frame,f'{value:.1f}',x,496,small=True)


def read_json(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError, TypeError):
        return {}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--assets',type=Path,default=ROOT/'data/flight_status')
    p.add_argument('--config',type=Path,default=ROOT/'data/flight_demo.json')
    p.add_argument('--selection-control',type=Path,default=Path('/run/pynq-desktop/view-selection.json'))
    p.add_argument('--button-control',type=Path,default=Path('/run/pynq-desktop/flight-status-button.json'))
    p.add_argument('--fps',type=float,default=20.)
    p.add_argument('--snapshot',type=Path)
    p.add_argument('--at',type=float,default=0.)
    p.add_argument('--data-source',choices=['demo','live'],default=os.environ.get('GROUND_STATION_SOURCE','demo'))
    p.add_argument('--telemetry-state',type=Path,default=Path(os.environ.get('TELEMETRY_STATE','/run/pynq-telemetry/state.json')))
    a = p.parse_args()
    if not math.isfinite(a.fps) or not 1 <= a.fps <= 30:
        p.error('--fps must be between 1 and 30')
    demo = DemoFlight.from_file(a.config)
    renderer = FlightRenderer(a.assets)
    live_flight = None
    if a.data_source == 'live':
        from telemetry import TelemetryReader
        from telemetry_view import LiveFlightView
        live_flight = LiveFlightView(TelemetryReader(a.telemetry_state))

    def sample(elapsed):
        return live_flight.sample() if live_flight else demo.sample(elapsed)

    def render(frame, elapsed):
        renderer.draw(frame,sample(elapsed),elapsed)
        if live_flight:
            live_flight.overlay(frame)
    if a.snapshot:
        frame = np.empty((600,800,4),np.uint8)
        render(frame,a.at)
        if not cv2.imwrite(str(a.snapshot),frame):
            raise RuntimeError('Cannot write flight preview')
        return
    from fast_surface import FastSurface
    stopping = [False]
    requested = [None]
    signal.signal(signal.SIGTERM,lambda *_:stopping.__setitem__(0,True))
    signal.signal(signal.SIGINT,lambda *_:stopping.__setitem__(0,True))
    signal.signal(signal.SIGUSR1,lambda *_:requested.__setitem__(0,True))
    signal.signal(signal.SIGUSR2,lambda *_:requested.__setitem__(0,False))
    clock = DemoClock()
    visible = False
    surface = None
    sequence = int(read_json(a.button_control).get('sequence',0))
    selection_token = None
    frames = 0
    render_seconds = 0.
    visible_seconds = 0.
    last_loop = time.monotonic()
    visible_before = False
    displayed = deque()
    next_status = next_wake = 0.
    status = Path(os.environ.get('XDG_RUNTIME_DIR','/run/pynq-xfce'))/'flight-status-view.json'

    def record(now):
        while displayed and now-displayed[0] > 1.5:
            displayed.popleft()
        fps = ((len(displayed)-1)/(displayed[-1]-displayed[0]) if len(displayed)>1 else 0.)
        state=sample(clock.elapsed(now))
        value = dict(pid=os.getpid(),source=state.source,visible=visible,sequence=sequence,
                     telemetry_connected=live_flight.connected if live_flight else False,
                     raster_backend=renderer.blitter.backend,
                     started=clock.started is not None,elapsed_seconds=clock.elapsed(now),
                     display_frames=frames,display_fps=round(fps,3),requested_fps=a.fps,
                     visible_seconds=visible_seconds,updated_monotonic=now,
                     mean_render_ms=1000*render_seconds/frames if frames else 0.,
                     state=state.to_dict() if live_flight is None or live_flight.connected else None)
        temporary = status.with_suffix('.tmp')
        temporary.write_text(json.dumps(value))
        temporary.replace(status)

    print(f'FLIGHT_STATUS_READY source={a.data_source} receive_only=true',flush=True)
    try:
        while not stopping[0]:
            now = time.monotonic()
            if visible_before:
                visible_seconds += now-last_loop
            last_loop = now
            selection = read_json(a.selection_control)
            if selection.get('view') is not None and 'revision' in selection:
                token = (selection['revision'],selection['view'])
                if token != selection_token:
                    visible = selection['view'] == 'flight_status'
                    selection_token = token
                    next_status = 0
            new_sequence = int(read_json(a.button_control).get('sequence',sequence))
            if not selection:
                if new_sequence < sequence:
                    visible = False
                elif (new_sequence-sequence)%2:
                    visible = not visible
            sequence = new_sequence
            if requested[0] is not None:
                visible = requested[0]
                requested[0] = None
                next_status = 0
            if visible and surface is None:
                surface = FastSurface(800,600,True,800,600)
                clock.start(time.monotonic())
                next_wake = next_status = 0
                print('FLIGHT_STATUS_SHOW',flush=True)
            if surface is not None and (not visible or surface.closed()):
                surface.close()
                surface = None
                visible = False
                displayed.clear()
                next_status = 0
                print('FLIGHT_STATUS_HIDE',flush=True)
            if surface is not None:
                index = surface.acquire()
                if index >= 0:
                    render_start = time.perf_counter()
                    elapsed = clock.elapsed(time.monotonic())
                    render(surface.images[index],elapsed)
                    surface.present(index)
                    render_seconds += time.perf_counter()-render_start
                    frames += 1
                    displayed.append(time.monotonic())
            if visible and now >= next_wake:
                for command in (['/usr/bin/xset','s','reset'],['/usr/bin/xset','dpms','force','on']):
                    subprocess.run(command,capture_output=True,timeout=2,check=False)
                next_wake = now+60
            if now >= next_status:
                record(time.monotonic())
                next_status = now+1
            visible_before = visible
            budget = 1/a.fps if visible else .05
            time.sleep(max(.001,budget-(time.monotonic()-now)))
    finally:
        if surface is not None:
            surface.close()
        visible = False
        record(time.monotonic())
        print('FLIGHT_STATUS_EXIT',flush=True)


if __name__ == '__main__':
    main()
