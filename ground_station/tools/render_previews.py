"""Render the actual application functions without X11 or a board."""
from pathlib import Path
import json
import sys
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / 'pynq_z2_desktop'
sys.path.insert(0, str(PROJECT / 'software'))
from point_cloud_map import PointCloudMap, Polyline, draw_route, draw_quadcopter
from flight_demo import DemoFlight
from flight_status import FlightRenderer


def main():
    output = ROOT / 'preview-output'
    output.mkdir(exist_ok=True)
    metadata = json.loads((PROJECT / 'data/trajectory.json').read_text())
    path = Polyline(metadata['points'], metadata['duration_seconds'])
    cloud = PointCloudMap(PROJECT / 'data/map_cloud.npz', path)
    frame = np.empty((600, 800, 4), np.uint8)
    for t in (0, 30, 60):
        cloud.draw(frame, t, path)
        draw_route(frame, path, t)
        draw_quadcopter(frame, *path.at(t))
        if not cv2.imwrite(str(output / f'btn1_{t:02d}.png'), frame):
            raise RuntimeError('Cannot save map preview')
    model = DemoFlight(json.loads((PROJECT / 'data/flight_demo.json').read_text()))
    renderer = FlightRenderer(PROJECT / 'data/flight_status')
    for name, t in [('standby',0),('cruise',23),('hover',47),('landed',58)]:
        renderer.draw(frame, model.sample(t), t)
        if not cv2.imwrite(str(output / f'btn2_{name}.png'), frame):
            raise RuntimeError('Cannot save status preview')
    print(output)


if __name__ == '__main__':
    main()
