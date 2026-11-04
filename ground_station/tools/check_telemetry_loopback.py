"""Run receiver + synthetic sender + both view adapters on this computer only."""
import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
SOFTWARE=ROOT/'pynq_z2_desktop/software'
sys.path.insert(0,str(SOFTWARE))
from telemetry import TelemetryReader
from telemetry_view import LiveFlightView, LiveMapView, MapCalibration
from flight_status import FlightRenderer
from point_cloud_map import Polyline, PointCloudMap


def main():
    output=ROOT/'preview-output'
    output.mkdir(exist_ok=True)
    data=ROOT/'pynq_z2_desktop/data'
    route=json.loads((data/'trajectory.json').read_text())
    path=Polyline(route['points'],route['duration_seconds'])
    frame=np.empty((600,800,4),np.uint8)
    PointCloudMap(data/'map_cloud.npz',path).draw(frame,path.duration,path)
    background=frame.copy()
    with tempfile.TemporaryDirectory() as directory:
        state=Path(directory)/'state.json'
        # Get an ephemeral loopback port; no firewall or board changes.
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as probe:
            probe.bind(('127.0.0.1',0));port=probe.getsockname()[1]
        receiver=subprocess.Popen([sys.executable,str(SOFTWARE/'telemetry_receiver.py'),
            '--port',str(port),'--state',str(state),'--timeout','2'],stdout=subprocess.DEVNULL)
        try:
            deadline=time.monotonic()+5
            while not state.exists():
                if receiver.poll() is not None or time.monotonic()>deadline:
                    raise RuntimeError('Receiver did not start')
                time.sleep(.02)
            subprocess.run([sys.executable,str(ROOT/'tools/send_telemetry_demo.py'),
                '--port',str(port),'--duration','.6','--hz','10'],check=True)
            time.sleep(.1)
            snapshot,connected,_=TelemetryReader(state).read()
            assert connected and snapshot['accepted']>=3
            map_view=LiveMapView(background,TelemetryReader(state),MapCalibration(json.loads((data/'telemetry_map.json').read_text())))
            assert map_view.draw(frame)
            assert cv2.imwrite(str(output/'wifi-map.png'),frame)
            flight=LiveFlightView(TelemetryReader(state))
            sample=flight.sample()
            assert flight.connected and sample.source=='udp_test'
            FlightRenderer(data/'flight_status').draw(frame,sample,0)
            flight.overlay(frame)
            assert cv2.imwrite(str(output/'wifi-status.png'),frame)
            # Future receiver-local time proves stopped traffic is stale.
            future=snapshot['received_monotonic']+3
            assert not map_view.draw(frame,future)
            flight.sample(future);flight.overlay(frame)
            assert not flight.connected
            report=dict(transport='127.0.0.1 UDP (not real WLAN)',accepted=snapshot['accepted'],
                        data_origin=snapshot['packet']['data_origin'],map_updated=True,
                        status_updated=True,timeout_masks_values=True,board_tested=False)
            (output/'wifi-loopback-check.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
            print(json.dumps(report))
        finally:
            receiver.terminate()
            try:receiver.wait(timeout=3)
            except subprocess.TimeoutExpired:
                receiver.kill();receiver.wait(timeout=3)


if __name__=='__main__':main()
