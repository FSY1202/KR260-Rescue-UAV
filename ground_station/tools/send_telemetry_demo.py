"""Synthetic sender to check the read-only link; never connects to a flight controller."""
import argparse
import json
from pathlib import Path
import secrets
import socket
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pynq_z2_desktop/software'))
from flight_demo import DemoFlight
from point_cloud_map import Polyline
from telemetry import encode_packet, number
from telemetry_view import MapCalibration


def sample_packet(elapsed, seq, session, vehicle, path, calibration):
    state=DemoFlight().sample(elapsed)
    x,y,angle=path.at(elapsed%path.duration)
    east,north=calibration.inverse([x,y])
    return dict(version=1,data_origin='test',vehicle_id=vehicle,session_id=session,seq=seq,
        position=dict(east_m=float(east),north_m=float(north),valid=True),
        attitude=dict(roll_deg=state.roll_deg,pitch_deg=state.pitch_deg,heading_deg=(angle+90+calibration.rotation)%360),
        flight=dict(mode=state.mode,armed=state.armed,phase=state.phase,
                    relative_altitude_m=state.relative_altitude_m,ground_speed_mps=state.ground_speed_mps,climb_rate_mps=state.climb_rate_mps),
        battery=dict(voltage_v=state.voltage_v,current_a=state.current_a,percent=state.battery_percent),
        sensors=dict(imu=True,barometer=True,compass=True,gnss=False))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host',default='127.0.0.1')
    p.add_argument('--port',type=int,default=14600)
    p.add_argument('--hz',type=float,default=10.)
    p.add_argument('--duration',type=float,default=30.)
    p.add_argument('--vehicle',default='uav1')
    p.add_argument('--key-file',type=Path)
    args=p.parse_args()
    number(args.hz,1,30);number(args.duration,.1,3600)
    if not 1<=args.port<=65535:p.error('Invalid port')
    key=args.key_file.read_bytes().strip() if args.key_file else None
    if key is not None and len(key)<32:p.error('Key requires >=32 bytes')
    data=ROOT/'pynq_z2_desktop/data'
    route=json.loads((data/'trajectory.json').read_text())
    path=Polyline(route['points'],route['duration_seconds'])
    calibration=MapCalibration(json.loads((data/'telemetry_map.json').read_text()))
    start=deadline=time.monotonic()
    session=secrets.token_hex(8)
    seq=0
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
        while time.monotonic()-start<args.duration:
            packet=sample_packet(time.monotonic()-start,seq,session,args.vehicle,path,calibration)
            sock.sendto(encode_packet(packet,key),(args.host,args.port))
            seq+=1
            deadline+=1/args.hz
            time.sleep(max(0,deadline-time.monotonic()))
    print(f'Sent {seq} SYNTHETIC test packets; no vehicle commands')


if __name__=='__main__':main()
