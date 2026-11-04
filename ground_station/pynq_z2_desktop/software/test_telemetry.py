"""Protocol, loopback transport, shared pages and outage tests; no board needed."""
import copy
import json
from pathlib import Path
import socket
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np

from flight_status import FlightRenderer
from telemetry import TelemetryReader, TelemetryStore, atomic_write, decode_packet, encode_packet, flight_state
from telemetry_receiver import receive_once
from telemetry_view import LiveFlightView, LiveMapView, MapCalibration

DATA=Path(__file__).resolve().parents[1]/'data'


def packet(seq=0, session='session1'):
    return dict(version=1,data_origin='test',vehicle_id='uav1',session_id=session,seq=seq,
        position=dict(east_m=1.,north_m=2.,valid=True),
        attitude=dict(roll_deg=1.,pitch_deg=-.5,heading_deg=359.),
        flight=dict(mode='HOLD',armed=True,phase='hover',relative_altitude_m=1.05,
                    ground_speed_mps=.22,climb_rate_mps=0.),
        battery=dict(voltage_v=16.8,current_a=2.9,percent=86.),
        sensors=dict(imu=True,barometer=True,compass=False,gnss=False))


class ProtocolTests(unittest.TestCase):
    def test_schema_numeric_limits_and_identity(self):
        original=packet()
        self.assertEqual(decode_packet(encode_packet(original)),original)
        for field,value in [('version',True),('seq',True),('seq',-1),('vehicle_id','bad/id'),('data_origin','unknown')]:
            value_packet=copy.deepcopy(original);value_packet[field]=value
            with self.assertRaises(ValueError):decode_packet(encode_packet(value_packet))
        for group,field,value in [('position','east_m',float('nan')),
                ('attitude','roll_deg',181),('attitude','heading_deg',360),
                ('flight','armed',1),('battery','percent',101),('sensors','imu','yes')]:
            value_packet=copy.deepcopy(original);value_packet[group][field]=value
            with self.assertRaises(ValueError):decode_packet(json.dumps(value_packet).encode())
        for raw in [b'{}',b'[]',b'\xff',b'x'*4097,b'{"version":1,"version":1}']:
            with self.assertRaises((ValueError,UnicodeError)):decode_packet(raw)

    def test_hmac_tamper_and_missing_signature(self):
        key=b'test-only-shared-key-for-32-bytes-minimum'
        raw=encode_packet(packet(),key)
        self.assertEqual(decode_packet(raw,key),packet())
        tampered=json.loads(raw);tampered['battery']['voltage_v']=12
        for bad in [encode_packet(packet()),json.dumps(tampered).encode(),raw]:
            with self.assertRaises(ValueError):decode_packet(bad,key if bad!=raw else b'wrong-key')

    def test_ordering_timeout_and_restart(self):
        store=TelemetryStore(timeout=2,restart_grace=3)
        self.assertTrue(store.accept(packet(3),100))
        self.assertFalse(store.accept(packet(3),101))
        self.assertFalse(store.accept(packet(2),101))
        self.assertTrue(store.snapshot(101.9)['connected'])
        self.assertFalse(store.snapshot(102.1)['connected'])
        self.assertFalse(store.accept(packet(0,'newsession'),102.5))
        self.assertEqual(store.received,100)
        self.assertTrue(store.accept(packet(0,'newsession'),103.1))
        self.assertEqual(len(store.trail),1)
        other=packet(1,'newsession');other['vehicle_id']='uav2'
        self.assertFalse(store.accept(other,103.2))

    def test_trail_bounded_gaps_and_invalid_position(self):
        store=TelemetryStore(trail_limit=3)
        for seq in range(10):
            p=packet(seq);p['position']['east_m']=seq
            self.assertTrue(store.accept(p,seq*.1))
        self.assertEqual(len(store.trail),3)
        p=packet(10);p['position']['valid']=False
        store.accept(p,1.)
        self.assertIsNone(store.trail[-1])
        store.accept(packet(12),5.)
        self.assertEqual(store.missing,1)
        self.assertIsNone(store.trail[-2])

    def test_udp_loopback_auth_peer_filter_and_bad_packet(self):
        store=TelemetryStore()
        key=b'loopback-tests-only-shared-key-32bytes'
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as receiver, socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
            receiver.bind(('127.0.0.1',0));receiver.settimeout(1)
            sender.sendto(encode_packet(packet(),key),receiver.getsockname())
            self.assertTrue(receive_once(receiver,store,'127.0.0.1',key,100))
            sender.sendto(encode_packet(packet(1),key),receiver.getsockname())
            self.assertFalse(receive_once(receiver,store,'127.0.0.2',key,101))
            sender.sendto(b'not-json',receiver.getsockname())
            self.assertFalse(receive_once(receiver,store,'127.0.0.1',key,102))
            self.assertEqual(store.received,100)
            self.assertEqual((store.accepted,store.rejected),(1,2))


class PageTests(unittest.TestCase):
    def test_calibration_axes_rotation_and_validation(self):
        c=MapCalibration(dict(origin_px=[400,500],pixels_per_meter=50,rotation_deg=0))
        np.testing.assert_array_equal(c.point(1,2),[450,400])
        self.assertEqual(c.heading(0),-90)
        for rotation in [0,90,-90,45]:
            c=MapCalibration(dict(origin_px=[400,500],pixels_per_meter=50,rotation_deg=rotation))
            np.testing.assert_allclose(c.inverse(c.point(1,2)),[1,2],atol=1e-12)
        with self.assertRaises(ValueError):MapCalibration(dict(origin_px=[400,500],pixels_per_meter=0,rotation_deg=0))

    def test_file_reader_freshness_receiver_exit_and_corruption(self):
        with tempfile.TemporaryDirectory() as folder:
            file=Path(folder)/'state.json';reader=TelemetryReader(file)
            self.assertFalse(reader.read(99)[1])
            store=TelemetryStore();store.accept(packet(),100)
            atomic_write(file,store.snapshot(100))
            self.assertTrue(reader.read(100)[1])
            # Receiver publishing can stop; readers still expire independently.
            self.assertFalse(reader.read(103)[1])
            self.assertFalse(TelemetryReader(file).read(1)[1])
            file.write_text('{bad')
            self.assertIsNone(reader.read(104)[0])

    def test_shared_snapshot_drives_map_and_status_not_demo(self):
        with tempfile.TemporaryDirectory() as folder:
            file=Path(folder)/'state.json';store=TelemetryStore()
            store.accept(packet(),100);atomic_write(file,store.snapshot(100))
            reader=TelemetryReader(file)
            flight=LiveFlightView(reader)
            state=flight.sample(100)
            self.assertEqual(state.source,'udp_test')
            self.assertEqual((state.heading_deg,state.current_a,state.local_position),(359.,2.9,'valid'))
            frame=np.empty((600,800,4),np.uint8)
            renderer=FlightRenderer(DATA/'flight_status')
            renderer.draw(frame,state,100);flight.overlay(frame)
            self.assertTrue(flight.connected)
            c=MapCalibration(dict(origin_px=[400,500],pixels_per_meter=50,rotation_deg=0))
            view=LiveMapView(np.zeros_like(frame),reader,c)
            with patch('telemetry_view.draw_quadcopter') as draw:
                self.assertTrue(view.draw(frame,100))
                self.assertEqual(draw.call_count,1)
                np.testing.assert_allclose(draw.call_args.args[1:3],[450,400])
                self.assertFalse(view.draw(frame,103))
                self.assertEqual(draw.call_count,1)  # stale plane disappears
            flight.sample(103);flight.overlay(frame)
            self.assertFalse(flight.connected)

    def test_invalid_position_does_not_draw_aircraft(self):
        with tempfile.TemporaryDirectory() as folder:
            file=Path(folder)/'state.json';store=TelemetryStore();p=packet();p['position']['valid']=False
            store.accept(p,100);atomic_write(file,store.snapshot(100))
            frame=np.zeros((600,800,4),np.uint8)
            view=LiveMapView(frame,TelemetryReader(file),MapCalibration(dict(origin_px=[400,500],pixels_per_meter=50,rotation_deg=0)))
            with patch('telemetry_view.draw_quadcopter') as draw:
                self.assertTrue(view.draw(frame,100))
                draw.assert_not_called()


if __name__=='__main__':unittest.main()
