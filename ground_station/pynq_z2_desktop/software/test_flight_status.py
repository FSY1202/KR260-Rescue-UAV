"""Boundary and semantic tests for the standalone demo; no board access."""
import math
from pathlib import Path
import unittest
import numpy as np

from button_input import ButtonPress
from flight_demo import DemoClock, DemoFlight
from flight_status import FlightRenderer, compass_left, horizon_matrix
from flight_native import HorizonBlitter
import cv2


class DemoTests(unittest.TestCase):
    def test_stage_and_arming_boundaries(self):
        demo = DemoFlight()
        for t, phase, mode, armed in [(0,'standby','POSCTL',False),
                (5,'takeoff','TAKEOFF',True),(10,'cruise','OFFBOARD',True),
                (45,'hover','HOLD',True),(50,'landing','LAND',True),
                (55,'standby','POSCTL',False),(60,'standby','POSCTL',False)]:
            s = demo.sample(t)
            self.assertEqual((s.phase,s.mode,s.armed),(phase,mode,armed))
            self.assertEqual(s.source,'preset_demo')
            self.assertEqual(s.gnss,'not_connected')
        for t in (5,10,45,50,55,60):
            before,after = demo.sample(t-1e-5),demo.sample(t+1e-5)
            self.assertAlmostEqual(before.relative_altitude_m,after.relative_altitude_m,places=4)
            self.assertAlmostEqual(before.ground_speed_mps,after.ground_speed_mps,places=4)

    def test_limits_and_derivative(self):
        demo = DemoFlight()
        for t in np.linspace(0,120,1201):
            s = demo.sample(float(t))
            for value in s.to_dict().values():
                if isinstance(value,float):self.assertTrue(math.isfinite(value))
            self.assertTrue(1. <= s.relative_altitude_m <= 1.1)
            self.assertTrue(.20 <= s.ground_speed_mps <= .25)
            self.assertEqual(s.voltage_v,16.8)
            self.assertTrue(2.8 <= s.current_a <= 3.)
            self.assertLessEqual(abs(s.roll_deg),1.)
            self.assertLessEqual(abs(s.pitch_deg),.6)
            self.assertTrue(0<=s.heading_deg<360)
            self.assertTrue(82<=s.battery_percent<=88)
        for t in (7.5,18.,23.,35.,52.5):
            eps=1e-4
            delta=(demo.sample(t+eps).relative_altitude_m-demo.sample(t-eps).relative_altitude_m)/(2*eps)
            self.assertAlmostEqual(delta,demo.sample(t).climb_rate_mps,places=6)

    def test_heading_wrap_and_independent_clock(self):
        demo=DemoFlight()
        headings=np.array([demo.sample(t).heading_deg for t in np.linspace(10,45,351)])
        delta=(np.diff(headings)+180)%360-180
        self.assertTrue((delta>=0).all())
        self.assertLess(delta.max(),2)
        self.assertTrue(any(headings<10))
        self.assertEqual(demo.sample(23),demo.sample(83))
        clock=DemoClock()
        self.assertEqual(clock.elapsed(90),0)
        clock.start(100)
        self.assertEqual(clock.elapsed(112),12)
        clock.start(150)  # a later reopen does not reset the cycle
        self.assertEqual(clock.elapsed(160),60)

    def test_invalid_inputs_and_config(self):
        for config in [{'battery_cells':6.5},{'battery_cells':True},{'maximum_altitude_m':float('nan')},
                       {'battery_end_percent':99},{'maximum_roll_deg':90},{'unknown':1}]:
            with self.assertRaises(ValueError):DemoFlight(config)
        for t in (-1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):DemoFlight().sample(t)

    def test_horizon_signs_and_compass_range(self):
        m=horizon_matrix(10,0)
        line=np.array([[320,320,1],[420,320,1]])@m.T
        self.assertLess(line[1,1],line[0,1])
        self.assertGreater((horizon_matrix(0,7)@np.array([320,320,1]))[1],140)
        self.assertLess((horizon_matrix(0,-7)@np.array([320,320,1]))[1],140)
        for h in (-.1,0,.1,359.9,360):
            self.assertTrue(0<=compass_left(h)<=3240-412)

    def test_btn_debounce_long_hold(self):
        button=ButtonPress(0,0)
        events=[]
        for now,level in [(0,0),(.01,1),(.02,0),(.03,1),(.08,1),(1.,1),
                          (2.,1),(2.01,0),(2.06,0),(2.1,1),(2.15,1)]:
            if button.update(level,now):events.append(now)
        self.assertEqual(events,[.08,2.15])

    def test_raster_assets_and_all_phases(self):
        renderer=FlightRenderer(Path(__file__).resolve().parents[1]/'data/flight_status')
        frame=np.zeros((600,800,4),np.uint8)
        for t in (0,7.5,23,47,52.5,58,60):
            renderer.draw(frame,DemoFlight().sample(t),t)
            self.assertEqual(frame.shape,(600,800,4))
            self.assertTrue((frame[:,:,3]==255).all())
            self.assertGreater(np.unique(frame[120:400,28:440,:3].reshape(-1,3),axis=0).shape[0],10)

    def test_native_nearest_strided_surface(self):
        blitter=HorizonBlitter()
        if blitter.backend != 'native_fixed_point':
            self.skipTest('Native ARM raster not available on this host')
        texture=np.empty((640,640,4),np.uint8)
        yy,xx=np.indices((640,640))
        texture[:,:,0]=xx%256;texture[:,:,1]=yy%256
        texture[:,:,2]=(xx+yy)%256;texture[:,:,3]=255
        for roll,pitch in [(0,0),(12,7),(-12,-7),(7.3,-5.9)]:
            frame=np.full((600,800,4),171,np.uint8)
            roi=frame[120:400,28:440]
            matrix=horizon_matrix(roll,pitch)
            blitter.draw(roi,texture,matrix)
            reference=cv2.warpAffine(texture,matrix,(412,280),flags=cv2.INTER_NEAREST,
                                     borderMode=cv2.BORDER_REPLICATE)
            differing=np.any(roi!=reference,axis=2)
            self.assertLess(differing.mean(),.005)
            self.assertTrue((frame[:120]==171).all())
            self.assertTrue((frame[400:]==171).all())
            self.assertTrue((frame[120:400,:28]==171).all())
            self.assertTrue((frame[120:400,440:]==171).all())


if __name__=='__main__':unittest.main()
