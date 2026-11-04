import copy
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/kr260_ego_pkg/scripts'))
from rescue_protocol import Journal, authority, observation


class Protocol(unittest.TestCase):
    def test_frames_preserve_source(self):
        status=dict(state='READY', detection_session_id='a'*32, source_camera_session_id='b'*32,
                    result_seq=1, source_frame_seq=0, source_frame_monotonic_ns=10**9,
                    detections=[dict(class_id=0,confidence=.8,xyxy=[1,2,10,20])])
        out=observation(status,2*10**9)
        self.assertTrue(out['person_detected'])
        self.assertEqual(out['source_monotonic_ns'],10**9)
        for field,value in [('state','ERROR'),('source_frame_monotonic_ns',3*10**9),
                            ('source_frame_monotonic_ns',1),('source_frame_monotonic_ns',True),
                            ('source_camera_session_id','bad'),('source_metadata_error','missing')]:
            changed=copy.deepcopy(status);changed[field]=value
            with self.assertRaises(ValueError):observation(changed,2*10**9+2)
        status['detections']=[]
        self.assertFalse(observation(status,2*10**9)['person_detected'])

    def test_authority(self):
        s=dict(run_id='a'*32,updated_monotonic=100.,actual_velocity_ned=[0,0,0],execute=True,
               triggered=True,armed=True,landed=False,nav_state=14,state='RESCUE_DROP',drop_request_id=1,
               healthy=True,planning_healthy=True)
        self.assertTrue(authority(s,'a'*32,1,100.2))
        self.assertFalse(authority(s,'a'*32,2,100.2))
        self.assertFalse(authority(s,'b'*32,1,100.2))
        self.assertFalse(authority(s,'a'*32,1,100.81))
        for key,value in [('execute',False),('armed',False),('landed',True),('nav_state',2),
                          ('state','PREVIEW'),('healthy',False),('planning_healthy',False),
                          ('actual_velocity_ned',[float('nan'),0,0]),('actual_velocity_ned',[.2,0,0])]:
            changed=dict(s);changed[key]=value
            self.assertFalse(authority(changed,'a'*32,1,100.2))

    def test_restart_no_replay(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertTrue(Journal(tmp).claim('a'*32,1))
            self.assertFalse(Journal(tmp).claim('a'*32,1))
            self.assertTrue(Journal(tmp).claim('b'*32,1))
            with self.assertRaises(ValueError):Journal(tmp).claim('../../bad',2)


if __name__=='__main__':unittest.main()
