"""Drive the ROS adapter with fake messages/processes; no ROS or UIO required."""
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/kr260_ego_pkg/scripts'))
import servo_drop_executor as adapter


class Message:
    READY,RUNNING,COMPLETE,FAILED=0,1,2,3
    def __init__(self):
        self.header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=100,nanosec=0))


class Executor(unittest.TestCase):
    def simulate(self, scenario):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            for name in ('servo_init.py','servo_move.py','servo_config.json','td8120mg.json'):
                (root/name).touch()
            clock=[100.]
            class Node:
                def __init__(self,*_):self.sent=[]
                def declare_parameter(self,name,value):
                    return types.SimpleNamespace(value={'servo_dir':str(root),'journal_dir':str(root/'journal')}.get(name,value))
                def create_publisher(self,*_):return types.SimpleNamespace(publish=lambda m:self.sent.append(m))
                def create_subscription(self,*_):pass
                def create_timer(self,*_):pass
                def get_clock(self):
                    return types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=int(clock[0]*1e9),
                        to_msg=lambda:types.SimpleNamespace(sec=int(clock[0]),nanosec=0)))
                def destroy_node(self):pass
            ros=types.ModuleType('rclpy');ros.init=lambda:None;ros.shutdown=lambda:None
            ros.spin=lambda node:scenario(node,clock,process)
            node_module=types.ModuleType('rclpy.node');node_module.Node=Node
            std=types.ModuleType('std_msgs.msg');std.String=Message
            messages=types.ModuleType('kr260_ego_interfaces.msg');messages.DropRequest=messages.DropFeedback=Message
            process=Mock();process.poll.return_value=None
            probe=types.SimpleNamespace(returncode=0,stdout=json.dumps({'state':'DISABLED'}))
            with patch.dict(sys.modules,{'rclpy':ros,'rclpy.node':node_module,'std_msgs.msg':std,
                                         'kr260_ego_interfaces.msg':messages}), \
                 patch.object(adapter.time,'monotonic',side_effect=lambda:clock[0]), \
                 patch.object(adapter.subprocess,'run',return_value=probe), \
                 patch.object(adapter.subprocess,'Popen',return_value=process) as popen:
                adapter.main()
                return popen.call_count

    @staticmethod
    def status(node, state='RESCUE_DROP', mode=14):
        value=dict(run_id='a'*32,updated_monotonic=100.,actual_velocity_ned=[0,0,0],execute=True,
                   triggered=True,armed=True,landed=False,nav_state=mode,state=state,drop_request_id=1,
                   healthy=True,planning_healthy=True)
        node.status_received(types.SimpleNamespace(data=json.dumps(value)))

    @staticmethod
    def request():
        m=Message();m.run_id='a'*32;m.request_id=1;m.cancel=False
        m.valid_for_s=1.;m.pulse_us=1000;m.duration_s=1.
        return m

    def test_wait_for_matching_status_complete_and_no_replay(self):
        def scenario(node,clock,process):
            self.status(node,'RESCUE_HOLD');node.tick();self.assertTrue(node.ready)
            node.request(self.request());node.tick();self.assertIsNone(node.process)
            self.status(node);node.tick();self.assertIsNotNone(node.process)
            process.poll.return_value=0;process.returncode=0
            process.communicate.return_value=(json.dumps(dict(state='DISABLED',channel=0,
                commanded_pulse_us=1000,duration_s=1.,movement_command_sent=True)),'')
            node.tick();self.assertEqual(node.result,Message.COMPLETE);self.assertTrue(node.disabled)
            cancel=self.request();cancel.cancel=True;node.request(cancel)
            clock[0]+=1.1;node.tick();clock[0]=100.
            node.request(self.request());node.tick()
            self.assertEqual(node.result,Message.FAILED)
            self.assertIn('replay',node.detail)
        self.assertEqual(self.simulate(scenario),1)

    def test_no_actuation_in_preview(self):
        def scenario(node,clock,process):
            self.status(node,'PREVIEW');node.tick();node.request(self.request());node.tick()
            clock[0]+=1.1;node.tick();self.assertIsNone(node.pending)
            self.assertIsNone(node.process)
        self.assertEqual(self.simulate(scenario),0)

    def test_position_takeover_terminates_and_fails(self):
        def scenario(node,clock,process):
            self.status(node);node.tick();node.request(self.request());node.tick()
            self.status(node,mode=2);node.tick();process.terminate.assert_called_once()
            process.poll.return_value=-15;process.returncode=-15;process.communicate.return_value=('','cancelled')
            node.tick();self.assertEqual(node.result,Message.FAILED)
            self.assertTrue(node.disabled)
        self.assertEqual(self.simulate(scenario),1)


if __name__=='__main__':unittest.main()
