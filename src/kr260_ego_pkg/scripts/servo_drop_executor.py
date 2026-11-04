#!/usr/bin/env python3
"""Bounded SRV1 actuator adapter. No arming, mode or navigation publishers."""
import json
from pathlib import Path
import subprocess
import sys
import time
from rescue_protocol import Journal, authority, session


def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from kr260_ego_interfaces.msg import DropRequest, DropFeedback

    class Executor(Node):
        def __init__(self):
            super().__init__('kr260_servo_drop_executor')
            self.directory = Path(self.declare_parameter('servo_dir', '').value).resolve()
            for name in ('servo_init.py', 'servo_move.py', 'servo_config.json', 'td8120mg.json'):
                if not (self.directory/name).is_file():
                    raise ValueError('servo_dir must point to hardware/servo/deploy')
            self.channel = self.declare_parameter('channel', 0).value
            if type(self.channel) is not int or self.channel not in (0, 1):
                raise ValueError('Select servo channel 0 or 1')
            self.journal = Journal(self.declare_parameter('journal_dir', str(Path.home()/'.local/state/kr260/drop')).value)
            self.status, self.pending, self.active = {}, None, None
            self.process, self.ready, self.next_probe = None, False, 0.
            self.result, self.disabled, self.detail = DropFeedback.READY, True, 'Waiting for SRV1'
            self.run, self.ident, self.deadline, self.cancelled = '', 0, 0., False
            self.cancelled_at = 0.
            self.pub = self.create_publisher(DropFeedback, '/kr260/rescue/drop_feedback', 10)
            self.create_subscription(String, '/kr260/ego/status', self.status_received, 1)
            self.create_subscription(DropRequest, '/kr260/rescue/drop_request', self.request, 10)
            self.create_timer(.1, self.tick)

        def status_received(self, msg):
            try:
                self.status = json.loads(msg.data)
            except (ValueError, TypeError):
                self.status = {}

        def command(self, script, *args):
            return [sys.executable, str(self.directory/script), *args]

        def probe(self):
            try:
                result = subprocess.run(self.command('servo_init.py', 'status'), capture_output=True, text=True, timeout=.5)
                status = json.loads(result.stdout)
                return result.returncode == 0 and status.get('state') == 'DISABLED'
            except (OSError, ValueError, subprocess.TimeoutExpired):
                return False

        def request(self, msg):
            if msg.cancel:
                if self.pending and msg.run_id == self.pending.run_id and msg.request_id == self.pending.request_id:
                    self.pending = None
                if (msg.run_id, msg.request_id) == (self.run, self.ident):
                    if self.process:
                        self.cancel('Controller cancelled request')
                    elif self.result == DropFeedback.COMPLETE:
                        self.ident = 0  # Completion acknowledged by controller.
                return
            if self.process or self.pending or self.ident or not self.ready:
                return
            age = (self.get_clock().now().nanoseconds - (msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
            if (not session(msg.run_id) or msg.request_id < 1 or not 0 <= age < msg.valid_for_s <= 1. or
                    not 500 <= msg.pulse_us <= 1700 or not .2 <= msg.duration_s <= 3.):
                return
            self.pending = msg
            self.deadline = time.monotonic()+msg.valid_for_s-age

        def cancel(self, reason):
            if not self.cancelled:
                self.cancelled_at = time.monotonic()
            self.cancelled = True
            self.detail = reason
            if self.process and self.process.poll() is None:
                self.process.terminate()  # servo_move SIGTERM handler runs its finally/disable.

        def tick(self):
            now = time.monotonic()
            if self.process:
                if not authority(self.status, self.run, self.ident, now) or now > self.deadline:
                    if not self.cancelled:
                        self.cancel('Authority stale/lost or bounded action deadline exceeded')
                if self.cancelled and self.process.poll() is None and now-self.cancelled_at > 1.:
                    self.process.kill()
                    self.process.wait(timeout=.5)
                    try:
                        subprocess.run(self.command('servo_init.py', 'disable'), timeout=.5, capture_output=True)
                    except (OSError, subprocess.TimeoutExpired):
                        self.detail += '; disable readback unavailable'
                if self.process.poll() is not None:
                    out, err = self.process.communicate()
                    ok = False
                    try:
                        value = json.loads(out)
                        ok = (not self.cancelled and self.process.returncode == 0 and value.get('state') == 'DISABLED' and
                              value.get('channel') == self.channel and value.get('commanded_pulse_us') == self.active.pulse_us and
                              abs(value.get('duration_s', -1)-self.active.duration_s) < .001 and value.get('movement_command_sent') is True)
                    except (ValueError, TypeError):
                        pass
                    self.disabled = self.probe()
                    self.result = DropFeedback.COMPLETE if ok and self.disabled else DropFeedback.FAILED
                    self.detail = 'Command finished; PWM off' if self.result == DropFeedback.COMPLETE else (self.detail or err[-500:])
                    self.process = None
            elif not self.ident and now >= self.next_probe:
                self.ready = self.disabled = self.probe()
                self.next_probe = now+1.
            if self.pending:
                req = self.pending
                if now > self.deadline:
                    self.pending = None
                elif authority(self.status, req.run_id, req.request_id, now):
                    self.pending = None
                    self.run, self.ident = req.run_id, req.request_id
                    self.active, self.cancelled = req, False
                    self.ready = False
                    try:
                        if not self.journal.claim(self.run, self.ident):
                            raise ValueError('Request already claimed; restart replay rejected')
                        self.process = subprocess.Popen(self.command('servo_move.py', '--channel', str(self.channel),
                            '--pulse-us', str(req.pulse_us), '--seconds', str(req.duration_s)),
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                        self.result, self.disabled, self.detail = DropFeedback.RUNNING, False, 'Bounded pulse in progress'
                        self.deadline = now+req.duration_s+1.
                    except (OSError, ValueError) as exc:
                        self.result, self.detail = DropFeedback.FAILED, str(exc)
                        self.disabled = self.probe()
            feedback = DropFeedback()
            feedback.header.stamp = self.get_clock().now().to_msg()
            feedback.run_id = self.run if self.ident else self.status.get('run_id', '')
            feedback.request_id = self.ident
            feedback.state = self.result if self.ident else (DropFeedback.READY if self.ready else DropFeedback.FAILED)
            feedback.outputs_disabled = self.disabled
            feedback.detail = self.detail
            if session(feedback.run_id):
                self.pub.publish(feedback)

        def close(self):
            if self.process and self.process.poll() is None:
                self.cancel('Executor shutting down')
                try:
                    self.process.communicate(timeout=2.)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.communicate(timeout=1.)
                    # Only this executor's still-active command is being recovered.
                    subprocess.run(self.command('servo_init.py', 'disable'), timeout=1., capture_output=True)

    rclpy.init()
    node = Executor()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
