import importlib.util
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "deploy"))
import servo_init as servo
import servo_move as move


class FakeClock:
    def __init__(self):
        self.value = 0.0
    def now(self):
        return self.value
    def sleep(self, delay):
        self.value += delay


class MMIO:
    def __init__(self):
        self.values = {servo.REG_ID: servo.HW_ID, servo.REG_VERSION: servo.VERSION,
                       servo.REG_ENABLE: 0, servo.REG_STATUS: 0}
        self.writes = []
        self.external_stop = False
        self.clock = None
    def read(self, offset):
        if self.external_stop and self.clock.value >= 0.08:
            self.values[servo.REG_ENABLE] = self.values[servo.REG_STATUS] = 0
        return self.values.get(offset, 0)
    def write(self, offset, value):
        self.writes.append((offset, value))
        self.values[offset] = value
        if offset == servo.REG_ENABLE:
            self.values[servo.REG_STATUS] = value << 8
        if offset == servo.REG_COMMIT:
            for target, source in [(servo.REG_ACTIVE_PERIOD, servo.REG_PERIOD),
                                   (servo.REG_ACTIVE_PULSE0, servo.REG_PULSE0),
                                   (servo.REG_ACTIVE_PULSE1, servo.REG_PULSE1)]:
                self.values[target] = self.values[source]


class ServoMoveTests(unittest.TestCase):
    def setUp(self):
        self.profile = json.loads((ROOT / "deploy/servo_config.json").read_text())
        self.model = json.loads((ROOT / "deploy/td8120mg.json").read_text())
        self.clock = FakeClock()
        self.io = MMIO()
        self.io.clock = self.clock
    def run_command(self, channel=0, pulse=1000, seconds=1):
        return move.run_pulse(self.io, channel, pulse, seconds, self.profile, self.model,
                              self.clock.now, self.clock.sleep)
    def test_both_channels_selectable_one_at_a_time_and_bounded(self):
        for channel in (0, 1):
            self.setUp()
            result = self.run_command(channel, 900, 0.5)
            enables = [v for a, v in self.io.writes if a == servo.REG_ENABLE]
            self.assertEqual([v for v in enables if v], [1 << channel])
            self.assertEqual(enables[-1], 0)
            self.assertEqual(self.io.values[servo.REG_STATUS], 0)
            self.assertAlmostEqual(self.clock.value, 0.5)
            self.assertFalse(result["physical_motion_verified"])
    def test_bad_requests_write_nothing(self):
        for channel, pulse, duration in [(2,1000,1),(True,1000,1),(0,499,1),
                                         (0,1701,1),(0,2500,1),(0,1000,4),(0,1000,float('nan'))]:
            with self.subTest(args=(channel,pulse,duration)):
                with self.assertRaises(servo.ServoError):
                    self.run_command(channel,pulse,duration)
                self.assertEqual(self.io.writes, [])
    def test_does_not_take_over_existing_output(self):
        self.io.values[servo.REG_ENABLE] = 2
        with self.assertRaises(servo.ServoError):
            self.run_command()
        self.assertEqual(self.io.writes, [])
    def test_external_disable_cancels_without_reenable(self):
        self.io.external_stop = True
        with self.assertRaises(move.StopRequested):
            self.run_command()
        enables = [v for a, v in self.io.writes if a == servo.REG_ENABLE]
        self.assertEqual([v for v in enables if v], [1])
        self.assertEqual(enables[-1], 0)
        self.assertLess(self.clock.value, 0.2)
    def test_interrupt_always_disables(self):
        def interrupt(_delay):
            raise move.StopRequested("test Ctrl-C")
        with self.assertRaises(move.StopRequested):
            move.run_pulse(self.io,0,1000,1,self.profile,self.model,self.clock.now,interrupt)
        self.assertEqual(self.io.writes[-1], (servo.REG_ENABLE,0))

    def test_requested_range_endpoints_and_900_are_accepted(self):
        for value in (500, 900, 1300, 1500, 1501, 1700):
            with self.subTest(pulse_us=value):
                self.setUp()
                result = self.run_command(pulse=value, seconds=0.2)
                self.assertEqual(result["commanded_pulse_us"], value)
                self.assertEqual(self.io.writes[-1], (servo.REG_ENABLE, 0))

    def test_1700_is_valid_and_1701_is_rejected_without_writes(self):
        result = self.run_command(pulse=1700, seconds=0.2)
        self.assertEqual(result["commanded_pulse_us"], 1700)
        self.assertIn((servo.REG_PULSE0, 1700), self.io.writes)
        self.assertEqual(self.io.writes[-1], (servo.REG_ENABLE, 0))
        self.io = MMIO()
        self.io.clock = self.clock
        with self.assertRaises(servo.ServoError):
            self.run_command(pulse=1701, seconds=0.2)
        self.assertEqual(self.io.writes, [])

    def test_invalid_profile_range_writes_nothing(self):
        self.model["initial_test_min_pulse_us"] = 499
        with self.assertRaises(servo.ServoError):
            self.run_command()
        self.assertEqual(self.io.writes, [])


if __name__ == "__main__":
    unittest.main()
