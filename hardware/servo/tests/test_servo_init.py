import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("servo_init", ROOT / "deploy/servo_init.py")
servo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(servo)


class FakeMMIO:
    def __init__(self):
        self.values = {servo.REG_ID: servo.HW_ID, servo.REG_VERSION: servo.VERSION,
                       servo.REG_ENABLE: 3, servo.REG_STATUS: 0x300,
                       servo.REG_ACTIVE_PERIOD: 10000,
                       servo.REG_ACTIVE_PULSE0: 1000, servo.REG_ACTIVE_PULSE1: 1000}
        self.writes = []
        self.reject_commit = False

    def read(self, offset):
        return self.values.get(offset, 0)

    def write(self, offset, value):
        self.writes.append((offset, value))
        self.values[offset] = value
        if offset == servo.REG_ENABLE:
            self.values[servo.REG_STATUS] &= ~0x300
        if offset == servo.REG_COMMIT:
            if self.reject_commit:
                self.values[servo.REG_STATUS] = 2
            else:
                for active, shadow in [(servo.REG_ACTIVE_PERIOD, servo.REG_PERIOD),
                                       (servo.REG_ACTIVE_PULSE0, servo.REG_PULSE0),
                                       (servo.REG_ACTIVE_PULSE1, servo.REG_PULSE1)]:
                    self.values[active] = self.values[shadow]


class ServoInitializationTests(unittest.TestCase):
    def setUp(self):
        self.profile = json.loads((ROOT / "deploy/servo_config.json").read_text())

    def test_init_never_enables_and_reads_back(self):
        io = FakeMMIO()
        result = servo.initialize(io, self.profile)
        self.assertTrue(result["initialized"])
        self.assertEqual(result["state"], "DISABLED")
        self.assertEqual(result["active_period_us"], 20000)
        self.assertEqual([value for reg, value in io.writes if reg == servo.REG_ENABLE], [0, 0])
        self.assertFalse(result["movement_command_sent"])

    def test_wrong_device_no_writes(self):
        io = FakeMMIO()
        io.values[servo.REG_ID] = 0x46524D32  # Camera FRM2, never repurpose it.
        with self.assertRaises(servo.ServoError):
            servo.initialize(io, self.profile)
        self.assertEqual(io.writes, [])

    def test_unsupported_version_no_writes(self):
        io = FakeMMIO()
        io.values[servo.REG_VERSION] += 1
        with self.assertRaises(servo.ServoError):
            servo.initialize(io, self.profile)
        self.assertEqual(io.writes, [])

    def test_reject_unsafe_parameters_before_writes(self):
        for field, value in [("enable_mask", 1), ("period_us", 0),
                             ("pulse0_us", 499), ("pulse1_us", 2501),
                             ("pulse0_us", True), ("supply_voltage_v", 12)]:
            with self.subTest(field=field, value=value):
                io = FakeMMIO()
                profile = dict(self.profile, **{field: value})
                with self.assertRaises(servo.ServoError):
                    servo.initialize(io, profile)
                self.assertEqual(io.writes, [])

    def test_commit_failure_remains_disabled(self):
        io = FakeMMIO()
        io.reject_commit = True
        with self.assertRaises(servo.ServoError):
            servo.initialize(io, self.profile)
        self.assertEqual(io.writes[-1], (servo.REG_ENABLE, 0))
        self.assertFalse(io.values[servo.REG_ENABLE])

    def test_missing_hardware_refuses_initialization(self):
        with patch.object(servo, "discover", return_value=None), patch.object(servo, "AtomicMMIO") as mapping:
            self.assertEqual(servo.main(["init"]), 3)
            mapping.assert_not_called()

    def test_discovery_uses_exact_address_and_not_fixed_uio_number(self):
        with tempfile.TemporaryDirectory() as name:
            base = Path(name)
            for node, address in [("uio4", "0xa0000000"), ("uio9", "0xa0030000")]:
                directory = base / node / "maps/map0"
                directory.mkdir(parents=True)
                (directory / "addr").write_text(address)
                (directory / "size").write_text("0x10000")
                (directory / "offset").write_text("0x0")
            self.assertEqual(servo.discover(base, Path("/dev")), (Path("/dev/uio9"), 65536))
            (base / "uio9/maps/map0/addr").write_text("0xa0010000")
            self.assertIsNone(servo.discover(base))

    def test_duplicate_device_is_refused(self):
        with tempfile.TemporaryDirectory() as name:
            for node in ["uio4", "uio7"]:
                directory = Path(name) / node / "maps/map0"
                directory.mkdir(parents=True)
                (directory / "addr").write_text("0xa0030000")
                (directory / "size").write_text("0x10000")
            with self.assertRaises(servo.ServoError):
                servo.discover(Path(name))


if __name__ == "__main__":
    unittest.main()
