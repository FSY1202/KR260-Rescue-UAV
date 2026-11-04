"""Exercise native access on a temporary regular file, never on a device."""
import importlib.util
from pathlib import Path
import struct
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("servo_native_subject", ROOT / "deploy/servo_init.py")
servo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(servo)


@unittest.skipUnless(sys.platform.startswith("linux") and (ROOT / "deploy/libservo_mmio.so").is_file(),
                     "Build the Linux native MMIO helper first")
class NativeMMIOTests(unittest.TestCase):
    def test_single_word_access_preserves_neighbors_and_closes(self):
        with tempfile.TemporaryDirectory() as name:
            path = Path(name) / "regular_file_not_a_device"
            path.write_bytes(b"\xa5" * 4096)
            for _ in range(5):
                with servo.AtomicMMIO(path, 4096) as io:
                    io.write(8, 0x12345678)
                    self.assertEqual(io.read(8), 0x12345678)
                    self.assertEqual(io.read(4), 0xA5A5A5A5)
                    self.assertEqual(io.read(12), 0xA5A5A5A5)
            data = path.read_bytes()
            self.assertEqual(data[8:12], struct.pack("<I", 0x12345678))
            self.assertEqual(data[:8] + data[12:], b"\xa5" * 4092)

    def test_bad_offsets_are_rejected_before_device_access(self):
        with tempfile.TemporaryDirectory() as name:
            path = Path(name) / "regular_file_not_a_device"
            path.write_bytes(b"\x5a" * 4096)
            with servo.AtomicMMIO(path, 4096) as io:
                for offset in [-4, 1, 4096, 4094, True]:
                    with self.subTest(offset=offset):
                        with self.assertRaises(servo.ServoError):
                            io.write(offset, 0)
            self.assertEqual(path.read_bytes(), b"\x5a" * 4096)

    def test_one_way_disable_can_open_while_controller_owns_lock(self):
        with tempfile.TemporaryDirectory() as name:
            path = Path(name) / "regular_file_not_a_device"
            path.write_bytes(b"\x00" * 4096)
            with servo.AtomicMMIO(path, 4096) as controller:
                controller.write(8, 1)
                with self.assertRaises(BlockingIOError):
                    servo.AtomicMMIO(path, 4096)
                with servo.AtomicMMIO(path, 4096, lock=False) as emergency:
                    emergency.write(8, 0)
                self.assertEqual(controller.read(8), 0)


if __name__ == "__main__":
    unittest.main()
