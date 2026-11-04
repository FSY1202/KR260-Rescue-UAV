"""CPU-side regressions with emulated MMIO. No camera or privileges needed."""
import sys
import types
from pathlib import Path
import unittest
import numpy as np

try:
    import fcntl
except ImportError:  # Windows host can test the MMIO-independent capture logic.
    sys.modules['fcntl'] = types.ModuleType('fcntl')
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'deploy'))
from frame_grabber import FrameGrabber


class FakeStream(FrameGrabber):
    def __init__(self):
        self.registers = {12: 9, 0x10: 17, 0x20: 640*480, 0x38: 1,
                          0x40: 100000, 0x44: 100000000, 0x48: 0,
                          0x18: 0, 0x1c: 0, 0x30: 18, 0x34: 3}
        self.writes = []
        self.base = 0
        self.mutate_during_copy = False
        self.lib = types.SimpleNamespace(camera_mmio_copy32=self.copy)

    def read(self, offset):
        return self.registers[offset]

    def write(self, offset, value):
        self.writes.append((offset, value))
        if value == 8:
            self.registers[12] = 3
        elif value == 2:
            self.registers[12] = 1

    def copy(self, base, offset, output, words):
        import ctypes
        view = np.ctypeslib.as_array((ctypes.c_uint32 * words).from_address(output))
        view[:] = 0xf80007e0
        if self.mutate_during_copy:
            self.registers[0x38] = 2

    def close(self, abort=True):
        pass


class StreamGrabberTests(unittest.TestCase):
    def test_complete_frozen_frame_and_release(self):
        g = FakeStream()
        pixels, info = g.capture()
        self.assertEqual(pixels.shape, (480, 640))
        self.assertEqual(list(pixels[0, :2]), [0x07e0, 0xf800])
        self.assertEqual(info['frame_seq'], 17)
        self.assertEqual(info['buffer_bank'], 1)
        self.assertEqual(info['claimed_frame_age_ms'], 1)
        self.assertEqual(g.writes, [(8, 8), (8, 2)])
        self.assertLess(info['capture_completed_monotonic_ns'], info['copy_completed_monotonic_ns'])

    def test_changed_bank_during_copy_is_rejected(self):
        g = FakeStream()
        g.mutate_during_copy = True
        with self.assertRaisesRegex(RuntimeError, 'ownership changed'):
            g.capture()
        self.assertEqual(g.writes[-1], (8, 2))
        self.assertNotIn((8, 4), g.writes)

    def test_stale_frame_rejected_and_released(self):
        g = FakeStream()
        g.registers[0x40] = 25000001
        with self.assertRaisesRegex(RuntimeError, '250 ms'):
            g.capture()
        self.assertEqual(g.writes[-1], (8, 2))

    def test_counter_wrap_cannot_make_old_frame_fresh(self):
        g = FakeStream()
        g.registers[0x48] = 1
        with self.assertRaisesRegex(RuntimeError, '250 ms'):
            g.capture()

    def test_bad_geometry_rejected(self):
        g = FakeStream()
        g.registers[0x20] -= 1
        with self.assertRaisesRegex(RuntimeError, 'Incomplete pixel count'):
            g.capture()


if __name__ == '__main__':
    unittest.main()
