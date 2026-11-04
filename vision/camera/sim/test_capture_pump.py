"""No camera required: bounded prefetch, ownership, errors, and shutdown."""
import sys
from pathlib import Path
import threading
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'deploy'))
from capture_pump import CapturePump


class FakeGrabber:
    def __init__(self):
        self.calls = 0
        self.entered = threading.Event()
        self.first_error = False

    def capture(self):
        self.calls += 1
        self.entered.set()
        if self.first_error and self.calls == 1:
            raise RuntimeError('invalid frame')
        return bytearray([self.calls]), {'frame_seq': self.calls}


class PumpTests(unittest.TestCase):
    def setUp(self):
        self.stop = threading.Event()
        self.fake = FakeGrabber()
        self.pump = CapturePump(self.fake, self.stop)

    def tearDown(self):
        self.stop.set()
        if self.pump.thread.ident is not None:
            self.pump.close()

    def wait_item(self):
        with self.pump.cv:
            self.assertTrue(self.pump.cv.wait_for(lambda: self.pump.item is not None, timeout=1))

    def test_bounded_prefetch_and_frame_ownership(self):
        self.pump.start()
        self.wait_item()
        with self.pump.cv:
            self.assertEqual(self.fake.calls, 1)
        first, meta = self.pump.get()
        self.assertEqual(meta['frame_seq'], 1)
        self.wait_item()  # next capture happens before consumer finishes its processing
        with self.pump.cv:
            self.assertEqual(self.fake.calls, 2)
        second, meta = self.pump.get()
        self.assertEqual(meta['frame_seq'], 2)
        self.assertEqual(first, bytearray([1]))
        self.assertIsNot(first, second)

    def test_error_propagated_then_recovery(self):
        self.fake.first_error = True
        self.pump.start()
        self.wait_item()
        with self.assertRaisesRegex(RuntimeError, 'invalid frame'):
            self.pump.get()
        value, meta = self.pump.get()
        self.assertEqual(value, bytearray([2]))
        self.assertEqual(meta['frame_seq'], 2)

    def test_stop_does_not_return_pending_stale_frame(self):
        self.pump.start()
        self.wait_item()
        self.stop.set()
        self.assertIsNone(self.pump.get())
        self.pump.close()
        self.assertFalse(self.pump.thread.is_alive())

    def test_close_wakes_empty_consumer(self):
        result = []
        consumer = threading.Thread(target=lambda: result.append(self.pump.get()))
        self.pump.start()
        self.stop.set()
        consumer.start()
        self.pump.close()
        consumer.join(timeout=1)
        self.assertFalse(consumer.is_alive())
        self.assertEqual(result, [None])


if __name__ == '__main__':
    unittest.main()
