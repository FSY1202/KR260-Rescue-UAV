"""One-frame prefetch; only this worker accesses the PL frame buffer.

The returned NumPy array owns its PS memory. A subsequent capture never writes
that array. A full mailbox blocks capture rather than accumulating old frames.
"""
import threading


class CapturePump:
    def __init__(self, grabber, stop):
        self.grabber = grabber
        self.stop = stop
        self.cv = threading.Condition()
        self.item = None
        self.closed = False
        self.thread = threading.Thread(target=self.run, daemon=True)

    def start(self):
        self.thread.start()

    def run(self):
        while not self.stop.is_set():
            with self.cv:
                self.cv.wait_for(lambda: self.item is None or self.closed or self.stop.is_set())
                if self.closed or self.stop.is_set():
                    return
            try:
                item = (self.grabber.capture(), None)
            except Exception as exc:
                item = (None, exc)
            with self.cv:
                if self.closed or self.stop.is_set():
                    return
                self.item = item
                self.cv.notify_all()
            if item[1] is not None:
                self.stop.wait(0.25)

    def get(self):
        with self.cv:
            self.cv.wait_for(lambda: self.item is not None or self.closed or self.stop.is_set())
            if self.closed or self.stop.is_set():
                return None
            value, error = self.item
            self.item = None
            self.cv.notify_all()
        if error is not None:
            raise error
        return value

    def close(self):
        with self.cv:
            self.closed = True
            self.cv.notify_all()
        # capture() has a bounded 2.5-second timeout. Never unmap live MMIO.
        self.thread.join(timeout=3)
        if self.thread.is_alive():
            raise RuntimeError('Capture worker did not stop; keep its MMIO mapped')
