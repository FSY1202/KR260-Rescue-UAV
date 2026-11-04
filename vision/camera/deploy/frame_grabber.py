"""Claim a complete frozen frame; copying never races the continuous writer."""
import ctypes
import fcntl
import mmap
import os
from pathlib import Path
import time
import numpy as np


class FrameGrabber:
    WIDTH, HEIGHT, FRAME_BYTES, MAP_BYTES = 640, 480, 614400, 1048576

    def __init__(self, lock_path='/run/lock/kr260_camera_frame.lock'):
        self.lock_fd = os.open(lock_path, os.O_RDWR | os.O_CREAT, 0o600)
        try:
            fcntl.flock(self.lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            os.close(self.lock_fd)
            raise RuntimeError('Another frame capture/preview process already owns the camera buffer')
        self.fd = None
        self.mem = None
        try:
            found = []
            for path in Path('/sys/class/uio').glob('uio*'):
                address = path / 'maps/map0/addr'
                if address.exists() and int(address.read_text().strip(), 0) == 0xa1000000:
                    found.append(path)
            if len(found) != 1:
                raise RuntimeError('Full-frame PL/overlay not loaded: no unique UIO at 0xa1000000')
            self.fd = os.open('/dev/' + found[0].name, os.O_RDWR | os.O_SYNC)
            self.mem = mmap.mmap(self.fd, self.MAP_BYTES)
            self.base = ctypes.addressof(ctypes.c_char.from_buffer(self.mem))
            self.lib = ctypes.CDLL(str(Path(__file__).with_name('libcamera_mmio.so')))
            self.lib.camera_mmio_read32.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
            self.lib.camera_mmio_read32.restype = ctypes.c_uint32
            self.lib.camera_mmio_write32.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint32]
            self.lib.camera_mmio_write32.restype = None
            self.lib.camera_mmio_copy32.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32]
            self.lib.camera_mmio_copy32.restype = None
            if self.read(0) != 0x46524d32 or self.read(4) != 0x20000:
                raise RuntimeError('Continuous FRM2 hardware/overlay required; refusing a mismatched firmware')
            if self.read(0x14) != self.FRAME_BYTES or self.read(0x24) != (480 << 16 | 640):
                raise RuntimeError('Unexpected hardware frame geometry')
            self.write(8, 4)
            self.write(8, 1)
        except BaseException:
            self.close(abort=False)
            raise

    def read(self, offset):
        return self.lib.camera_mmio_read32(self.base, offset)

    def write(self, offset, value):
        self.lib.camera_mmio_write32(self.base, offset, value)

    def capture(self, timeout=2.5):
        started = time.monotonic_ns()
        try:
            deadline = time.monotonic() + timeout
            while True:
                status = self.read(12)
                if status & 2:
                    raise RuntimeError('Previous claimed frame was not released')
                if status & 8:
                    break
                if time.monotonic() >= deadline:
                    raise TimeoutError('No complete camera frame; check camera.sh start and camera wiring')
                time.sleep(0.001)
            self.write(8, 8)  # Atomic claim, independent of concurrent frame completion.
            if not self.read(12) & 2:
                raise RuntimeError('No complete frame could be claimed')
            acquired = time.monotonic_ns()
            for _ in range(4):
                age_high = self.read(0x48)
                age_low = self.read(0x40)
                if age_high == self.read(0x48):
                    age_cycles = (age_high << 32) | age_low
                    break
            else:
                raise RuntimeError('Unstable frame age counter')
            clock_hz = self.read(0x44)
            if not clock_hz:
                raise RuntimeError('Invalid hardware clock metadata')
            if age_cycles > clock_hz // 4:
                raise RuntimeError('Latest camera frame is over 250 ms old; refusing stale image')
            # Approximate EOF time from the PL age counter; not exposure time.
            frame_completed = acquired - age_cycles * 1000000000 // clock_hz
            sequence = self.read(0x10)
            bank = self.read(0x38)
            if bank not in (0, 1, 2):
                raise RuntimeError('Invalid claimed buffer bank')
            if self.read(0x20) != self.WIDTH * self.HEIGHT:
                raise RuntimeError('Incomplete pixel count; refusing image')
            array = np.empty(self.FRAME_BYTES // 4, dtype='<u4')
            self.lib.camera_mmio_copy32(self.base, 0x1000, array.ctypes.data, array.size)
            copied = time.monotonic_ns()
            if self.read(0x10) != sequence or self.read(0x38) != bank or not self.read(12) & 2:
                raise RuntimeError('Frame ownership changed while copying; refusing mixed image')
            pixels = array.view('<u2').reshape(self.HEIGHT, self.WIDTH)
            info = dict(frame_seq=sequence, width=self.WIDTH, height=self.HEIGHT,
                        capture_started_monotonic_ns=started,
                        capture_completed_monotonic_ns=frame_completed,
                        copy_completed_monotonic_ns=copied,
                        capture_ms=(acquired-started)/1e6, copy_ms=(copied-acquired)/1e6,
                        claim_wait_ms=(acquired-started)/1e6,
                        claimed_frame_age_ms=age_cycles*1000/clock_hz,
                        capture_errors=self.read(0x18), fifo_overflows=self.read(0x1c),
                        sensor_frames=self.read(0x30), skipped_frames=self.read(0x34),
                        buffer_bank=bank, capture_mode='continuous_triple_buffer')
            self.write(8, 2)
            return pixels, info
        except BaseException:
            self.write(8, 2)  # Never reset the stream because one read failed.
            raise

    def close(self, abort=True):
        if self.mem is not None:
            if abort:
                self.write(8, 4)
            self.mem.close()
            self.mem = None
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
        if self.lock_fd is not None:
            os.close(self.lock_fd)
            self.lock_fd = None
