#!/usr/bin/env python3
"""OV5640 SCCB setup and existing PL statistics; never opens the FC UART."""
import argparse
import ctypes
import json
import mmap
import os
from pathlib import Path
import time


class Message(ctypes.Structure):
    _fields_ = [("addr", ctypes.c_uint16), ("flags", ctypes.c_uint16),
                ("len", ctypes.c_uint16), ("buf", ctypes.POINTER(ctypes.c_ubyte))]


class Transfer(ctypes.Structure):
    _fields_ = [("msgs", ctypes.POINTER(Message)), ("nmsgs", ctypes.c_uint32)]


class Sensor:
    def __init__(self):
        paths = [p for p in Path('/sys/class/i2c-dev').glob('i2c-*')
                 if 'a0010000' in str(p.resolve())]
        if len(paths) != 1:
            raise RuntimeError('Expected exactly one AXI IIC a0010000; load matching overlay first')
        self.fd = os.open('/dev/' + paths[0].name, os.O_RDWR)

    def transfer(self, data, read_length=0):
        tx = (ctypes.c_ubyte * len(data))(*data)
        rx = (ctypes.c_ubyte * read_length)()
        messages = [Message(0x3c, 0, len(tx), tx)]
        if read_length:
            messages.append(Message(0x3c, 1, read_length, rx))
        array = (Message * len(messages))(*messages)
        request = Transfer(array, len(messages))
        libc = ctypes.CDLL(None, use_errno=True)
        result = libc.ioctl(self.fd, 0x0707, ctypes.byref(request))
        if result != len(messages):
            err = ctypes.get_errno()
            raise OSError(err, os.strerror(err) if result < 0 else 'Incomplete I2C transfer')
        return bytes(rx)

    def read(self, address):
        return self.transfer([address >> 8, address & 255], 1)[0]

    def write(self, address, value):
        try:
            self.transfer([address >> 8, address & 255, value])
        except OSError as exc:
            raise RuntimeError(f'SCCB write {address:#06x}={value:#04x}: {exc}') from exc


class Detector:
    def __init__(self):
        matches = []
        for path in Path('/sys/class/uio').glob('uio*'):
            addr = path / 'maps/map0/addr'
            if addr.exists() and int(addr.read_text().strip(), 0) == 0xa0000000:
                matches.append(path)
        if len(matches) != 1:
            raise RuntimeError('Expected one color detector UIO at a0000000')
        self.fd = os.open('/dev/' + matches[0].name, os.O_RDWR | os.O_SYNC)
        self.mem = mmap.mmap(self.fd, 4096)
        self.base = ctypes.addressof(ctypes.c_char.from_buffer(self.mem))
        self.io = ctypes.CDLL(str(Path(__file__).with_name('libcamera_mmio.so')))
        self.io.camera_mmio_read32.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
        self.io.camera_mmio_read32.restype = ctypes.c_uint32
        self.io.camera_mmio_write32.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint32]
        self.io.camera_mmio_write32.restype = None
        if self.read(0) != 0x434f4c31:
            raise RuntimeError('Unexpected detector ID; refusing writes')

    def read(self, offset):
        if offset < 0 or offset >= 4096 or offset % 4:
            raise ValueError('MMIO offset must be aligned and inside mapped page')
        return self.io.camera_mmio_read32(self.base, offset)

    def write(self, offset, value):
        if offset < 0 or offset >= 4096 or offset % 4 or not 0 <= value <= 0xffffffff:
            raise ValueError('Invalid 32-bit MMIO access')
        self.io.camera_mmio_write32(self.base, offset, value)

    def snapshot(self):
        for _ in range(10):
            first = self.read(0x10)
            raw = [self.read(i) for i in range(0x10, 0x8c, 4)]
            if first == self.read(0x10):
                break
        else:
            raise RuntimeError('Unable to read coherent frame snapshot')
        result = dict(frame_seq=first, width=raw[1] & 65535, height=raw[1] >> 16,
                      sync_errors=raw[2], dropped_notifications=raw[3],
                      sample_rgb565=hex(raw[5]))
        for name, off in [('red', 0x50), ('green', 0x64), ('blue', 0x78)]:
            idx = (off - 0x10) // 4
            count, sx, sy, xb, yb = raw[idx:idx+5]
            result[name] = dict(count=count, valid=count >= self.read({'red':0x34,'green':0x3c,'blue':0x44}[name]),
                                center_x=sx/count if count else None,
                                center_y=sy/count if count else None,
                                bounds=[xb & 65535, yb & 65535, xb >> 16, yb >> 16])
        self.write(0x8c, 1)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['probe', 'init', 'status', 'watch', 'stop'])
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--color-bars', action='store_true')
    args = parser.parse_args()
    detector = Detector()
    if args.action == 'stop':
        detector.write(8, 4)
        print('Camera held in reset/powerdown; UART untouched')
        return
    if args.action in ('probe', 'init'):
        detector.write(8, 0)
        time.sleep(0.01)
        detector.write(8, 2)
        time.sleep(0.05)
        sensor = Sensor()
        chip_id = (sensor.read(0x300a) << 8) | sensor.read(0x300b)
        print(json.dumps(dict(sensor_id=hex(chip_id))), flush=True)
        if chip_id != 0x5640:
            raise RuntimeError('Not OV5640; initialization refused')
        if args.action == 'probe':
            return
        # Establish the PLL before selecting it; do not wake with reset defaults.
        sensor.write(0x3103, 0x11)
        sensor.write(0x3008, 0x82)
        time.sleep(0.02)
        sensor.write(0x3008, 0x42)
        clocks = [(0x3034,0x18),(0x3035,0x21),(0x3036,0x60),
                  (0x3037,0x13),(0x3108,0x01)]
        for address, value in clocks:
            sensor.write(address, value)
        sensor.write(0x3103, 0x03)
        tables = json.loads(Path(__file__).with_name('ov5640_linux_v5_10_tables.json').read_text())['tables']
        for rows in tables.values():
            for address, value, mask, delay_ms in rows:
                # Keep the sensor in standby until PLL, format and pads are set.
                if address in (0x3008, 0x3103):
                    continue
                sensor.write(address, value)
                if delay_ms:
                    time.sleep(delay_ms / 1000)
        sensor.write(0x3008, 0x42)
        # 24 MHz oscillator -> 48 MHz DVP; HTS 1600 * VTS 1000 -> 30 FPS.
        # DVP pad/clock enables follow STM32's OV5640 DVP setup. The actual
        # module requires 0x4740=0x20 for HREF high / VSYNC blanking high.
        profile = clocks + [(0x3808,0x02),(0x3809,0x80),(0x380a,0x01),
                   (0x380b,0xe0),(0x380c,0x06),(0x380d,0x40),(0x380e,0x03),
                   (0x380f,0xe8),(0x3a02,0x03),(0x3a03,0xe8),
                   (0x3a14,0x03),(0x3a15,0xe8),(0x3a08,0x01),(0x3a09,0x2c),
                   (0x3a0a,0x00),(0x3a0b,0xfa),
                   (0x4300,0x61),(0x501f,0x01),(0x300e,0x58),(0x302e,0x00),
                   (0x3017,0xff),(0x3018,0xf3),(0x460c,0x23),(0x4713,0x03),
                   (0x4740,0x20),(0x4730,0x00),
                   (0x3503,0x00),(0x3406,0x00),(0x503d,0x80 if args.color_bars else 0x00)]
        for address, value in profile:
            sensor.write(address, value)
        sensor.write(0x3008, 0x02)
        time.sleep(0.1)
        for address in (0x3035,0x3036,0x3017,0x3018,0x3808,0x3809,0x380a,0x380b,0x4300,0x501f,0x4740):
            expected = dict(profile)[address]
            actual = sensor.read(address)
            if expected != actual:
                raise RuntimeError(f'Register {address:#06x}: expected {expected:#x}, got {actual:#x}')
        detector.write(0x20, (240 << 16) | 320)
        detector.write(8, 0x03)  # Release reset and enable; IRQ masked.
        time.sleep(0.1)
        detector.write(8, 0x13)  # Commit shadow config after the PCLK domain is running.
        time.sleep(0.08)
        print('RGB565 BE, VGA configured; stream rate/sync require measured verification', flush=True)
    if args.action == 'status':
        print(json.dumps(detector.snapshot(), ensure_ascii=False))
        return
    deadline = time.monotonic() + args.seconds
    start_seq, start_time = detector.read(0x10), time.monotonic()
    previous, next_log, seen = start_seq, start_time, 0
    while time.monotonic() < deadline:
        snap = detector.snapshot()
        if snap['frame_seq'] != previous:
            previous = snap['frame_seq']
            seen += 1
            if (snap['width'], snap['height']) != (640,480):
                raise RuntimeError('Unexpected image dimensions: ' + json.dumps(snap))
        now = time.monotonic()
        if now >= next_log:
            snap['measured_fps'] = ((snap['frame_seq'] - start_seq) & 0xffffffff) / (now - start_time)
            print(json.dumps(snap), flush=True)
            next_log = now + 1
        time.sleep(0.005)
    if not seen:
        raise RuntimeError('No new DVP frames: check clock/reset/wiring/sensor setup')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        raise SystemExit('CAMERA_ERROR: ' + str(exc))
