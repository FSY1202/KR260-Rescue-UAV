#!/usr/bin/env python3
"""KR260 H7 servo preparation. No command in this tool enables a PWM output.

The installed camera/UART PL does not contain this new peripheral. Report that
honestly; never fall back to /dev/mem, bit-banged GPIO or an unrelated PWM chip.
"""
import argparse
import ctypes
import json
import mmap
import os
from pathlib import Path
import sys
import time

BASE_ADDRESS = 0xA0030000
HW_ID = 0x53525631
VERSION = 0x00010000
REG_ID, REG_VERSION, REG_ENABLE, REG_STATUS = 0x00, 0x04, 0x08, 0x0C
REG_PERIOD, REG_PULSE0, REG_PULSE1, REG_COMMIT = 0x10, 0x14, 0x18, 0x1C
REG_ACTIVE_PERIOD, REG_ACTIVE_PULSE0, REG_ACTIVE_PULSE1 = 0x20, 0x24, 0x28
REGISTER_BYTES = 0x2C
HERE = Path(__file__).resolve().parent
PIN_MAP = {
    "SER_0": {"header": "H7", "signal_pin": 1, "power_pin": 3,
              "ground_pin": 5, "gpio": 19, "fpga_pin": "Y13"},
    "SER_1": {"header": "H7", "signal_pin": 2, "power_pin": 4,
              "ground_pin": 6, "gpio": 16, "fpga_pin": "AB15"},
}


class ServoError(RuntimeError):
    pass


def validate_profile(profile):
    required = {"schema_version", "supply_voltage_v", "signal_voltage_v",
                "period_us", "pulse0_us", "pulse1_us", "enable_mask",
                "servo_model_confirmed"}
    if not isinstance(profile, dict) or set(profile) != required:
        raise ServoError("Unknown/missing configuration fields")
    for field in ("schema_version", "period_us", "pulse0_us", "pulse1_us", "enable_mask"):
        if type(profile[field]) is not int:
            raise ServoError(field + " must be an integer")
    if profile["schema_version"] != 1:
        raise ServoError("Unsupported configuration schema")
    if profile["supply_voltage_v"] != 5 or profile["signal_voltage_v"] != 3.3:
        raise ServoError("This board profile uses 5 V power and 3.3 V logic")
    if profile["enable_mask"] != 0:
        raise ServoError("Initialization must keep BOTH outputs disabled")
    if type(profile["servo_model_confirmed"]) is not bool:
        raise ServoError("servo_model_confirmed must be boolean")
    period = profile["period_us"]
    if not 10000 <= period <= 30000:
        raise ServoError("period_us outside the hardware range 10000..30000")
    for field in ("pulse0_us", "pulse1_us"):
        if not 500 <= profile[field] <= 2500 or profile[field] >= period:
            raise ServoError(field + " outside the hardware range 500..2500")
    return profile


def discover(sysfs=Path("/sys/class/uio"), dev=Path("/dev")):
    matches = []
    for node in sorted(Path(sysfs).glob("uio[0-9]*")):
        try:
            addr = int((node / "maps/map0/addr").read_text().strip(), 0)
            size = int((node / "maps/map0/size").read_text().strip(), 0)
            offset_file = node / "maps/map0/offset"
            offset = int(offset_file.read_text().strip(), 0) if offset_file.exists() else 0
        except (OSError, ValueError):
            continue
        if addr == BASE_ADDRESS:
            if size < REGISTER_BYTES or offset != 0:
                raise ServoError("Unexpected servo UIO mapping size/offset")
            matches.append((Path(dev) / node.name, size))
    if len(matches) > 1:
        raise ServoError("Ambiguous servo UIO address; nothing was written")
    return matches[0] if matches else None


class AtomicMMIO:
    """Local native helper performs volatile, aligned device accesses."""
    def __init__(self, device, size, library=HERE / "libservo_mmio.so", lock=True):
        import fcntl
        if not Path(library).is_file():
            raise ServoError("MMIO helper missing: run build_mmio.sh on KR260 first")
        self.library = ctypes.CDLL(str(library))
        self.library.servo_read32.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
        self.library.servo_read32.restype = ctypes.c_uint32
        self.library.servo_write32.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint32]
        self.library.servo_write32.restype = None
        self.size = size
        self.fd = os.open(device, os.O_RDWR | os.O_CLOEXEC)
        self.mapping = self.buffer = self.pointer = None
        try:
            if lock:
                fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.mapping = mmap.mmap(self.fd, size, flags=mmap.MAP_SHARED,
                                     prot=mmap.PROT_READ | mmap.PROT_WRITE, offset=0)
            self.buffer = (ctypes.c_char * size).from_buffer(self.mapping)
            # An address value avoids ctypes.cast's reference cycle, so close()
            # can release the exported buffer before closing the mmap.
            self.pointer = ctypes.c_void_p(ctypes.addressof(self.buffer))
        except BaseException:
            self.close()
            raise

    def _check(self, offset):
        if type(offset) is not int or offset < 0 or offset % 4 or offset + 4 > self.size:
            raise ServoError("Unaligned/out-of-bounds MMIO access")

    def read(self, offset):
        self._check(offset)
        return int(self.library.servo_read32(self.pointer, offset))

    def write(self, offset, value):
        self._check(offset)
        self.library.servo_write32(self.pointer, offset, value)

    def close(self):
        self.pointer = None
        self.buffer = None
        if self.mapping is not None:
            self.mapping.close()
            self.mapping = None
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


def require_abi(io):
    ident, version = io.read(REG_ID), io.read(REG_VERSION)
    if ident != HW_ID or version != VERSION:
        raise ServoError(f"Unexpected peripheral ID/version: 0x{ident:08x}/0x{version:08x}; no writes")


def read_status(io):
    require_abi(io)
    enabled, flags = io.read(REG_ENABLE), io.read(REG_STATUS)
    return {
        "state": "DISABLED" if not (enabled & 3 or flags & 0x300) else "ENABLED",
        "id": f"0x{HW_ID:08x}", "version": f"0x{VERSION:08x}",
        "requested_enable_mask": enabled & 3,
        "effective_enable_mask": (flags >> 8) & 3,
        "pending_commit": bool(flags & 1), "commit_error": bool(flags & 2),
        "active_period_us": io.read(REG_ACTIVE_PERIOD),
        "active_pulse0_us": io.read(REG_ACTIVE_PULSE0),
        "active_pulse1_us": io.read(REG_ACTIVE_PULSE1),
    }


def disable(io):
    require_abi(io)
    io.write(REG_ENABLE, 0)
    result = read_status(io)
    if result["state"] != "DISABLED":
        raise ServoError("Could not verify disabled output")
    return result


def initialize(io, profile):
    validate_profile(profile)  # Reject unsafe profiles before any device write.
    require_abi(io)           # Do not touch other peripherals.
    try:
        io.write(REG_ENABLE, 0)
        if io.read(REG_ENABLE) & 3 or io.read(REG_STATUS) & 0x300:
            raise ServoError("Output disable did not take effect")
        io.write(REG_PERIOD, profile["period_us"])
        io.write(REG_PULSE0, profile["pulse0_us"])
        io.write(REG_PULSE1, profile["pulse1_us"])
        io.write(REG_COMMIT, 1)
        deadline = time.monotonic() + 0.05
        while io.read(REG_STATUS) & 1:
            if time.monotonic() >= deadline:
                raise ServoError("Configuration commit timeout")
            time.sleep(0.001)
        result = read_status(io)
        if result["commit_error"]:
            raise ServoError("Hardware rejected configuration")
        for field in ("period_us", "pulse0_us", "pulse1_us"):
            if result["active_" + field] != profile[field]:
                raise ServoError("Configuration readback mismatch: " + field)
        if result["state"] != "DISABLED":
            raise ServoError("Unexpected enabled output")
        result["initialized"] = True
        result["movement_command_sent"] = False
        return result
    finally:
        # Also fail closed if configuration/readback fails. Never write enable=1.
        io.write(REG_ENABLE, 0)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", nargs="?", default="status", choices=("plan", "status", "init", "disable"))
    parser.add_argument("--config", type=Path, default=HERE / "servo_config.json")
    args = parser.parse_args(argv)
    try:
        if args.action == "plan":
            profile = validate_profile(json.loads(args.config.read_text(encoding="utf-8")))
            print(json.dumps({"state": "PREPARED_OFFLINE", "hardware_initialized": False,
                              "profile": profile, "pins": PIN_MAP}, indent=2))
            return 0
        found = discover()
        if not found:
            info = {"state": "PWM_NOT_LOADED", "hardware_initialized": False,
                    "movement_command_sent": False,
                    "reason": "Current PL has no SRV1 peripheral at 0xA0030000. No firmware was loaded or changed."}
            print(json.dumps(info, indent=2))
            return 0 if args.action == "status" else 3
        profile = None
        if args.action == "init":
            profile = validate_profile(json.loads(args.config.read_text(encoding="utf-8")))
        # A read-only status or one-way disable must remain available while a
        # bounded motion command owns the configuration lock. Neither enables.
        with AtomicMMIO(*found, lock=args.action == "init") as io:
            if args.action == "init":
                result = initialize(io, profile)
            elif args.action == "disable":
                result = disable(io)
            else:
                result = read_status(io)
        print(json.dumps(result, indent=2))
        return 0
    except (ServoError, OSError, ValueError) as error:
        print(json.dumps({"state": "ERROR", "error": str(error)}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
