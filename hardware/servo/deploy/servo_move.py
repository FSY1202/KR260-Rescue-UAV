#!/usr/bin/env python3
"""Explicit, single-channel, time-bounded TD-8120MG bench command.

No default channel/target, automatic sweep, startup motion, ROS or FC messages.
An angle is not inferred from an uncalibrated mechanical linkage. A normal
return, Ctrl-C or SIGTERM disables both channels. SIGKILL/power loss cannot run
Python cleanup; use the separate one-way disable command if needed.
"""
import argparse
import json
import math
from pathlib import Path
import signal
import sys
import time

import servo_init as servo


class StopRequested(RuntimeError):
    pass


def validate_request(channel, pulse_us, seconds, model):
    if model.get("model") != "TIANKONGRC TD-8120MG 180deg" or model.get("supply_voltage_v") != 5:
        raise servo.ServoError("Wrong servo model profile")
    if type(channel) is not int or channel not in (0, 1):
        raise servo.ServoError("Select channel 0 (H7 1/3/5) or 1 (H7 2/4/6)")
    low, high = model.get("initial_test_min_pulse_us"), model.get("initial_test_max_pulse_us")
    if type(low) is not int or type(high) is not int or not 500 <= low <= high <= 2500:
        raise servo.ServoError("Invalid configured pulse range")
    if type(pulse_us) is not int or not low <= pulse_us <= high:
        raise servo.ServoError(f"Configured pulse range is {low}..{high} us")
    if not isinstance(seconds, (int, float)) or isinstance(seconds, bool) or not math.isfinite(seconds) or not 0.2 <= seconds <= 3.0:
        raise servo.ServoError("Duration must be finite and within 0.2..3.0 seconds")
    if model.get("period_us") != 20000 or model.get("max_test_duration_s") != 3:
        raise servo.ServoError("Unexpected servo test limits")


def run_pulse(io, channel, pulse_us, seconds, profile, model, clock=time.monotonic, sleep=time.sleep):
    validate_request(channel, pulse_us, seconds, model)
    servo.require_abi(io)
    existing = servo.read_status(io)
    if existing["requested_enable_mask"] or existing["effective_enable_mask"]:
        raise servo.ServoError("Servo output already active; nothing was changed")
    settings = dict(profile)
    settings["period_us"] = 20000
    settings["pulse0_us" if channel == 0 else "pulse1_us"] = pulse_us
    servo.validate_profile(settings)
    owned = False
    try:
        owned = True
        servo.initialize(io, settings)
        mask = 1 << channel
        io.write(servo.REG_ENABLE, mask)  # The only explicit motion enable.
        start = clock()
        deadline = start + seconds
        while clock() < deadline:
            requested = io.read(servo.REG_ENABLE) & 3
            flags = io.read(servo.REG_STATUS)
            if requested != mask:
                raise StopRequested("Output disabled/changed externally; command cancelled")
            if flags & 2:
                raise servo.ServoError("PWM commit error during command")
            if clock() - start > 0.05 and ((flags >> 8) & 3) != mask:
                raise servo.ServoError("Expected PWM output did not become effective")
            sleep(min(0.01, max(0.0, deadline - clock())))
        return {"state": "DISABLED", "channel": channel, "commanded_pulse_us": pulse_us,
                "duration_s": seconds, "movement_command_sent": True,
                "physical_motion_verified": False, "mechanical_endpoints_calibrated": False}
    finally:
        if owned:
            io.write(servo.REG_ENABLE, 0)
            if io.read(servo.REG_ENABLE) & 3 or io.read(servo.REG_STATUS) & 0x300:
                raise servo.ServoError("Could not confirm output disabled after command")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channel", required=True, type=int, choices=(0, 1))
    parser.add_argument("--pulse-us", required=True, type=int)
    parser.add_argument("--seconds", type=float, default=1.0)
    args = parser.parse_args(argv)
    previous = {}
    def interrupted(signum, _frame):
        raise StopRequested("Interrupted by signal " + str(signum))
    try:
        model = json.loads((servo.HERE / "td8120mg.json").read_text(encoding="utf-8"))
        profile = json.loads((servo.HERE / "servo_config.json").read_text(encoding="utf-8"))
        validate_request(args.channel, args.pulse_us, args.seconds, model)
        servo.validate_profile(profile)
        found = servo.discover()
        if not found:
            raise servo.ServoError("PWM_NOT_LOADED: no SRV1 UIO peripheral; nothing was enabled")
        signals = [signal.SIGINT, signal.SIGTERM]
        if hasattr(signal, "SIGHUP"):
            signals.append(signal.SIGHUP)
        for sig in signals:
            previous[sig] = signal.signal(sig, interrupted)
        with servo.AtomicMMIO(*found) as io:
            print(json.dumps(run_pulse(io, args.channel, args.pulse_us, args.seconds, profile, model), indent=2))
        return 0
    except StopRequested as error:
        print(json.dumps({"state": "CANCELLED", "reason": str(error)}), file=sys.stderr)
        return 130
    except (servo.ServoError, OSError, ValueError) as error:
        print(json.dumps({"state": "ERROR", "reason": str(error)}), file=sys.stderr)
        return 2
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == "__main__":
    raise SystemExit(main())
