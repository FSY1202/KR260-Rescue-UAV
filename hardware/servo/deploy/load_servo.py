#!/usr/bin/env python3
"""Explicit, idempotent PL load after reboot. No motion, file replacement or ROS."""
import json
import os
import sys

import install_candidate as installer
import servo_init as servo


def main():
    if len(sys.argv) != 1:
        raise RuntimeError("Usage: sudo servo.sh load")
    if os.geteuid() != 0:
        raise RuntimeError("Use sudo for this fixed-purpose PL load")
    receipt = json.loads((installer.ROOT / "pl_installation_receipt.json").read_text())
    for name in installer.NAMES:
        path = installer.FW / name
        if installer.sha(path) != receipt["installed_sha256"][str(path)]:
            raise RuntimeError("Installed servo firmware changed; refusing to load")
    found = servo.discover()
    if found:
        # Do not reset an already loaded/possibly active servo or localization.
        with servo.AtomicMMIO(*found, lock=False) as io:
            print(json.dumps({"state": "ALREADY_LOADED", "servo": servo.read_status(io)}, indent=2))
        return
    installer.assert_consumers_stopped()
    previous = installer.active_profile()
    if previous == "kr260_ov5640":
        raise RuntimeError("Profile is already active but SRV1 is missing; no forced reload")
    try:
        if previous:
            installer.command(["/usr/bin/xmutil", "unloadapp"])
        installer.command(["/usr/bin/xmutil", "loadapp", "kr260_ov5640"])
        print(json.dumps({"state": "LOADED_DISABLED", "verification": installer.verify_loaded()}, indent=2))
    except BaseException:
        current = installer.active_profile()
        if current:
            installer.command(["/usr/bin/xmutil", "unloadapp"])
        if previous:
            installer.command(["/usr/bin/xmutil", "loadapp", previous])
        raise


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, ValueError) as error:
        print(json.dumps({"state": "ERROR", "reason": str(error)}), file=sys.stderr)
        raise SystemExit(2)
