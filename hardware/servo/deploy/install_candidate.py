#!/usr/bin/env python3
"""Fixed-purpose ground-only PL installer. Does not run servo motion or ROS.

Default is check-only. --execute/--rollback require root and stopped consumers.
Only the existing profile's bit/dtbo and the camera's known-hash check change.
Flight package sources, launchers, PX4, network and service definitions do not.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT / "candidate_firmware"
BACKUP = ROOT / "backup/pl_before_servo_20261030"
FW = Path("/lib/firmware/xilinx/kr260_ov5640")
CAMERA = Path(os.environ.get("KR260_CAMERA_ENTRY", str(ROOT.parents[1]/"vision/camera/deploy/camera.sh")))
WORKSPACE = Path(os.environ.get("KR260_WORKSPACE", str(ROOT.parents[1])))
UNITS = ("kr260-position.service", "kr260-mission.service", "kr260-ego.service", "kr260-camera-preview.service")
OLD_BIT = "4d7d287668a878fb61d96be5412cb4eee19ba3a8ef319aa25d4ac8f21b44a7e9"
OLD_DTBO = "b92b60c8586d3057ad4782a1f88c7f08012c69006a25e2439c8f92f96ce61b98"
OLD_CAMERA = "eb11c9d13efa0fe9f053e0dd9c077715159d1eaab87dd70021264d85520d1e69"
assert all(re.fullmatch(r"[0-9a-f]{64}", x) for x in (OLD_BIT, OLD_DTBO, OLD_CAMERA))
NAMES = ("kr260_ov5640.bit.bin", "kr260_ov5640.dtbo")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def command(args, check=True):
    result = subprocess.run(args, text=True, capture_output=True, timeout=45)
    if check and result.returncode:
        raise RuntimeError(f"{args!r} failed: {result.stderr or result.stdout}")
    return result


def active_profile():
    output = command(["/usr/bin/xmutil", "listapps"]).stdout
    parsed, active = 0, []
    for line in output.splitlines():
        fields = line.split()
        if len(fields) >= 6 and re.fullmatch(r"-1|[0-9]+(?:,[0-9]+)*,?", fields[-1]):
            parsed += 1
            if fields[-1] != "-1":
                active.append(fields[0])
    if not parsed or len(active) > 1 or any(x not in ("k26-starter-kits", "kr260_ov5640") for x in active):
        raise RuntimeError("Unknown accelerator state; refusing to load/unload: " + output)
    return active[0] if active else None


def assert_consumers_stopped():
    for unit in UNITS:
        state = command(["systemctl", "show", unit, "-p", "ActiveState", "--value"], check=False).stdout.strip()
        if state in ("active", "activating", "deactivating", "reloading"):
            raise RuntimeError(unit + " is running; nothing stopped by this installer")
    node_names = {"MicroXRCEAgent", "livox_ros_driver2_node", "fastlio_mapping", "bridge_node", "kr260_mission_node"}
    scripts = {"camera_preview.py", "frame_grabber.py", "capture_pump.py"}
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            argv = (proc / "cmdline").read_bytes().split(b"\0")
            words = [Path(x.decode()).name for x in argv if x]
        except (OSError, UnicodeError):
            continue
        if words and (words[0] in node_names or any(x in scripts for x in words)):
            raise RuntimeError("External consumer present: " + proc.name + ":" + " ".join(words[:3]))
    if Path("/dev/ttyS0").exists() and command(["fuser", "-s", "/dev/ttyS0"], check=False).returncode == 0:
        raise RuntimeError("UART is in use; no forced takeover")


def atomic_write(path, data, mode, uid, gid):
    path = Path(path)
    fd, temporary = tempfile.mkstemp(prefix=".servo-update-", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fchmod(stream.fileno(), mode)
            os.fchown(stream.fileno(), uid, gid)
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def camera_compat(original, new_hash):
    if hashlib.sha256(original).hexdigest() != OLD_CAMERA or not re.fullmatch("[0-9a-f]{64}", new_hash):
        raise RuntimeError("Camera entry differs from audited version; refusing to overwrite")
    text = original.decode()
    begin = text.index("        if ! printf '%s  %s\\n'")
    end = text.index("        found=0", begin)
    check = (
        "        # Both known FRM2 images preserve the camera/UART ABI.\n"
        "        current_hash=$(sha256sum /lib/firmware/xilinx/kr260_ov5640/kr260_ov5640.bit.bin | cut -d ' ' -f 1)\n"
        "        case \"$current_hash\" in\n"
        f"            {OLD_BIT}|{new_hash}) ;;\n"
        "            *) echo 'Camera firmware checksum mismatch; refusing initialization' >&2; exit 1 ;;\n"
        "        esac\n"
    )
    return (text[:begin] + check + text[end:]).encode()


def unchanged_flight_entries():
    return {str(WORKSPACE / "scripts" / name): sha(WORKSPACE / "scripts" / name)
            for name in ("kr260_position.sh",)}


def verify_loaded():
    import servo_init as servo
    deadline = time.monotonic() + 8
    while not servo.discover():
        if time.monotonic() > deadline:
            raise RuntimeError("New servo UIO did not appear")
        time.sleep(0.1)
    with servo.AtomicMMIO(*servo.discover()) as io:
        status = servo.read_status(io)
        if status["state"] != "DISABLED":
            raise RuntimeError("Expected reset-disabled servo state")
        config = json.loads((ROOT / "deploy/servo_config.json").read_text())
        status = servo.initialize(io, config)  # All enable writes remain zero.
    uart = Path("/sys/class/tty/ttyS0/device").resolve()
    if uart.name != "a0020000.serial":
        raise RuntimeError("UART1 binding was not preserved: " + str(uart))
    expected = {0xA0000000: 0x434F4C31, 0xA1000000: 0x46524D32}
    observed = {}
    for node in Path("/sys/class/uio").glob("uio*"):
        address = int((node / "maps/map0/addr").read_text().strip(), 0)
        if address in expected:
            size = int((node / "maps/map0/size").read_text().strip(), 0)
            with servo.AtomicMMIO(Path("/dev") / node.name, size, lock=False) as io:
                observed[address] = io.read(0)
    if observed != expected:
        raise RuntimeError("Original camera peripheral IDs not preserved: " + repr(observed))
    return {"servo": status, "uart": str(uart), "camera_ids": {hex(k): hex(v) for k, v in observed.items()}}


def restore_backup(record):
    allowed = {str(FW / name) for name in NAMES} | {str(CAMERA)}
    if set(record["saved_files"]) != allowed:
        raise RuntimeError("Unexpected rollback target paths")
    for original, info in record["saved_files"].items():
        if info["saved_as"] != Path(original).name:
            raise RuntimeError("Unexpected backup file name")
        if sha(BACKUP / info["saved_as"]) != info["sha256"]:
            raise RuntimeError("Backup checksum mismatch; nothing unloaded")
    active = active_profile()
    if active:
        command(["/usr/bin/xmutil", "unloadapp"])
    for original, info in record["saved_files"].items():
        data = (BACKUP / info["saved_as"]).read_bytes()
        if hashlib.sha256(data).hexdigest() != info["sha256"]:
            raise RuntimeError("Backup checksum mismatch")
        atomic_write(Path(original), data, info["mode"], info["uid"], info["gid"])
    if record["previous_active"]:
        command(["/usr/bin/xmutil", "loadapp", record["previous_active"]])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--execute", action="store_true")
    group.add_argument("--rollback", action="store_true")
    args = parser.parse_args()
    if os.geteuid() != 0:
        parser.error("root is required to inspect/prepare the fixed PL profile")
    assert_consumers_stopped()
    if args.rollback:
        record = json.loads((BACKUP / "manifest.json").read_text())
        restore_backup(record)
        print("ROLLED_BACK: previous firmware/camera hash check restored; no flight or servo command started")
        return
    manifest = json.loads((STAGE / "manifest.json").read_text())
    for name in NAMES:
        if sha(STAGE / name) != manifest["files"][name]:
            raise RuntimeError("Candidate checksum mismatch: " + name)
    if sha(FW / NAMES[0]) != OLD_BIT or sha(FW / NAMES[1]) != OLD_DTBO or sha(CAMERA) != OLD_CAMERA:
        raise RuntimeError("Installed baseline changed; refusing to overwrite")
    previous = active_profile()
    replacement = camera_compat(CAMERA.read_bytes(), manifest["files"][NAMES[0]])
    flight_before = unchanged_flight_entries()
    if not args.execute:
        print(json.dumps({"state": "CHECKED_NOT_INSTALLED", "previous_active": previous,
                          "candidate": manifest, "flight_entries": flight_before}, indent=2))
        return
    if BACKUP.exists():
        raise RuntimeError("Backup already exists; refusing duplicate installation")
    BACKUP.mkdir(parents=True)
    record = {"previous_active": previous, "saved_files": {}, "flight_entries_before": flight_before,
              "servo_motion_sent": False, "candidate": manifest}
    for source in [FW / name for name in NAMES] + [CAMERA]:
        st = source.stat()
        saved_as = source.name
        shutil.copy2(source, BACKUP / saved_as)
        record["saved_files"][str(source)] = {"saved_as": saved_as, "sha256": sha(source),
            "mode": stat.S_IMODE(st.st_mode), "uid": st.st_uid, "gid": st.st_gid}
    (BACKUP / "manifest.json").write_text(json.dumps(record, indent=2) + "\n")
    mutated = False
    try:
        assert_consumers_stopped()
        mutated = True
        if previous:
            command(["/usr/bin/xmutil", "unloadapp"])
        for name in NAMES:
            info = record["saved_files"][str(FW / name)]
            atomic_write(FW / name, (STAGE / name).read_bytes(), info["mode"], info["uid"], info["gid"])
        info = record["saved_files"][str(CAMERA)]
        atomic_write(CAMERA, replacement, info["mode"], info["uid"], info["gid"])
        command(["sh", "-n", str(CAMERA)])
        command(["/usr/bin/xmutil", "loadapp", "kr260_ov5640"])
        record["loaded_verification"] = verify_loaded()
        if unchanged_flight_entries() != flight_before:
            raise RuntimeError("Flight entries changed unexpectedly")
        record["state"] = "INSTALLED_BOTH_SERVO_OUTPUTS_DISABLED"
        record["installed_sha256"] = {str(p): sha(p) for p in [FW / name for name in NAMES] + [CAMERA]}
        (ROOT / "pl_installation_receipt.json").write_text(json.dumps(record, indent=2) + "\n")
        print(json.dumps(record, indent=2))
    except BaseException:
        if mutated:
            restore_backup(record)
        raise


if __name__ == "__main__":
    main()
