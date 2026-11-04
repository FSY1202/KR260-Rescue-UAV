"""Own the combined PL image, boot framebuffer and native DRM device."""
from pathlib import Path
import json
import os
import pwd
import secrets
import signal
import socket
import stat
import subprocess
import time

import numpy as np
from pynq import Overlay, MMIO, allocate
from button_input import ButtonPress

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = Path('/run/pynq-desktop')
stop = False


def command(*args, **kwargs):
    return subprocess.run(args, check=True, **kwargs)


def notify(message):
    address = os.environ.get('NOTIFY_SOCKET')
    if address:
        if address.startswith('@'):
            address = '\0'+address[1:]
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as client:
            client.sendto(message.encode(), address)


def finish(signum, frame):
    global stop
    stop = True


def main():
    signal.signal(signal.SIGTERM, finish)
    signal.signal(signal.SIGINT, finish)
    RUNTIME.mkdir(mode=0o755, exist_ok=True)
    frame = None
    mmio = None
    module_loaded = False
    try:
        bitfile = ROOT/'output/desktop_gesture.bit'
        print('DESKTOP_PHASE loading_overlay', flush=True)
        overlay = Overlay(str(bitfile))
        print('DESKTOP_PHASE checking_gesture_core', flush=True)
        version = overlay.processing_config.mmio.read(8)
        if version != 0x47535432:
            raise RuntimeError('Combined overlay has an incompatible gesture core')
        if 'board_controls' not in overlay.ip_dict:
            raise RuntimeError('This desktop requires the overlay with board controls')
        controls = overlay.board_controls.mmio
        raw = controls.read(0)
        button = ButtonPress(raw & 1, time.monotonic())
        button1 = ButtonPress((raw >> 1) & 1, time.monotonic())
        button2 = ButtonPress((raw >> 2) & 1, time.monotonic())
        sequence = 0
        sequence1 = 0
        sequence2 = 0
        request = RUNTIME/'camera-button.json'
        request1 = RUNTIME/'map-button.json'
        request2 = RUNTIME/'flight-status-button.json'
        selection = RUNTIME/'view-selection.json'
        selected_view = 'desktop'
        revision = 0

        def select_view(view):
            nonlocal selected_view, revision
            selected_view = 'desktop' if selected_view == view else view
            revision += 1
            temporary = selection.with_suffix('.tmp')
            temporary.write_text(json.dumps({'view': selected_view, 'revision': revision}))
            os.chmod(temporary, 0o644)
            temporary.replace(selection)

        def save_button():
            temporary = request.with_suffix('.tmp')
            temporary.write_text(json.dumps({'sequence': sequence, 'button': 'BTN0'}))
            os.chmod(temporary, 0o644)
            temporary.replace(request)

        def save_map_button():
            temporary = request1.with_suffix('.tmp')
            temporary.write_text(json.dumps({'sequence': sequence1, 'button': 'BTN1'}))
            os.chmod(temporary, 0o644)
            temporary.replace(request1)

        def save_flight_button():
            temporary = request2.with_suffix('.tmp')
            temporary.write_text(json.dumps({'sequence': sequence2, 'button': 'BTN2'}))
            os.chmod(temporary, 0o644)
            temporary.replace(request2)

        save_button()
        save_map_button()
        save_flight_button()
        select_view('desktop')
        address = overlay.ip_dict['hdmi_vdma']['phys_addr']
        mmio = MMIO(address, 0x10000)
        print('DESKTOP_PHASE resetting_vdma '+hex(address), flush=True)
        mmio.write(0, 4)
        deadline = time.monotonic()+2
        while mmio.read(0) & 4:
            if time.monotonic() > deadline:
                raise TimeoutError('HDMI VDMA reset did not complete')
            time.sleep(.001)
        frame = allocate(shape=(600, 800), dtype=np.uint32, cacheable=False)
        print('DESKTOP_PHASE starting_boot_frame', flush=True)
        frame[:] = 0x00203048
        frame.flush()
        for offset in (0x5c, 0x60, 0x64):
            mmio.write(offset, frame.physical_address)
        mmio.write(0x58, 3200)
        mmio.write(0x54, 3200)
        mmio.write(0x28, 0)
        mmio.write(0, 0x00010001)
        mmio.write(0x50, 600)
        print('DESKTOP_PHASE loading_drm_driver', flush=True)
        command('/sbin/insmod', str(ROOT/'kernel/pynq_hdmi_drm.ko'),
                'vdma_base='+hex(address))
        module_loaded = True
        card = None
        for candidate in Path('/sys/class/drm').glob('card[0-9]*'):
            if not candidate.name[4:].isdigit():
                continue
            module = candidate/'device/driver/module'
            if module.exists() and module.resolve().name == 'pynq_hdmi_drm':
                card = Path('/dev/dri')/candidate.name
                break
        if card is None:
            raise RuntimeError('The HDMI DRM card did not appear')
        # /run is mounted nodev; DRM character nodes must live on /dev.
        private = Path('/dev/pynq-desktop-dri')
        private.mkdir(exist_ok=True)
        node = private/card.name
        if node.exists():
            node.unlink()
        os.mknod(node, stat.S_IFCHR | 0o660, card.stat().st_rdev)
        config = (ROOT/'software/xorg-hdmi.conf').read_text()
        (RUNTIME/'xorg.conf').write_text(config.replace('/dev/dri/card1', str(card)))
        user = pwd.getpwnam('xilinx')
        auth = RUNTIME/'Xauthority'
        auth.touch(mode=0o600)
        command('/usr/bin/xauth', '-f', str(auth), input=(
            'add :0 MIT-MAGIC-COOKIE-1 '+secrets.token_hex(16)+'\n').encode())
        os.chown(auth, user.pw_uid, user.pw_gid)
        os.chmod(auth, 0o600)
        state = {'bitfile': str(bitfile), 'drm_card':str(card), 'width':800,
                 'height':600, 'vdma_base':hex(address), 'gesture_version':hex(version),
                 'controls_base': hex(overlay.ip_dict['board_controls']['phys_addr'])}
        (RUNTIME/'state.json').write_text(json.dumps(state, indent=2))
        print('DESKTOP_HARDWARE_READY '+json.dumps(state), flush=True)
        notify('READY=1\nSTATUS=HDMI and gesture hardware ready')
        while not stop:
            now = time.monotonic()
            raw = controls.read(0)
            if button.update(raw & 1, now):
                sequence += 1
                save_button()
                select_view('camera')
                print('CAMERA_BUTTON BTN0 sequence='+str(sequence), flush=True)
            if button1.update((raw >> 1) & 1, now):
                sequence1 += 1
                save_map_button()
                select_view('map')
                print('MAP_BUTTON BTN1 sequence='+str(sequence1), flush=True)
            if button2.update((raw >> 2) & 1, now):
                sequence2 += 1
                save_flight_button()
                select_view('flight_status')
                print('FLIGHT_BUTTON BTN2 sequence='+str(sequence2), flush=True)
            time.sleep(.01)
    finally:
        notify('STOPPING=1')
        # Unit ordering stops Xorg (and releases DRM framebuffers) first.
        if mmio is not None:
            mmio.write(0, 4)
            deadline = time.monotonic()+2
            while mmio.read(0) & 4:
                if time.monotonic() > deadline:
                    raise TimeoutError('VDMA must stop before its boot buffer is freed')
                time.sleep(.001)
        if module_loaded:
            command('/sbin/rmmod', 'pynq_hdmi_drm')
        if frame is not None:
            frame.freebuffer()


if __name__ == '__main__':
    main()
