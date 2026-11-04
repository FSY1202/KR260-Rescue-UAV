"""UDP receive-only service. Wi-Fi/Ethernet must already be configured."""
import argparse
import ipaddress
import os
from pathlib import Path
import signal
import socket
import time

from telemetry import MAX_PACKET, DEFAULT_STATE, TelemetryStore, decode_packet, atomic_write


def receive_once(sock, store, peer=None, key=None, now=None):
    raw, address = sock.recvfrom(MAX_PACKET+1)
    if peer is not None and address[0] != peer:
        store.rejected += 1
        return False
    try:
        packet = decode_packet(raw,key)
    except (ValueError, TypeError, UnicodeError, RecursionError):
        store.rejected += 1
        return False
    return store.accept(packet, time.monotonic() if now is None else now)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bind',default=os.environ.get('TELEMETRY_BIND','127.0.0.1'))
    p.add_argument('--port',type=int,default=14600)
    p.add_argument('--peer',default=os.environ.get('TELEMETRY_PEER'))
    p.add_argument('--vehicle',default='uav1')
    p.add_argument('--timeout',type=float,default=2.)
    p.add_argument('--restart-grace',type=float,default=3.)
    p.add_argument('--state',type=Path,default=Path(DEFAULT_STATE))
    p.add_argument('--key-file',type=Path,default=os.environ.get('TELEMETRY_KEY_FILE'))
    args = p.parse_args()
    try:
        bind = ipaddress.IPv4Address(args.bind)
        peer = str(ipaddress.IPv4Address(args.peer)) if args.peer else None
        if not bind.is_loopback and peer is None:
            p.error('Non-loopback bind requires --peer (sender IPv4)')
        if not 1 <= args.port <= 65535:p.error('Invalid UDP port')
        key = args.key_file.read_bytes().strip() if args.key_file else None
        if key is not None and len(key)<32:p.error('Key file requires at least 32 bytes')
        store = TelemetryStore(args.vehicle,args.timeout,restart_grace=args.restart_grace)
    except (OSError,ValueError) as exc:
        p.error(str(exc))
    stop = [False]
    signal.signal(signal.SIGTERM,lambda *_:stop.__setitem__(0,True))
    signal.signal(signal.SIGINT,lambda *_:stop.__setitem__(0,True))
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,65536)
        sock.bind((str(bind),args.port))
        sock.settimeout(.05)
        print(f'TELEMETRY_LISTEN {bind}:{args.port} vehicle={args.vehicle} receive_only=true',flush=True)
        publish = log = 0.
        atomic_write(args.state,store.snapshot(time.monotonic()))
        while not stop[0]:
            try:receive_once(sock,store,peer,key)
            except socket.timeout:pass
            now=time.monotonic()
            # Bounded 20 Hz publishing even during continuous UDP traffic.
            if now>=publish:
                atomic_write(args.state,store.snapshot(now))
                publish=now+.05
            if now>=log:
                print(f'TELEMETRY accepted={store.accepted} rejected={store.rejected} connected={store.snapshot(now)["connected"]}',flush=True)
                log=now+5.
        value=store.snapshot(time.monotonic())
        value['packet']=None  # Never leave an apparently live sample after clean exit.
        value['connected']=False
        atomic_write(args.state,value)


if __name__=='__main__':main()
