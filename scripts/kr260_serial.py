#!/usr/bin/env python3
"""Run a command or transfer a file over an already logged-in USB console.

Uses echoed-off shell here-documents for transfers. Does not store credentials.
Do not run while another program owns the serial port.
"""
import argparse
import base64
from pathlib import Path
import re
import shlex
import sys
import time
import uuid


def run(port, command, timeout):
    marker = 'KR260_DONE_' + uuid.uuid4().hex
    port.write(b'\nset +e; stty -echo -hupcl clocal\n')
    time.sleep(0.2)
    port.reset_input_buffer()
    begin = marker + '_BEGIN'
    # A command may use set -e or cd. Confine both to a subshell: a failed
    # diagnostic must return a status, not log the shared console out.
    script = "printf '" + begin + "\\n'\n(\n" + command + "\n)\nprintf '\\n" + marker + ":%s\\n' \"$?\""
    encoded = base64.b64encode(script.encode()).decode()
    port.write(('eval "$(printf %s ' + shlex.quote(encoded) + ' | base64 -d)"\n').encode())
    deadline = time.monotonic() + timeout
    output = bytearray()
    pattern = re.compile(rb'\r?\n' + marker.encode() + rb':(\d+)\r?\n')
    while time.monotonic() < deadline:
        output.extend(port.read(max(1, port.in_waiting)))
        match = pattern.search(output)
        if match:
            start = re.search(begin.encode() + rb'\r?\n', output)
            if start:
                return bytes(output[start.end():match.start()]), int(match.group(1))
    # Preserve output for diagnosis; do not interrupt a potentially ongoing build.
    raise TimeoutError(output.decode(errors='replace'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='COM8')
    parser.add_argument('--timeout', type=float, default=60)
    sub = parser.add_subparsers(dest='action', required=True)
    sub.add_parser('exec').add_argument('command')
    for action in ('upload', 'download'):
        child = sub.add_parser(action)
        child.add_argument('local')
        child.add_argument('remote')
    args = parser.parse_args()
    import serial
    with serial.Serial(args.port, 115200, timeout=0.1, write_timeout=60) as port:
        if args.action == 'exec':
            output, code = run(port, args.command, args.timeout)
            sys.stdout.buffer.write(output)
        elif args.action == 'upload':
            payload = base64.encodebytes(Path(args.local).read_bytes()).decode('ascii')
            delimiter = 'TRANSFER_' + uuid.uuid4().hex
            output, code = run(port, 'base64 -d > ' + shlex.quote(args.remote) +
                " <<'" + delimiter + "'\n" + payload + delimiter, args.timeout)
            sys.stdout.buffer.write(output)
        else:
            output, code = run(port, 'base64 -w0 ' + shlex.quote(args.remote), args.timeout)
            if code == 0:
                Path(args.local).write_bytes(base64.b64decode(output.strip(), validate=True))
                print('Downloaded', args.local)
        return code


if __name__ == '__main__':
    raise SystemExit(main())
