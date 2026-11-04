#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
exec /usr/local/share/pynq-venv/bin/python3 "$root/software/gesture.py" --backend cpu "$@"
