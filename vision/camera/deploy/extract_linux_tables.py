"""Extract the GPL-2.0-or-later register data, preserving provenance."""
import hashlib
import json
import re
import sys
from pathlib import Path

source = Path(sys.argv[1]).read_bytes()
text = source.decode()
tables = {}
for name in ("ov5640_init_setting_30fps_VGA", "ov5640_setting_VGA_640_480"):
    body = re.search(r"static const struct reg_value " + name + r"\[\] = \{(.*?)\n\};", text, re.S)
    if not body:
        raise RuntimeError(name + " not found")
    rows = re.findall(r"\{\s*(0x[0-9a-f]+),\s*(0x[0-9a-f]+),\s*(\d+),\s*(\d+)\s*\}", body[1])
    tables[name] = [[int(a, 16), int(v, 16), int(m), int(d)] for a, v, m, d in rows]
    if not rows or any(r[2] for r in tables[name]):
        raise RuntimeError("Unexpected empty or masked table")
out = {"license": "GPL-2.0-or-later", "copyright": [
    "2011-2013 Freescale Semiconductor, Inc.", "2014-2017 Mentor Graphics Inc."],
    "source": "https://github.com/torvalds/linux/blob/v5.10/drivers/media/i2c/ov5640.c",
    "source_sha256": hashlib.sha256(source).hexdigest(), "tables": tables}
Path(sys.argv[2]).write_text(json.dumps(out, indent=2) + "\n")
print({k: len(v) for k, v in tables.items()})
