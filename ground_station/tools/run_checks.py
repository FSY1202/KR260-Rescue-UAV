"""Run the portable checks without opening hardware, windows or SSH."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
DESKTOP = ROOT / 'pynq_z2_desktop/software'
GESTURE = ROOT / 'pynq_z2_gesture/software'


def main():
    sys.path[:0] = [str(DESKTOP), str(GESTURE)]
    suites = unittest.TestSuite()
    loader = unittest.TestLoader()
    for directory, pattern in [
        (DESKTOP, 'test_button_input.py'),
        (DESKTOP, 'test_point_cloud_map.py'),
        (DESKTOP, 'test_flight_status.py'),
        (DESKTOP, 'test_telemetry.py'),
        (ROOT / 'pynq_z2_gesture/tests', 'test_*.py'),
        (GESTURE, 'test_local_preview.py'),
        (GESTURE, 'test_preview_detail.py'),
    ]:
        # Discovery paths are distinct modules; each directory is its own root.
        suites.addTests(loader.discover(str(directory), pattern=pattern, top_level_dir=str(directory)))
    result = unittest.TextTestRunner(verbosity=2).run(suites)
    return 0 if result.wasSuccessful() else 1


if __name__ == '__main__':
    raise SystemExit(main())
