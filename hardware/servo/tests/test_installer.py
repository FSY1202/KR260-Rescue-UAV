import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("servo_installer", ROOT / "deploy/install_candidate.py")
installer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(installer)


class InstallerTests(unittest.TestCase):
    def test_baseline_hashes_are_valid(self):
        for value in (installer.OLD_BIT, installer.OLD_DTBO, installer.OLD_CAMERA):
            self.assertRegex(value, r"^[0-9a-f]{64}$")

    def test_camera_change_only_replaces_hash_check(self):
        original = (ROOT / "tests/fixtures/camera_baseline.sh").read_bytes()
        # This known original is also present on the board; no permissive fallback.
        new_hash = "a" * 64
        patched = installer.camera_compat(original, new_hash)
        begin = original.index(b"        if ! printf")
        end = original.index(b"        found=0", begin)
        new_end = patched.index(b"        found=0", begin)
        self.assertEqual(patched[:begin], original[:begin])
        self.assertEqual(patched[new_end:], original[end:])
        self.assertIn(installer.OLD_BIT.encode(), patched)
        self.assertIn(new_hash.encode(), patched)
        self.assertIn(b"exit 1", patched[begin:new_end])

    def test_changed_camera_entry_is_not_overwritten(self):
        with self.assertRaises(RuntimeError):
            installer.camera_compat(b"somebody else's changed file", "a" * 64)

    def test_active_consumer_is_not_stopped(self):
        def fake(args, check=True):
            return subprocess.CompletedProcess(args, 0, stdout="active\n", stderr="")
        with patch.object(installer, "command", side_effect=fake) as called:
            with self.assertRaises(RuntimeError):
                installer.assert_consumers_stopped()
            self.assertEqual(called.call_count, 1)
            self.assertEqual(called.call_args[0][0][:2], ["systemctl", "show"])

    def test_rollback_rejects_unknown_destination_before_unload(self):
        with patch.object(installer, "command") as command:
            with self.assertRaises(RuntimeError):
                installer.restore_backup({"saved_files": {"/not/our/file": {}}})
            command.assert_not_called()


if __name__ == "__main__":
    unittest.main()
