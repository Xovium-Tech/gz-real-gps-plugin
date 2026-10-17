"""Exercise adapter installation only in isolated, synthetic PX4 trees."""

import argparse
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import stat
import subprocess
import tempfile
import unittest
from unittest.mock import patch


SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "px4_gnss_bridge.py"
SPEC = importlib.util.spec_from_file_location("px4_gnss_bridge", SCRIPT)
bridge = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bridge)

SOURCE = """// Preserve pre-existing local changes and formatting.
#include "GZBridge.hpp"

void GZBridge::navSatCallback(const gz::msgs::NavSat &msg)
{
\tconst uint64_t timestamp = hrt_absolute_time();
\taddGpsNoise(msg);
\tsensor_gps_s sensor_gps{};
\t_sensor_gps_pub.publish(sensor_gps);
\t_gpos_ground_truth_pub.publish(gps_truth);
}

// Unrelated callbacks must stay unchanged.
"""
MESSAGE = """uint64 timestamp
uint64 timestamp_sample
uint32 device_id
float64 latitude_deg
float64 longitude_deg
float64 altitude_msl_m
float64 altitude_ellipsoid_m
float32 eph
float32 epv
float32 hdop
float32 vdop
float32 s_variance_m_s
float32 c_variance_rad
uint8 satellites_used
uint8 fix_type
float32 vel_n_m_s
float32 vel_e_m_s
float32 vel_d_m_s
float32 vel_m_s
float32 cog_rad
bool vel_ned_valid
uint64 time_utc_usec
uint8 jamming_state
uint8 spoofing_state
float32 heading
float32 heading_offset
"""
HEADER = "sensor_gps_publisher _sensor_gps_pub;\nground_truth_publisher _gpos_ground_truth_pub;\n"


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.root / bridge.BRIDGE
        self.source.parent.mkdir(parents=True)
        self.source.write_bytes(SOURCE.encode())
        self.source.chmod(0o640)
        self.source.with_name("GZBridge.hpp").write_text(HEADER)
        (self.root / "msg").mkdir()
        (self.root / "msg" / "SensorGps.msg").write_text(MESSAGE)
        self.helper = self.source.with_name("GZRealGps.hpp")
        self.backup = self.source.with_name("GZBridge.cpp.gz-real-gps.backup")
        self.state = self.source.with_name(".gz-real-gps-adapter.json")
        self.stdout = io.StringIO()
        self.stderr = io.StringIO()
        git = patch.object(bridge.subprocess, "run", return_value=subprocess.CompletedProcess([], 0, "v1.16.0-dirty\n"))
        git.start()
        self.addCleanup(git.stop)

    def run_adapter(self, apply=False, revert=False):
        args = argparse.Namespace(px4=self.root, apply=apply, revert=revert, check=not (apply or revert))
        with contextlib.redirect_stdout(self.stdout), contextlib.redirect_stderr(self.stderr):
            return bridge.run(args)

    def snapshot(self):
        return {str(path.relative_to(self.root)): (path.read_bytes(), stat.S_IMODE(path.stat().st_mode))
                for path in self.root.rglob("*") if path.is_file()}

    def test_preview_is_read_only_and_contains_adapter_diff(self):
        before = self.snapshot()
        self.assertEqual(self.run_adapter(), 0)
        self.assertEqual(self.snapshot(), before)
        self.assertIn("+" + bridge.INCLUDE, self.stdout.getvalue())
        self.assertIn(bridge.BEGIN, self.stdout.getvalue())
        self.assertIn("Preview only", self.stderr.getvalue())

    def test_apply_reapply_and_revert_preserve_previous_source_and_mode(self):
        before = self.snapshot()
        self.assertEqual(self.run_adapter(apply=True), 0)
        self.assertEqual(self.backup.read_bytes(), SOURCE.encode())
        patched = self.source.read_text()
        snippet = (bridge.ASSETS / "navsat_callback.inc").read_text()
        self.assertEqual(patched.replace(bridge.INCLUDE, "", 1).replace(snippet, "", 1), SOURCE)
        self.assertEqual(patched.count(bridge.BEGIN), 1)
        self.assertEqual(stat.S_IMODE(self.source.stat().st_mode), 0o640)
        state = json.loads(self.state.read_text())
        self.assertEqual(state["original_sha256"], bridge.digest(SOURCE.encode()))
        self.assertEqual(state["patched_sha256"], bridge.digest(self.source.read_bytes()))
        self.assertEqual(state["helper_sha256"], bridge.digest(self.helper.read_bytes()))
        installed = self.snapshot()
        self.assertEqual(self.run_adapter(apply=True), 0)
        self.assertEqual(self.snapshot(), installed)
        self.assertEqual(self.run_adapter(revert=True), 0)
        self.assertEqual(self.snapshot(), before)

    def test_changed_installed_files_are_never_overwritten(self):
        self.run_adapter(apply=True)
        installed = self.snapshot()
        for path in (self.source, self.helper, self.backup):
            with self.subTest(path=path.name):
                original = path.read_bytes()
                path.write_bytes(original + b"\n// new local edit\n")
                edited = self.snapshot()
                for operation in ({"apply": True}, {"revert": True}, {}):
                    with self.assertRaisesRegex(bridge.CompatibilityError, "refusing to overwrite edits"):
                        self.run_adapter(**operation)
                    self.assertEqual(self.snapshot(), edited)
                path.write_bytes(original)
        self.assertEqual(self.snapshot(), installed)

    def test_rejects_new_failure_injection_and_locking_without_writes(self):
        for statement in ("if (_failure_config.gps_off) { return; }", "pthread_mutex_lock(&lock);"):
            with self.subTest(statement=statement):
                self.source.write_text(SOURCE.replace("\taddGpsNoise(msg);", "\t" + statement + "\n\taddGpsNoise(msg);"))
                before = self.snapshot()
                with self.assertRaisesRegex(bridge.CompatibilityError, "manual integration"):
                    self.run_adapter(apply=True)
                self.assertEqual(self.snapshot(), before)

    def test_rejects_incompatible_callback_and_crlf_without_writes(self):
        cases = (
            SOURCE.replace("const gz::msgs::NavSat &msg", "const ignition::msgs::NavSat &msg"),
            SOURCE.replace("addGpsNoise(msg);", "changedNoiseAPI(msg);"),
            SOURCE.replace("\n", "\r\n"),
            SOURCE + SOURCE,
            SOURCE + "\n// " + bridge.BEGIN,
        )
        for source in cases:
            with self.subTest(source=source[:70]):
                self.source.write_bytes(source.encode())
                before = self.snapshot()
                with self.assertRaises(bridge.CompatibilityError):
                    self.run_adapter(apply=True)
                self.assertEqual(self.snapshot(), before)

    def test_rejects_missing_message_field_and_publisher_without_writes(self):
        message = self.root / "msg" / "SensorGps.msg"
        header = self.source.with_name("GZBridge.hpp")
        for path, content in ((message, MESSAGE.replace("uint8 spoofing_state\n", "")),
                              (header, HEADER.replace("_gpos_ground_truth_pub", "_renamed_pub"))):
            with self.subTest(path=path.name):
                original = path.read_text()
                path.write_text(content)
                before = self.snapshot()
                with self.assertRaises(bridge.CompatibilityError):
                    self.run_adapter(apply=True)
                self.assertEqual(self.snapshot(), before)
                path.write_text(original)

    def test_existing_untracked_helper_or_backup_is_preserved(self):
        for path in (self.helper, self.backup):
            with self.subTest(path=path.name):
                path.write_bytes(b"pre-existing file")
                before = self.snapshot()
                with self.assertRaisesRegex(bridge.CompatibilityError, "without a record"):
                    self.run_adapter(apply=True)
                self.assertEqual(self.snapshot(), before)
                path.unlink()

    def test_missing_installation_cannot_be_reverted(self):
        before = self.snapshot()
        with self.assertRaisesRegex(bridge.CompatibilityError, "no adapter installation record"):
            self.run_adapter(revert=True)
        self.assertEqual(self.snapshot(), before)

    def test_failed_source_replacement_cleans_up_installation_files(self):
        before = self.snapshot()
        with patch.object(bridge, "atomic_replace", side_effect=OSError("simulated disk error")):
            with self.assertRaisesRegex(OSError, "simulated disk error"):
                self.run_adapter(apply=True)
        self.assertEqual(self.snapshot(), before)


if __name__ == "__main__":
    unittest.main()
