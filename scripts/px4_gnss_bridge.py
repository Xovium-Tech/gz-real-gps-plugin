#!/usr/bin/env python3
"""Preview/apply/revert the checked, additive PX4 Gazebo GNSS adapter.

The default mode is a read-only compatibility check and unified diff to stdout.
No network access, git checkout, reset, or changes outside the chosen PX4 tree.
"""

from __future__ import annotations

import argparse
import difflib
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parent.parent
ASSETS = ROOT / "integration" / "px4"
BRIDGE = Path("src/modules/simulation/gz_bridge/GZBridge.cpp")
INCLUDE = '#include "GZRealGps.hpp"\n'
BEGIN = "GZ_REAL_GPS_ADAPTER_V1_BEGIN"
END = "GZ_REAL_GPS_ADAPTER_V1_END"
REQUIRED_FIELDS = (
    "timestamp timestamp_sample device_id latitude_deg longitude_deg "
    "altitude_msl_m altitude_ellipsoid_m eph epv hdop vdop s_variance_m_s "
    "c_variance_rad satellites_used fix_type vel_n_m_s vel_e_m_s vel_d_m_s "
    "vel_m_s cog_rad vel_ned_valid time_utc_usec jamming_state spoofing_state "
    "heading heading_offset"
).split()


class CompatibilityError(RuntimeError):
    pass


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def make_adapter(source: str, sensor_message: str, bridge_header: str) -> str:
    """Check the v1.16-style API before making exactly two additive insertions."""
    if BEGIN in source or END in source or INCLUDE in source:
        raise CompatibilityError("adapter markers already exist; inspect installation state")
    if "\r" in source:
        raise CompatibilityError("CRLF source is unsupported; refusing an implicit newline rewrite")
    message_fields = set(re.findall(r"^\w+\s+(\w+)\s*(?:#.*)?$", sensor_message, re.MULTILINE))
    missing = sorted(set(REQUIRED_FIELDS) - message_fields)
    if missing:
        raise CompatibilityError("unsupported SensorGps.msg; missing fields: " + ", ".join(missing))
    for member in ("_sensor_gps_pub", "_gpos_ground_truth_pub"):
        if member not in bridge_header:
            raise CompatibilityError(f"unsupported GZBridge.hpp; missing {member}")
    signature = r"void GZBridge::navSatCallback\(const gz::msgs::NavSat &msg\)\n\{\n"
    matches = list(re.finditer(signature, source))
    if len(matches) != 1:
        raise CompatibilityError("expected one v1.16-style navSatCallback(const gz::msgs::NavSat &msg); "
                                 "PX4 v1.15 and other callback APIs need a manual port")
    match = matches[0]
    end = source.find("\n}\n", match.end())
    if end < 0:
        raise CompatibilityError("cannot locate the navSatCallback closing brace")
    body = source[match.end():end]
    for anchor in ("const uint64_t timestamp = hrt_absolute_time();", "addGpsNoise(",
                   "sensor_gps_s sensor_gps", "_sensor_gps_pub.publish(sensor_gps);",
                   "_gpos_ground_truth_pub.publish(gps_truth);"):
        if body.count(anchor) != 1:
            raise CompatibilityError(f"unsupported navSatCallback structure: expected one {anchor!r}")
    if "pthread_mutex" in body or "_failure_config" in body:
        raise CompatibilityError("callback has locking or newer PX4 failure-injection logic; "
                                 "manual integration is required to preserve those semantics")
    if source.count('#include "GZBridge.hpp"\n') != 1:
        raise CompatibilityError("expected one GZBridge.hpp include")
    snippet = (ASSETS / "navsat_callback.inc").read_text()
    modified = source[:match.end()] + snippet + source[match.end():]
    return modified.replace('#include "GZBridge.hpp"\n', '#include "GZBridge.hpp"\n' + INCLUDE, 1)


def atomic_replace(path: Path, data: bytes, mode: int) -> None:
    handle, temporary = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    try:
        with os.fdopen(handle, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, mode)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def new_file(path: Path, data: bytes) -> None:
    with path.open("xb") as stream:
        stream.write(data)


def run(args: argparse.Namespace) -> int:
    px4 = args.px4.resolve()
    source_file = px4 / BRIDGE
    helper_file = source_file.with_name("GZRealGps.hpp")
    backup_file = source_file.with_name("GZBridge.cpp.gz-real-gps.backup")
    state_file = source_file.with_name(".gz-real-gps-adapter.json")
    original = source_file.read_bytes()
    mode = stat.S_IMODE(source_file.stat().st_mode)
    helper = (ASSETS / "GZRealGps.hpp").read_bytes()
    try:
        version = subprocess.run(["git", "-C", str(px4), "describe", "--always", "--dirty"],
                                 capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        version = "source snapshot (no git version)"
    print(f"PX4: {version}; GZBridge.cpp SHA256 {digest(original)}", file=sys.stderr)

    if state_file.exists():
        state = json.loads(state_file.read_text())
        if (state.get("adapter_version") != 1 or digest(original) != state.get("patched_sha256")
                or not helper_file.exists() or digest(helper_file.read_bytes()) != state.get("helper_sha256")
                or not backup_file.exists() or digest(backup_file.read_bytes()) != state.get("original_sha256")):
            raise CompatibilityError("installed adapter or its backup changed; refusing to overwrite edits. "
                                     "Review the marked callback block and backup manually")
        if args.revert:
            atomic_replace(source_file, backup_file.read_bytes(), mode)
            helper_file.unlink()
            backup_file.unlink()
            state_file.unlink()
            print("Restored the exact pre-adapter GZBridge.cpp.", file=sys.stderr)
        else:
            if digest(helper) != state.get("helper_sha256"):
                raise CompatibilityError("installed helper differs from this plugin version; revert before applying again")
            print("Adapter is already installed and verified; no changes.", file=sys.stderr)
        return 0

    if args.revert:
        raise CompatibilityError("no adapter installation record; nothing was reverted")
    if helper_file.exists() or backup_file.exists():
        raise CompatibilityError("adapter helper/backup already exists without a record; inspect it before continuing")
    modified = make_adapter(original.decode("utf-8"), (px4 / "msg/SensorGps.msg").read_text(),
                            source_file.with_name("GZBridge.hpp").read_text()).encode("utf-8")
    if not args.apply:
        sys.stdout.writelines(difflib.unified_diff(original.decode().splitlines(True),
                              modified.decode().splitlines(True), fromfile="a/" + str(BRIDGE),
                              tofile="b/" + str(BRIDGE)))
        sys.stdout.writelines(difflib.unified_diff([], helper.decode().splitlines(True),
                              fromfile="/dev/null", tofile="b/" + str(BRIDGE.with_name("GZRealGps.hpp"))))
        print("Compatible. Preview only; use --apply to install this adapter.", file=sys.stderr)
        return 0

    state = {"adapter_version": 1, "px4_version": version, "original_sha256": digest(original),
             "patched_sha256": digest(modified), "helper_sha256": digest(helper)}
    created = []
    try:
        for path, data in ((backup_file, original), (helper_file, helper),
                           (state_file, (json.dumps(state, indent=2) + "\n").encode())):
            new_file(path, data)
            created.append(path)
        if source_file.read_bytes() != original:
            raise CompatibilityError("GZBridge.cpp changed concurrently; adapter was not applied")
        atomic_replace(source_file, modified, mode)
    except BaseException:
        for path in reversed(created):
            path.unlink()
        raise
    print("Applied adapter; original source backed up. Rebuild PX4 SITL before running.", file=sys.stderr)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("px4", type=Path, help="PX4-Autopilot checkout directory")
    operation = parser.add_mutually_exclusive_group()
    operation.add_argument("--check", action="store_true", help="compatibility check and diff (default)")
    operation.add_argument("--apply", action="store_true", help="install adapter after all compatibility checks")
    operation.add_argument("--revert", action="store_true", help="restore backup if installed files are unchanged")
    args = parser.parse_args()
    try:
        return run(args)
    except (OSError, UnicodeError, ValueError, CompatibilityError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
