"""Offline regression tests for downloads, validation and almanac provenance."""

from datetime import datetime, timezone
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch
from urllib.request import Request


SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "fetch_almanac.py"
SPEC = importlib.util.spec_from_file_location("fetch_almanac", SCRIPT)
almanac = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(almanac)


def record(prn=1, health=0, week=0, toa=0):
    return (f"""******** Week {week} almanac for PRN-{prn:02d} ********
ID: {prn}
Health: {health}
Eccentricity: 1.25D-02
Time of Applicability(s): {toa}
Orbital Inclination(rad): 0.96
Rate of Right Ascen(r/s): -8.0e-9
SQRT(A) (m 1/2): 5153.6
Right Ascen at Week(rad): -1.0
Argument of Perigee(rad): 2.0
Mean Anom(rad): 0.5
Af0(s): 0.0001
Af1(s/s): 0
week: {week}
""").encode("ascii")


class DownloadTests(unittest.TestCase):
    def response(self, payload, headers=None, url="https://example.org/archive.yuma"):
        response = Mock()
        response.__enter__ = Mock(return_value=response)
        response.__exit__ = Mock(return_value=False)
        response.headers = headers or {}
        response.geturl.return_value = url
        response.read.return_value = payload
        return response

    def test_fetch_preserves_payload_and_resolved_url(self):
        payload = record()
        response = self.response(payload, {"Content-Length": str(len(payload))})
        with patch.object(almanac, "build_opener") as opener:
            opener.return_value.open.return_value = response
            data, url = almanac.fetch("https://example.org/latest.yuma", 7)
        self.assertEqual(data, payload)
        self.assertEqual(url, "https://example.org/archive.yuma")
        response.read.assert_called_once_with(almanac.MAX_BYTES + 1)
        self.assertEqual(opener.return_value.open.call_args.kwargs["timeout"], 7)

    def test_insecure_urls_and_invalid_timeout_never_open_network(self):
        cases = [(url, 20) for url in (
            "http://example.org/a", "file:///tmp/a", "https:///a",
            "https://user:password@example.org/a",
        )] + [("https://example.org/a", timeout) for timeout in (0, -1, float("inf"), float("nan"))]
        with patch.object(almanac, "build_opener") as opener:
            for url, timeout in cases:
                with self.subTest(url=url, timeout=timeout), self.assertRaises(ValueError):
                    almanac.fetch(url, timeout)
        opener.assert_not_called()

    def test_redirect_handler_rejects_http_before_following_it(self):
        with self.assertRaisesRegex(ValueError, "HTTPS"):
            almanac.HttpsRedirectHandler().redirect_request(
                Request("https://example.org/a"), None, 302, "Found", {}, "http://example.org/b")

    def test_rejects_error_pages_truncated_and_oversized_downloads(self):
        cases = (
            (b"<html>error</html>", {"Content-Type": "text/html"}, "https://example.org/a"),
            (record(), {"Content-Length": "invalid"}, "https://example.org/a"),
            (record(), {"Content-Length": str(almanac.MAX_BYTES + 1)}, "https://example.org/a"),
            (record(), {"Content-Length": "1"}, "https://example.org/a"),
            (b"x" * (almanac.MAX_BYTES + 1), {}, "https://example.org/a"),
            (record(), {}, "http://example.org/a"),
        )
        for payload, headers, url in cases:
            with self.subTest(headers=headers, url=url, size=len(payload)):
                with patch.object(almanac, "build_opener") as opener:
                    opener.return_value.open.return_value = self.response(payload, headers, url)
                    with self.assertRaises(ValueError):
                        almanac.fetch("https://example.org/a")

    def test_download_writes_exact_payload_and_reproducible_provenance(self):
        payload = record() + record(prn=2, health=1, toa=86400)
        reference = datetime(2019, 4, 8, tzinfo=timezone.utc)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "data" / "gps.yuma"
            with patch.object(almanac, "fetch", return_value=(payload, "https://example.org/archive")):
                metadata = almanac.download("https://example.org/latest", output, reference=reference)
            self.assertEqual(output.read_bytes(), payload)
            self.assertEqual(json.loads(output.with_suffix(".yuma.json").read_text()), metadata)
            self.assertEqual(metadata["source_url"], "https://example.org/latest")
            self.assertEqual(metadata["resolved_url"], "https://example.org/archive")
            self.assertEqual(metadata["sha256"], hashlib.sha256(payload).hexdigest())
            self.assertEqual(metadata["satellite_records"], 2)
            self.assertEqual(metadata["healthy_satellites"], 1)
            self.assertEqual(metadata["full_gps_weeks"], [2048])
            self.assertEqual(metadata["oldest_almanac_epoch_utc"], "2019-04-06T23:59:42Z")
            self.assertEqual(metadata["recommended_simulation_epoch_utc"], "2019-04-07T23:59:42Z")
            self.assertEqual(metadata["gps_utc_offset_seconds"], 18)
            self.assertAlmostEqual(metadata["age_days_at_reference"], 18 / 86400)
            self.assertEqual(sorted(path.name for path in output.parent.iterdir()), ["gps.yuma", "gps.yuma.json"])

    def test_invalid_download_preserves_existing_files(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "gps.yuma"
            sidecar = output.with_suffix(".yuma.json")
            output.write_bytes(b"previous almanac")
            sidecar.write_bytes(b"previous metadata")
            with patch.object(almanac, "fetch", return_value=(b"<html>error</html>", "https://example.org/a")):
                with self.assertRaises(ValueError):
                    almanac.download("https://example.org/a", output)
            self.assertEqual(output.read_bytes(), b"previous almanac")
            self.assertEqual(sidecar.read_bytes(), b"previous metadata")

    def test_cli_reports_download_failure(self):
        with patch.object(almanac, "download", side_effect=ValueError("bad almanac")):
            with patch("sys.stderr", new_callable=io.StringIO) as stderr:
                self.assertEqual(almanac.main([]), 1)
        self.assertIn("Almanac download failed: bad almanac", stderr.getvalue())


class ValidationTests(unittest.TestCase):
    def test_accepts_fortran_exponents_and_multiple_unique_records(self):
        records = almanac.validate_yuma(record() + record(prn=32, health=63))
        self.assertEqual(len(records), 2)
        self.assertEqual(records[0]["Eccentricity"], 0.0125)
        self.assertEqual(records[1]["ID"], 32)

    def test_rejects_missing_duplicate_unknown_and_mismatched_fields(self):
        payload = record()
        cases = (
            payload.replace(b"Af1(s/s): 0\n", b""),
            payload + b"Af1(s/s): 0\n",
            payload + b"unknown: 1\n",
            payload.replace(b"ID: 1\n", b"ID: 2\n"),
            payload.replace(b"week: 0\n", b"week: 1\n"),
            payload + payload,
            record(prn=0),
            record(prn=33),
            b"",
            b"<html>error</html>",
            b"\xff",
        )
        for payload in cases:
            with self.subTest(payload=payload[:50]), self.assertRaises(ValueError):
                almanac.validate_yuma(payload)

    def test_rejects_invalid_or_nonphysical_numbers(self):
        cases = {
            b"Health: 0": (b"Health: -1", b"Health: 64", b"Health: 0.5"),
            b"Eccentricity: 1.25D-02": (b"Eccentricity: -0.1", b"Eccentricity: 1", b"Eccentricity: nan"),
            b"Time of Applicability(s): 0": (b"Time of Applicability(s): -1", b"Time of Applicability(s): 604800"),
            b"Orbital Inclination(rad): 0.96": (b"Orbital Inclination(rad): -1", b"Orbital Inclination(rad): 4"),
            b"SQRT(A) (m 1/2): 5153.6": (b"SQRT(A) (m 1/2): 3999", b"SQRT(A) (m 1/2): 6001"),
            b"Rate of Right Ascen(r/s): -8.0e-9": (b"Rate of Right Ascen(r/s): 0.1",),
            b"Mean Anom(rad): 0.5": (b"Mean Anom(rad): 7",),
            b"Af0(s): 0.0001": (b"Af0(s): 1e999", b"Af0(s): 1.0 trailing text"),
        }
        for original, replacements in cases.items():
            for replacement in replacements:
                with self.subTest(value=replacement), self.assertRaises(ValueError):
                    almanac.validate_yuma(record().replace(original, replacement))

    def test_week_rollover_is_resolved_near_reference_and_full_weeks_stay_fixed(self):
        reference = datetime(2019, 4, 7, tzinfo=timezone.utc)
        expected = datetime(2019, 4, 6, 23, 59, 42, tzinfo=timezone.utc)
        self.assertEqual(almanac.resolve_epoch(0, 0, reference), (2048, expected))
        self.assertEqual(almanac.resolve_epoch(2048, 0, reference), (2048, expected))
        self.assertEqual(almanac.resolve_epoch(0, 0, almanac.GPS_EPOCH), (0, almanac.GPS_EPOCH))
        with self.assertRaises(ValueError):
            almanac.resolve_epoch(0, 0, datetime(2019, 4, 7))
        with self.assertRaises(ValueError):
            almanac.resolve_epoch(0, 0, datetime(1979, 1, 1, tzinfo=timezone.utc))


if __name__ == "__main__":
    unittest.main()
