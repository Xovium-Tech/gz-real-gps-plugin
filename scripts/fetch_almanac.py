#!/usr/bin/env python3
"""Download and validate a GPS Yuma almanac before a simulation (Python stdlib only).

Source index: https://celestrak.org/GPS/almanac/Yuma/
Format: https://celestrak.org/GPS/almanac/Yuma/definition.php
The simulator never needs a network connection. Archive the .yuma and .json files
together for repeatable runs. A Yuma almanac describes GPS, not every GNSS system.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import tempfile
from urllib.error import URLError
from urllib.parse import urlparse
from urllib.request import HTTPRedirectHandler, Request, build_opener


DEFAULT_URL = "https://celestrak.org/GPS/almanac/Yuma/almanac.yuma.txt"
MAX_BYTES = 512 * 1024
GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)
WEEK_SECONDS = 604800
FIELDS = (
    "ID", "Health", "Eccentricity", "Time of Applicability(s)",
    "Orbital Inclination(rad)", "Rate of Right Ascen(r/s)",
    "SQRT(A) (m 1/2)", "Right Ascen at Week(rad)",
    "Argument of Perigee(rad)", "Mean Anom(rad)", "Af0(s)", "Af1(s/s)", "week",
)
HEADER = re.compile(r"\*+\s+Week\s+(\d+)\s+almanac for PRN-(\d+)\s+\*+", re.I)
NUMBER = re.compile(r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[EeDd][+-]?\d+)?")
# UTC effective dates of GPS-UTC increases. GPS began at UTC+0 seconds.
# Source: https://maia.usno.navy.mil/products/leap-second
# Update this table if another leap second is announced.
LEAP_DATES = tuple(datetime.fromisoformat(date).replace(tzinfo=timezone.utc) for date in (
    "1981-07-01", "1982-07-01", "1983-07-01", "1985-07-01", "1988-01-01",
    "1990-01-01", "1991-01-01", "1992-07-01", "1993-07-01", "1994-07-01",
    "1996-01-01", "1997-07-01", "1999-01-01", "2006-01-01", "2009-01-01",
    "2012-07-01", "2015-07-01", "2017-01-01",
))


def require_https(url: str) -> None:
    parsed = urlparse(url)
    if parsed.scheme.lower() != "https" or not parsed.hostname:
        raise ValueError("the almanac URL must use HTTPS")
    if parsed.username or parsed.password:
        raise ValueError("credentials in the almanac URL are not supported")


class HttpsRedirectHandler(HTTPRedirectHandler):
    """Reject an HTTPS-to-HTTP redirect before the redirected request is sent."""

    def redirect_request(self, req, fp, code, msg, headers, newurl):
        require_https(newurl)
        return super().redirect_request(req, fp, code, msg, headers, newurl)


def fetch(url: str, timeout: float = 20.0) -> tuple[bytes, str]:
    require_https(url)
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be a positive, finite number")
    request = Request(url, headers={
        "User-Agent": "gz-real-gps-plugin/almanac-fetcher",
        "Accept": "text/plain, application/octet-stream;q=0.9",
    })
    with build_opener(HttpsRedirectHandler()).open(request, timeout=timeout) as response:
        resolved_url = response.geturl()
        require_https(resolved_url)
        if "html" in response.headers.get("Content-Type", "").lower():
            raise ValueError("server returned HTML instead of a Yuma almanac")
        length = response.headers.get("Content-Length")
        if length is not None and (not length.isdigit() or int(length) > MAX_BYTES):
            raise ValueError("invalid or excessive Content-Length for the almanac")
        payload = response.read(MAX_BYTES + 1)
    if len(payload) > MAX_BYTES:
        raise ValueError(f"almanac exceeds {MAX_BYTES} bytes")
    if length is not None and len(payload) != int(length):
        raise ValueError("incomplete response: Content-Length does not match the download")
    return payload, resolved_url


def validate_yuma(payload: bytes) -> list[dict[str, float]]:
    """Require complete, unique records and finite, physically plausible elements."""
    if not payload or len(payload) > MAX_BYTES:
        raise ValueError("empty or oversized almanac")
    try:
        content = payload.decode("ascii")
    except UnicodeDecodeError as exc:
        raise ValueError("Yuma almanac must be ASCII text") from exc
    records = []
    current = None
    header_week = header_prn = None
    seen_prns = set()

    def finish_record():
        if current is None:
            return
        missing = set(FIELDS) - current.keys()
        if missing:
            raise ValueError(f"PRN {header_prn}: incomplete record; missing {', '.join(sorted(missing))}")
        for field in ("ID", "Health", "week"):
            if not current[field].is_integer():
                raise ValueError(f"PRN {header_prn}: {field} must be an integer")
        prn, week = int(current["ID"]), int(current["week"])
        if prn != header_prn or week != header_week:
            raise ValueError("record header PRN/week does not match its fields")
        if not 1 <= prn <= 32 or prn in seen_prns:
            raise ValueError(f"invalid or duplicate GPS PRN {prn}")
        if not 0 <= current["Health"] <= 63 or not 0 <= week <= 65535:
            raise ValueError(f"PRN {prn}: invalid health or GPS week")
        if not 0 <= current["Eccentricity"] < 1:
            raise ValueError(f"PRN {prn}: eccentricity must be in [0, 1)")
        if not 0 <= current["Time of Applicability(s)"] < WEEK_SECONDS:
            raise ValueError(f"PRN {prn}: time of applicability is outside one GPS week")
        if not 0 <= current["Orbital Inclination(rad)"] <= math.pi:
            raise ValueError(f"PRN {prn}: inclination must be in [0, pi]")
        if not 4000 <= current["SQRT(A) (m 1/2)"] <= 6000:
            raise ValueError(f"PRN {prn}: semimajor axis is outside the GPS orbit range")
        if abs(current["Rate of Right Ascen(r/s)"]) > 1e-5:
            raise ValueError(f"PRN {prn}: implausible right ascension rate")
        for field in ("Right Ascen at Week(rad)", "Argument of Perigee(rad)", "Mean Anom(rad)"):
            if abs(current[field]) > 2 * math.pi:
                raise ValueError(f"PRN {prn}: {field} is outside [-2*pi, 2*pi]")
        seen_prns.add(prn)
        records.append(current)

    for line_number, raw_line in enumerate(content.splitlines(), 1):
        line = raw_line.strip()
        if not line:
            continue
        match = HEADER.fullmatch(line)
        if match:
            finish_record()
            header_week, header_prn = map(int, match.groups())
            current = {}
            continue
        if current is None or ":" not in line:
            raise ValueError(f"line {line_number}: not a Yuma record (possibly an HTML error page)")
        field, raw_value = line.split(":", 1)
        field, raw_value = " ".join(field.split()), raw_value.strip()
        if field not in FIELDS or field in current:
            raise ValueError(f"line {line_number}: unknown or duplicate field {field!r}")
        if not NUMBER.fullmatch(raw_value):
            raise ValueError(f"line {line_number}: invalid numeric value for {field}")
        value = float(raw_value.replace("D", "E").replace("d", "e"))
        if not math.isfinite(value):
            raise ValueError(f"line {line_number}: non-finite {field}")
        current[field] = value
    finish_record()
    if not records:
        raise ValueError("no GPS satellite records found")
    return records


def parse_utc(value: str) -> datetime:
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("use an ISO 8601 time, e.g. 2026-10-12T12:00:00Z") from exc
    if parsed.tzinfo is None:
        raise argparse.ArgumentTypeError("the reference time must include Z or a UTC offset")
    return parsed.astimezone(timezone.utc)


def gps_utc_offset(utc: datetime) -> int:
    return sum(utc >= date for date in LEAP_DATES)


def resolve_epoch(week: int, toa: float, reference: datetime) -> tuple[int, datetime]:
    """Resolve a 10-bit GPS week to the nearest 1024-week era of reference UTC."""
    if reference.tzinfo is None:
        raise ValueError("reference time needs a timezone")
    reference_gps = (reference - GPS_EPOCH).total_seconds() + gps_utc_offset(reference)
    if reference_gps < 0:
        raise ValueError("reference time predates the GPS epoch (1980-01-06)")
    if week < 1024:
        era = max(0, math.floor((reference_gps / WEEK_SECONDS - week - toa / WEEK_SECONDS) / 1024 + 0.5))
        week += era * 1024
    gps_seconds = week * WEEK_SECONDS + toa
    # Compare thresholds in GPS time to handle the UTC offset without iteration.
    offset = sum(gps_seconds >= (date - GPS_EPOCH).total_seconds() + n
                 for n, date in enumerate(LEAP_DATES, 1))
    return week, GPS_EPOCH + timedelta(seconds=gps_seconds - offset)


def iso_utc(value: datetime) -> str:
    return value.isoformat().replace("+00:00", "Z")


def atomic_write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=f".{path.name}.", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def download(url: str, output: Path, timeout: float = 20.0,
             reference: datetime | None = None) -> dict:
    payload, resolved_url = fetch(url, timeout)
    records = validate_yuma(payload)
    fetched_at = datetime.now(timezone.utc)
    reference = reference or fetched_at
    epochs = [resolve_epoch(int(record["week"]), record["Time of Applicability(s)"], reference)
              for record in records]
    newest_epoch = max(epoch for _, epoch in epochs)
    metadata = {
        "source_url": url,
        "resolved_url": resolved_url,
        "retrieved_utc": iso_utc(fetched_at),
        "sha256": hashlib.sha256(payload).hexdigest(),
        "satellite_records": len(records),
        "healthy_satellites": sum(record["Health"] == 0 for record in records),
        "week_resolution_reference_utc": iso_utc(reference),
        "full_gps_weeks": sorted(set(week for week, _ in epochs)),
        "oldest_almanac_epoch_utc": iso_utc(min(epoch for _, epoch in epochs)),
        "newest_almanac_epoch_utc": iso_utc(newest_epoch),
        "recommended_simulation_epoch_utc": iso_utc(newest_epoch),
        "recommended_simulation_epoch_unix": newest_epoch.timestamp(),
        "gps_utc_offset_seconds": gps_utc_offset(newest_epoch),
        "age_days_at_reference": (reference - newest_epoch).total_seconds() / 86400,
        "note": "GPS almanac orbital approximation; not live receiver observations or precise ephemerides.",
    }
    # Each file is atomically replaced. The hash binds the sidecar to its data.
    atomic_write(output, payload)
    atomic_write(output.with_suffix(output.suffix + ".json"),
                 (json.dumps(metadata, indent=2) + "\n").encode("utf-8"))
    return metadata


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--url", default=DEFAULT_URL, help="HTTPS Yuma URL; use an archive URL for historical runs")
    parser.add_argument("--output", type=Path, default=Path("data/gps-latest.yuma"), help="output file (default: data/gps-latest.yuma)")
    parser.add_argument("--timeout", type=float, default=20.0, help="network timeout in seconds (default: 20)")
    parser.add_argument("--reference-utc", type=parse_utc,
                        help="date near the desired almanac to resolve GPS week rollover; defaults to now")
    args = parser.parse_args(argv)
    try:
        metadata = download(args.url, args.output, args.timeout, args.reference_utc)
    except (OSError, URLError, ValueError, OverflowError) as exc:
        print(f"Almanac download failed: {exc}", file=sys.stderr)
        return 1
    print(f"Saved {metadata['satellite_records']} GPS satellite records "
          f"({metadata['healthy_satellites']} healthy) to {args.output}")
    print(f"Source: {metadata['resolved_url']}")
    print(f"Almanac epoch UTC: {metadata['recommended_simulation_epoch_utc']}")
    print(f"Recommended simulation epoch (Unix seconds): {metadata['recommended_simulation_epoch_unix']:.3f}")
    print(f"GPS minus UTC at that epoch: {metadata['gps_utc_offset_seconds']} s")
    print(f"Age relative to reference date: {metadata['age_days_at_reference']:.2f} days")
    print(f"Provenance and checksum: {args.output}.json")
    if abs(metadata["age_days_at_reference"]) > 14:
        print("Warning: almanac is more than 14 days from the reference date; "
              "use its epoch for replay or obtain a fresher almanac.", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
