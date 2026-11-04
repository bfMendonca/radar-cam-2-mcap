#!/usr/bin/env python3
"""Verify a radar-cam-2-mcap output file (development tool, not part of the package).

Usage: ``verify_output.py OUTPUT.mcap [--session SESSION_DIR] [--allow-prefix]``

Checks, each reported as PASS/FAIL/INFO:

* per-topic message count and first/last ``log_time`` (ISO UTC and ns);
* ``log_time`` is non-decreasing in file order, and file order equals ``log_time`` order
  (``iter_messages(log_time_order=True)`` compared with the unordered iteration);
* ``header.stamp`` equals ``log_time`` on every message of the image, radar scan, radar status
  and camera metadata topics (decoded with ``mcap_ros2``);
* the file-level metadata records are printed;
* with ``--session``: image count equals the number of ``SensorTimestamp`` objects over the
  session's ``video/*.meta.json``. With ``--allow-prefix`` a difference is printed instead of
  failing (the converter drops frames beyond the common prefix of h264 and meta.json).

Exit status 1 when any check failed, else 0.
"""

from __future__ import annotations

import argparse
import sys
from datetime import UTC, datetime
from pathlib import Path

from mcap.reader import make_reader
from mcap_ros2.reader import read_ros2_messages

from radar_cam_2_mcap.video import read_frame_meta

STAMPED = (
    "/camera/image/compressed",
    "/radar/scan",
    "/radar/status",
    "/camera/metadata",
)
IMAGE_TOPIC = "/camera/image/compressed"


def iso(ns: int) -> str:
    sec, rem = divmod(ns, 1_000_000_000)
    return datetime.fromtimestamp(sec, UTC).strftime("%Y-%m-%dT%H:%M:%S") + f".{rem:09d}Z"


class Report:
    def __init__(self) -> None:
        self.failed = 0

    def check(self, ok: bool, label: str, detail: str = "") -> None:
        if not ok:
            self.failed += 1
        print(f"[{'PASS' if ok else 'FAIL'}] {label}" + (f": {detail}" if detail else ""))

    @staticmethod
    def info(label: str, detail: str = "") -> None:
        print(f"[INFO] {label}" + (f": {detail}" if detail else ""))


def topic_summary(path: Path, rep: Report) -> dict[str, int]:
    """Counts, first/last log_time, monotonicity in file order and vs log-time order."""
    counts: dict[str, int] = {}
    first: dict[str, int] = {}
    last: dict[str, int] = {}
    file_order: list[tuple[int, int]] = []  # (log_time, channel_id) in file order
    prev = -1
    inversions = 0
    with path.open("rb") as fh:
        reader = make_reader(fh)
        for _schema, channel, message in reader.iter_messages():
            t = channel.topic
            counts[t] = counts.get(t, 0) + 1
            first.setdefault(t, message.log_time)
            last[t] = message.log_time
            if message.log_time < prev:
                inversions += 1
            prev = max(prev, message.log_time)
            file_order.append((message.log_time, message.sequence))
    print("Topics:")
    for t in sorted(counts):
        print(
            f"  {t}: {counts[t]} msgs, first {iso(first[t])} ({first[t]}),"
            f" last {iso(last[t])} ({last[t]})"
        )
    rep.check(inversions == 0, "log_time non-decreasing in file order", f"{inversions} inversions")

    # Compare against log_time order: the sequence of log times must be identical.
    with path.open("rb") as fh:
        sorted_times = [
            m.log_time for _s, _c, m in make_reader(fh).iter_messages(log_time_order=True)
        ]
    same = sorted_times == [t for t, _ in file_order]
    rep.check(
        same and len(sorted_times) == len(file_order),
        "file order equals log_time order (iter_messages(log_time_order=True))",
        f"{len(file_order)} vs {len(sorted_times)} msgs",
    )
    return counts


def check_stamps(path: Path, rep: Report) -> None:
    bad: dict[str, int] = {}
    seen: dict[str, int] = {}
    worst: dict[str, int] = {}
    for item in read_ros2_messages(str(path), topics=list(STAMPED)):
        topic = item.channel.topic
        stamp = item.ros_msg.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        seen[topic] = seen.get(topic, 0) + 1
        diff = abs(stamp_ns - item.log_time_ns)
        if diff:
            bad[topic] = bad.get(topic, 0) + 1
            worst[topic] = max(worst.get(topic, 0), diff)
    for topic in STAMPED:
        n = seen.get(topic, 0)
        if n == 0:
            rep.info(f"header.stamp == log_time on {topic}", "no messages")
            continue
        rep.check(
            bad.get(topic, 0) == 0,
            f"header.stamp == log_time on {topic}",
            f"{bad.get(topic, 0)} of {n} differ, worst {worst.get(topic, 0)} ns",
        )


def print_metadata(path: Path) -> dict[str, dict[str, str]]:
    print("File metadata:")
    out: dict[str, dict[str, str]] = {}
    with path.open("rb") as fh:
        for md in make_reader(fh).iter_metadata():
            out[md.name] = dict(md.metadata)
    for name, values in out.items():
        print(f"  [{name}]")
        for k, v in values.items():
            print(f"    {k} = {v}")
    return out


def check_session(
    path: Path, session: Path, counts: dict[str, int], allow_prefix: bool, rep: Report
) -> None:
    total = 0
    for meta in sorted((session / "video").glob("*.meta.json")):
        n = len(read_frame_meta(meta))
        total += n
        rep.info(f"{meta.name}", f"{n} SensorTimestamp objects")
    images = counts.get(IMAGE_TOPIC, 0)
    if allow_prefix:
        rep.info(
            "image count vs meta.json objects (--allow-prefix)",
            f"images {images}, meta objects {total}, difference {total - images}",
        )
    else:
        rep.check(
            images == total, "image count == sum of meta.json objects", f"{images} vs {total}"
        )


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("output", type=Path, help="MCAP file to verify")
    ap.add_argument("--session", type=Path, help="source session folder for the image count check")
    ap.add_argument(
        "--allow-prefix",
        action="store_true",
        help="the conversion used --allow-prefix: print the image/meta difference, do not fail",
    )
    args = ap.parse_args(argv)

    rep = Report()
    print(f"File: {args.output} ({args.output.stat().st_size} bytes)")
    counts = topic_summary(args.output, rep)
    check_stamps(args.output, rep)
    print_metadata(args.output)
    if args.session is not None:
        check_session(args.output, args.session, counts, args.allow_prefix, rep)
    print("RESULT:", "FAILED" if rep.failed else "OK", f"({rep.failed} failed checks)")
    return 1 if rep.failed else 0


if __name__ == "__main__":
    sys.exit(main())
