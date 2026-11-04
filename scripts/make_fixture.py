#!/usr/bin/env python3
"""Build a small test fixture from a real radar-cam-recorder session folder.

Usage::

    python scripts/make_fixture.py SESSION_DIR OUT_DIR --frames N [--segments 0,1]
                                   [--unlisted 3] [--no-frames-csv 3]

For every selected segment (default: every segment that has a ``video/NNNN.meta.json``)
the script writes into ``OUT_DIR``:

* ``video/NNNN.meta.json``: the first N per-frame metadata objects (valid JSON, even when
  the source file is truncated because the recorder was cut off);
* ``video/NNNN.frames.csv``: the header and the first N rows, only if the source has one
  (``--no-frames-csv`` leaves it out for the given segments);
* ``video/NNNN.h264``: the leading bytes of the Annex B stream up to the start of frame
  N+1, so it begins with SPS/PPS and an IDR and holds exactly N frames;
* ``radar/NNNN.csv``: the header and the lines whose ``t_rx_ns`` lies inside the time
  window ``[first, last]`` of the kept frames' ``SensorTimestamp``.

``events.jsonl`` is copied unchanged. ``session.json`` is copied with the bulky,
irrelevant ``config`` sub-sections dropped (only ``camera``, ``radar``, ``recording`` and
``setup`` remain); ``clock_samples``, ``state``, ``devices`` and the rest are kept
verbatim. In its ``segments`` list, selected segments keep their record but ``frames`` is
set to N, ``end_boottime_ns`` to the last kept frame timestamp, and the ``drops`` and
``max_gap_ns`` counters (which no longer describe the trimmed data) are removed.
Segments that are not selected, or are named by ``--unlisted``, are left out of the list
(``--unlisted`` still writes their files, to imitate a segment missing from
``session.json``).

The source session only has to contain the first ``N`` frames' worth of each ``.h264``
(the script reads just that prefix), so it can be a partial copy of a large session.
Nothing in SESSION_DIR is modified. Only the standard library is used.

Radar-only mode::

    python scripts/make_fixture.py SESSION_DIR OUT_DIR --radar-lines 1-16,318-400

writes just ``OUT_DIR/radar/0000.csv`` (line numbers are 1-based, line 1 is the header;
``--radar-segment`` picks another segment) with those lines verbatim, original timestamps
unchanged, plus a ``README.txt`` naming the source session and slice. No video, meta or
``session.json`` is written and ``--frames`` is not used. ``tests/fixtures/radar_targets``
is ``--radar-lines 1-16,318-400`` of session s000060 (lines 2-16 are radar-configuration
command ACKs, 318-400 a run of data frames with targets, many split across reads).
"""

from __future__ import annotations

import argparse
import csv
import json
import mmap
import shutil
import sys
from pathlib import Path
from typing import Any

KEEP_CONFIG = ("camera", "radar", "recording", "setup")
_START = b"\x00\x00\x01"
_VCL_TYPES = (1, 5)


def read_meta(path: Path) -> list[dict[str, Any]]:
    """Parse the recorder's meta.json, tolerating a missing ``]`` or a torn last object."""
    text = path.read_text(encoding="utf-8", errors="replace")
    decoder = json.JSONDecoder()
    pos = text.index("[") + 1
    out: list[dict[str, Any]] = []
    while True:
        while pos < len(text) and text[pos] in " \t\r\n,":
            pos += 1
        if pos >= len(text) or text[pos] != "{":
            break
        try:
            obj, pos = decoder.raw_decode(text, pos)
        except ValueError:
            break  # torn last object
        out.append(obj)
    return out


def h264_cut_offset(path: Path, frames: int) -> int:
    """Byte offset where frame ``frames + 1`` starts in an Annex B file.

    A frame starts at its first NAL (AUD/SEI/SPS/PPS before the slice belong to it); a new
    frame is a slice NAL (type 1 or 5) with ``first_mb_in_slice == 0``.
    """
    with path.open("rb") as fh, mmap.mmap(fh.fileno(), 0, access=mmap.ACCESS_READ) as data:
        nals: list[tuple[int, int, bool]] = []  # (start offset incl. zero_byte, type, new frame)
        pos = data.find(_START)
        while pos >= 0 and pos + 4 <= len(data):
            nal_type = data[pos + 3] & 0x1F
            first_slice = nal_type in _VCL_TYPES and bool(data[pos + 4] & 0x80)
            start = pos - 1 if pos > 0 and data[pos - 1] == 0 else pos
            nals.append((start, nal_type, first_slice))
            if sum(1 for n in nals if n[2]) > frames + 1:
                break
            pos = data.find(_START, pos + 3)
    firsts = [i for i, n in enumerate(nals) if n[2]]
    if len(firsts) <= frames:
        raise SystemExit(f"{path}: fewer than {frames + 1} frames available in this file")
    i = firsts[frames]
    while i > 0 and nals[i - 1][1] not in _VCL_TYPES:
        i -= 1
    return nals[i][0]


def copy_prefix(src: Path, dst: Path, size: int) -> None:
    with src.open("rb") as fin, dst.open("wb") as fout:
        fout.write(fin.read(size))


def trim_radar(src: Path, dst: Path, lo: int, hi: int) -> int:
    kept = 0
    with (
        src.open(encoding="utf-8", errors="replace") as fin,
        dst.open("w", encoding="utf-8") as fout,
    ):
        header = fin.readline()
        fout.write(header if header.endswith("\n") else header + "\n")
        for line in fin:
            if not line.endswith("\n"):
                continue  # torn last line
            try:
                t = int(line.split(",", 1)[0])
            except ValueError:
                continue
            if lo <= t <= hi:
                fout.write(line)
                kept += 1
    return kept


def build(
    session: Path,
    out: Path,
    frames: int,
    segments: list[int] | None,
    unlisted: set[int],
    no_frames_csv: set[int],
) -> None:
    video, radar = session / "video", session / "radar"
    if segments is None:
        segments = sorted(int(p.name.split(".")[0]) for p in video.glob("[0-9]*.meta.json"))
    out_video, out_radar = out / "video", out / "radar"
    out_video.mkdir(parents=True, exist_ok=True)
    out_radar.mkdir(parents=True, exist_ok=True)

    last_ts: dict[int, int] = {}
    for idx in segments:
        stem = f"{idx:04d}"
        meta = read_meta(video / f"{stem}.meta.json")[:frames]
        if len(meta) < frames:
            raise SystemExit(f"segment {idx}: only {len(meta)} frames in meta.json")
        (out_video / f"{stem}.meta.json").write_text(json.dumps(meta, indent=1) + "\n")
        cut = h264_cut_offset(video / f"{stem}.h264", frames)
        copy_prefix(video / f"{stem}.h264", out_video / f"{stem}.h264", cut)
        src_csv = video / f"{stem}.frames.csv"
        if src_csv.exists() and idx not in no_frames_csv:
            with src_csv.open(newline="") as fin:
                rows = [r for _, r in zip(range(frames + 1), csv.reader(fin), strict=False)]
            with (out_video / f"{stem}.frames.csv").open("w", newline="") as fout:
                csv.writer(fout, lineterminator="\n").writerows(rows)
        lo, hi = meta[0]["SensorTimestamp"], meta[-1]["SensorTimestamp"]
        last_ts[idx] = hi
        if (radar / f"{stem}.csv").exists():
            n = trim_radar(radar / f"{stem}.csv", out_radar / f"{stem}.csv", lo, hi)
            print(f"segment {idx}: {frames} frames, {cut} h264 bytes, {n} radar lines")

    shutil.copyfile(session / "events.jsonl", out / "events.jsonl")
    data = json.loads((session / "session.json").read_text(encoding="utf-8"))
    if isinstance(data.get("config"), dict):
        data["config"] = {k: v for k, v in data["config"].items() if k in KEEP_CONFIG}
    listed = []
    for seg in data.get("segments", []):
        if seg["index"] in segments and seg["index"] not in unlisted:
            seg = {k: v for k, v in seg.items() if k not in ("drops", "max_gap_ns")}
            seg["frames"] = frames
            seg["end_boottime_ns"] = last_ts[seg["index"]]
            listed.append(seg)
    data["segments"] = listed
    (out / "session.json").write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")


def build_radar_only(session: Path, out: Path, spec: str, segment: int) -> None:
    lines = (session / "radar" / f"{segment:04d}.csv").read_text(encoding="utf-8").split("\n")
    picked: list[str] = []
    for part in spec.split(","):
        lo, _, hi = part.partition("-")
        first, last = int(lo), int(hi or lo)
        picked.extend(lines[first - 1 : last])
    (out / "radar").mkdir(parents=True, exist_ok=True)
    (out / "radar" / f"{segment:04d}.csv").write_text("\n".join(picked) + "\n", encoding="utf-8")
    (out / "README.txt").write_text(
        f"Radar-only fixture: radar/{segment:04d}.csv is lines {spec} (1-based, line 1 is the\n"
        f"header) of radar/{segment:04d}.csv of the real session {session.name}, verbatim, with\n"
        "original t_rx_ns. There is no session.json. Reads are fragmented, so a data frame\n"
        "often spans two lines. Built by scripts/make_fixture.py --radar-lines.\n",
        encoding="utf-8",
    )


def _indexes(text: str) -> list[int]:
    return [int(x) for x in text.split(",") if x]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0] if __doc__ else None)
    ap.add_argument("session_dir", type=Path)
    ap.add_argument("out_dir", type=Path)
    ap.add_argument("--frames", type=int, help="frames to keep per segment")
    ap.add_argument("--radar-lines", help="radar-only mode: 1-based line ranges, e.g. 1-16,318-400")
    ap.add_argument("--radar-segment", type=int, default=0, help="segment for --radar-lines")
    ap.add_argument("--segments", type=_indexes, help="comma separated segment indexes")
    ap.add_argument("--unlisted", type=_indexes, default=[], help="write but do not list")
    ap.add_argument("--no-frames-csv", type=_indexes, default=[], help="omit frames.csv")
    args = ap.parse_args(argv)
    if args.radar_lines:
        build_radar_only(args.session_dir, args.out_dir, args.radar_lines, args.radar_segment)
        return 0
    if args.frames is None:
        ap.error("--frames is required unless --radar-lines is given")
    build(
        args.session_dir,
        args.out_dir,
        args.frames,
        args.segments,
        set(args.unlisted),
        set(args.no_frames_csv),
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
