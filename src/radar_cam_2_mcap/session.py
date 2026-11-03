"""Tolerant reader for a radar-cam-recorder session folder (recorder ``format_version`` 1).

Only metadata and file discovery live here; frames and radar data are decoded elsewhere.
Missing optional keys never crash; an unknown ``format_version`` is refused.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

__all__ = [
    "ClockAnchor",
    "SegmentFiles",
    "SessionError",
    "SessionInfo",
    "load_events",
    "load_session",
]

SUPPORTED_FORMAT_VERSION = 1

_VIDEO_FILE = re.compile(r"(\d+)\.(h264|meta\.json)")
_RADAR_FILE = re.compile(r"(\d+)\.csv")


class SessionError(Exception):
    """Raised when a session folder cannot be used."""


@dataclass(frozen=True)
class ClockAnchor:
    label: str
    boottime_ns: int
    realtime_ns: int


@dataclass(frozen=True)
class SegmentFiles:
    index: int
    h264: Path | None
    meta: Path | None
    radar: Path | None


@dataclass
class SessionInfo:
    path: Path
    session_id: str
    state: str
    stop_reason: str | None
    boot_id: str | None
    ntp_synced: bool
    wall_clock_source: str | None
    anchors: list[ClockAnchor]
    recorder_config: dict[str, Any]
    devices: dict[str, Any]
    segments: list[SegmentFiles]
    listed_segment_indexes: list[int]
    warnings: list[str] = field(default_factory=list)


def _optional_str(value: Any) -> str | None:
    return value if isinstance(value, str) else None


def _dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _read_anchors(raw: Any, warnings: list[str]) -> list[ClockAnchor]:
    anchors: list[ClockAnchor] = []
    for sample in raw if isinstance(raw, list) else []:
        try:
            anchors.append(
                ClockAnchor(
                    label=str(sample["label"]),
                    boottime_ns=int(sample["boottime_ns"]),
                    realtime_ns=int(sample["realtime_ns"]),
                )
            )
        except (KeyError, TypeError, ValueError):
            warnings.append(f"ignored malformed clock sample: {sample!r}")
    return anchors


def _listed_indexes(raw: Any) -> list[int]:
    indexes: list[int] = []
    for seg in raw if isinstance(raw, list) else []:
        if isinstance(seg, dict) and isinstance(seg.get("index"), int):
            indexes.append(seg["index"])
    return sorted(set(indexes))


def _discover(directory: Path, pattern: re.Pattern[str]) -> dict[int, dict[str, Path]]:
    found: dict[int, dict[str, Path]] = {}
    if directory.is_dir():
        for file in directory.iterdir():
            match = pattern.fullmatch(file.name)
            if match and file.is_file():
                kind = match.group(2) if match.lastindex == 2 else "csv"
                found.setdefault(int(match.group(1)), {})[kind] = file
    return found


def load_session(path: Path) -> SessionInfo:
    """Load ``path/session.json`` and discover its segments from files and the listing."""
    path = Path(path)
    session_file = path / "session.json"
    try:
        data = json.loads(session_file.read_text(encoding="utf-8"))
    except OSError as exc:
        raise SessionError(f"cannot read {session_file}: {exc}") from exc
    except ValueError as exc:
        raise SessionError(f"{session_file} is not valid JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise SessionError(f"{session_file} is not a JSON object")
    version = data.get("format_version")
    if version != SUPPORTED_FORMAT_VERSION:
        raise SessionError(
            f"unsupported session format_version {version!r} "
            f"(this converter reads version {SUPPORTED_FORMAT_VERSION})"
        )

    warnings: list[str] = []
    state = _optional_str(data.get("state")) or "unknown"
    if state == "in_progress":
        warnings.append("session is still in progress (recorder did not close it)")
    elif state == "interrupted":
        warnings.append("session was interrupted; the last segment may be truncated")

    listed = _listed_indexes(data.get("segments"))
    video = _discover(path / "video", _VIDEO_FILE)
    radar = _discover(path / "radar", _RADAR_FILE)

    segments: list[SegmentFiles] = []
    for index in sorted(set(video) | set(radar) | set(listed)):
        files = video.get(index, {})
        seg = SegmentFiles(
            index=index,
            h264=files.get("h264"),
            meta=files.get("meta.json"),
            radar=radar.get(index, {}).get("csv"),
        )
        if seg.meta is None or (seg.h264 is None and seg.radar is None):
            warnings.append(f"segment {index} dropped: needs meta.json plus h264 or radar csv")
            continue
        if index not in listed:
            warnings.append(f"segment {index} is on disk but not listed in session.json")
        segments.append(seg)

    return SessionInfo(
        path=path,
        session_id=_optional_str(data.get("session_id")) or path.name,
        state=state,
        stop_reason=_optional_str(data.get("stop_reason")),
        boot_id=_optional_str(data.get("boot_id")),
        ntp_synced=data.get("ntp_synced") is True,
        wall_clock_source=_optional_str(data.get("wall_clock_source")),
        anchors=_read_anchors(data.get("clock_samples"), warnings),
        recorder_config=_dict(data.get("config")),
        devices=_dict(data.get("devices")),
        segments=segments,
        listed_segment_indexes=listed,
        warnings=warnings,
    )


def load_events(path: Path) -> list[dict[str, Any]]:
    """Read ``events.jsonl``; a missing file gives ``[]`` and unparsable lines are skipped."""
    try:
        text = Path(path).read_text(encoding="utf-8", errors="replace")
    except FileNotFoundError:
        return []
    events: list[dict[str, Any]] = []
    for line in text.splitlines():
        try:
            obj = json.loads(line)
        except ValueError:
            continue  # blank or partial (torn) line
        if isinstance(obj, dict):
            events.append(obj)
    return events
