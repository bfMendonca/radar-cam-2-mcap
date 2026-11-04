"""Single-session conversion: session folder to one MCAP (or one MCAP per segment).

Streams (video, radar, static) are generators merged by ``log_ns`` with ``heapq.merge``, so
memory stays bounded by roughly one frame per stream. ``heapq.merge`` assumes every input is
sorted; each stream is sorted as long as the timeline is monotonic. If anchors make a stream's
log time go backwards the messages are NOT reordered or altered: the inversions are counted per
topic and reported as a warning, and the file is then not strictly time-ordered.

``/events`` carries every ``events.jsonl`` entry (merged by its mapped boot time) plus converter
findings. Findings known before streaming (session warnings) are stamped at the file's first
message time, right after the static messages; findings known only after streaming (video,
radar and cross-check results, segment gaps, out-of-order counts) are stamped at the file's last
message time and written last, so ``log_time`` stays non-decreasing. Every finding is also in
the ``warnings`` metadata. With ``per_segment`` each recorder event goes to the file whose
segment span contains it (before the first span: first file; after: last file), segment-level
findings to the file they concern and session-level ones to the first file.

Segment gap labels use the actual segment indexes of consecutive segments that have frames,
so a missing or frameless segment gives e.g. ``0->2``. INFO-level findings (gaps under 10 s,
recovered radar frames, trailing bytes alone) go to ``ConvertResult.notes`` and the ``notes``
metadata; ``warnings`` holds WARN-level findings only.

``/camera/camera_info`` is written once per file (at the time of the file's first message),
not repeated with every frame. Same for ``/tf_static``.
"""

from __future__ import annotations

import heapq
import json
import os
import tempfile
from collections.abc import Iterable, Iterator
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Literal

import ld2451

import radar_cam_2_mcap
from radar_cam_2_mcap import mapping
from radar_cam_2_mcap.config import Config
from radar_cam_2_mcap.radar import (
    RadarStats,
    RecoveringFrameDecoder,
    flush_trailing,
    iter_radar_reports,
)
from radar_cam_2_mcap.session import SegmentFiles, SessionInfo, load_events, load_session
from radar_cam_2_mcap.sink import McapSink
from radar_cam_2_mcap.timebase import Timeline, crosscheck, time_quality
from radar_cam_2_mcap.video import iter_video_frames, read_frame_meta

__all__ = [
    "ConvertOptions",
    "ConvertResult",
    "OutputExistsError",
    "convert_session",
    "output_paths",
    "temp_glob",
]

# Merge item: (log_ns, (rank, seq), topic, datatype, message). rank orders equal timestamps:
# static 0 (pre-stream findings follow its two messages), video 1, radar 2, events 3.
_Item = tuple[int, tuple[int, int], str, str, dict[str, Any]]

_RANK_STATIC, _RANK_VIDEO, _RANK_RADAR, _RANK_EVENT = 0, 1, 2, 3
_INFO, _WARN = 20, 30
_GAP_WARN_NS = 10_000_000_000
# Cross-check sampling: every 25th frame plus the first and last of each segment.
_SAMPLE_STRIDE = 25
_RADAR_PROBLEMS = (
    "framing_errors",
    "bad_payloads",
    "partial_lines",
    "garbled_lines",
    "trailing_bytes",
    "bytes_discarded",
)
_Finding = tuple[int, str]  # (level, text)

_UNSYNCED = (
    "session time is unsynced: wall-clock timestamps are only as good as the Pi's clock "
    "at recording time"
)


class OutputExistsError(Exception):
    """The output file exists and overwriting was not requested."""


@dataclass
class ConvertOptions:
    overwrite: bool = False
    per_segment: bool = False
    allow_prefix: bool = False
    time_base: Literal["wall", "boot"] | None = None


@dataclass
class ConvertResult:
    outputs: list[Path] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)  # INFO-level findings (not warnings)


@dataclass
class _SegStats:
    index: int
    listed: bool
    session_json: dict[str, Any]
    frames: int = 0
    radar_reports: int = 0
    first_boot_ns: int | None = None
    last_boot_ns: int | None = None

    def touch(self, boot_ns: int) -> None:
        if self.first_boot_ns is None:
            self.first_boot_ns = boot_ns
        self.last_boot_ns = boot_ns


def _listing(session: SessionInfo) -> dict[int, dict[str, Any]]:
    """Per-segment entries of session.json (drops, max_gap, ...), tolerant of damage."""
    try:
        data = json.loads((session.path / "session.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    out: dict[int, dict[str, Any]] = {}
    for seg in data.get("segments", []) if isinstance(data, dict) else []:
        if isinstance(seg, dict) and isinstance(seg.get("index"), int):
            out[seg["index"]] = seg
    return out


def _json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, ensure_ascii=False)


class _Run:
    """State shared by every file of one conversion."""

    def __init__(
        self, session: SessionInfo, cfg: Config, opts: ConvertOptions, timeline: Timeline
    ) -> None:
        self.session = session
        self.cfg = cfg
        self.opts = opts
        self.timeline = timeline
        self.listing = _listing(session)
        self.decoder = RecoveringFrameDecoder(recover=cfg.radar.recover_short_frames)
        self.video_warnings: list[str] = []
        self.out_of_order: dict[str, int] = {}
        self.quality = time_quality(session)
        self.findings: list[_Finding] = []  # converter findings beyond the run-level warning lists
        self.events: list[dict[str, Any]] = []
        self.event_warnings: list[str] = []
        for ev in load_events(session.path / "events.jsonl"):
            if isinstance(ev.get("boottime_ns"), int) and not isinstance(ev["boottime_ns"], bool):
                self.events.append(ev)
            else:
                self.event_warnings.append(
                    f"skipped events.jsonl entry without boottime_ns: {ev.get('reason', ev)!r}"
                )
        self.events.sort(key=lambda e: e["boottime_ns"])  # stable; normally already ordered
        self.prev_seg_last: tuple[int, int] | None = None  # (index, last sensor ts)
        self.file_findings: list[_Finding] = []
        self.samples: list[tuple[int, int]] = []
        self._seen = {"video": 0, "timeline": 0}
        self._size_warned = False
        self._seen_ooo: dict[str, int] = {}

    def warnings(self) -> list[str]:
        out = [
            *self.session.warnings,
            *self.event_warnings,
            *self.timeline.warnings,
            *self.video_warnings,
        ]
        out += [
            f"{topic}: {n} messages out of order within stream"
            for topic, n in self.out_of_order.items()
        ]
        if self.quality == "unsynced":
            out.append(_UNSYNCED)
        return [*out, *(t for lvl, t in self.findings if lvl >= _WARN)]

    def notes(self) -> list[str]:
        return [t for lvl, t in self.findings if lvl < _WARN]

    def segment_span(self, seg: SegmentFiles) -> tuple[int, int] | None:
        """Boot-time span of a segment: first/last frame stamps, else first/last radar line."""
        try:
            if seg.h264 is not None and seg.meta is not None:
                frames = read_frame_meta(seg.meta)
                if frames:
                    return frames[0].sensor_ts_ns, frames[-1].sensor_ts_ns
            if seg.radar is not None:
                stamps = []
                for line in seg.radar.read_text(encoding="utf-8", errors="replace").splitlines():
                    head = line.split(",", 1)[0]
                    if head.isdigit():
                        stamps.append(int(head))
                if stamps:
                    return stamps[0], stamps[-1]
        except (OSError, ValueError):
            pass
        return None

    def assign_events(self, groups: list[list[SegmentFiles]]) -> list[list[dict[str, Any]]]:
        """Split recorder events over files by segment span (one group: all of them)."""
        out: list[list[dict[str, Any]]] = [[] for _ in groups]
        if len(groups) == 1:
            out[0] = list(self.events)
            return out
        starts: list[tuple[int, int]] = []  # (span start, file number)
        for i, group in enumerate(groups):
            spans = [sp for sp in (self.segment_span(g) for g in group) if sp is not None]
            if spans:
                starts.append((min(a for a, _ in spans), i))
        for ev in self.events:
            target = 0
            for start, i in starts:
                if start <= ev["boottime_ns"]:
                    target = i
            out[target].append(ev)
        return out

    def _track(self, last: dict[str, int], topic: str, log_ns: int) -> None:
        if topic in last and log_ns < last[topic]:
            self.out_of_order[topic] = self.out_of_order.get(topic, 0) + 1
        last[topic] = log_ns

    def video_stream(
        self, segs: Iterable[SegmentFiles], stats: dict[int, _SegStats]
    ) -> Iterator[_Item]:
        cfg, tl = self.cfg, self.timeline
        last: dict[str, int] = {}
        seq = 0
        n = 0
        for seg in segs:
            first_ts: int | None = None
            last_sample: tuple[int, int] | None = None
            sampled = False
            took_first = False
            for frame in iter_video_frames(
                seg,
                cfg.image,
                allow_prefix=self.opts.allow_prefix,
                warnings=self.video_warnings,
            ):
                if not self._size_warned and (frame.width, frame.height) != (
                    cfg.camera.width,
                    cfg.camera.height,
                ):
                    self._size_warned = True
                    self.video_warnings.append(
                        f"decoded frame size {frame.width}x{frame.height} differs from "
                        f"camera_info {cfg.camera.width}x{cfg.camera.height} "
                        "([camera] width/height); intrinsics may not match the images"
                    )
                boot = frame.meta.sensor_ts_ns
                log = tl.to_log_ns(boot)
                self._track(last, cfg.topics.image, log)
                stats[seg.index].frames += 1
                stats[seg.index].touch(boot)
                if first_ts is None:
                    first_ts = boot
                    prev = self.prev_seg_last
                    if prev is not None:
                        gap = boot - prev[1]
                        self.file_findings.append(
                            (
                                _INFO if gap < _GAP_WARN_NS else _WARN,
                                f"segment gap {prev[0]}->{seg.index}: {gap} ns",
                            )
                        )
                wall_us = frame.meta.frame_wall_clock_us
                if wall_us is not None:
                    last_sample = (boot, wall_us)
                    sampled = n % _SAMPLE_STRIDE == 0 or not took_first
                    took_first = True
                    if sampled:
                        self.samples.append(last_sample)
                n += 1
                self.prev_seg_last = (seg.index, boot)
                img = mapping.compressed_image(frame, log, cfg.topics.frame_camera)
                yield (
                    log,
                    (_RANK_VIDEO, seq),
                    cfg.topics.image,
                    "sensor_msgs/CompressedImage",
                    img,
                )
                meta = mapping.camera_metadata(frame, log, boot, seg.index)
                meta["header"]["frame_id"] = cfg.topics.frame_camera
                yield (
                    log,
                    (_RANK_VIDEO, seq + 1),
                    cfg.topics.camera_metadata,
                    "radar_cam_msgs/CameraMetadata",
                    meta,
                )
                seq += 2
            if last_sample is not None and not sampled:
                self.samples.append(last_sample)

    def radar_stream(
        self, segs: Iterable[SegmentFiles], rstats: RadarStats, stats: dict[int, _SegStats]
    ) -> Iterator[_Item]:
        cfg, tl = self.cfg, self.timeline
        last: dict[str, int] = {}
        seq = 0
        for seg in segs:
            if seg.radar is None:
                continue
            reports: Iterator[ld2451.RadarReport] = iter_radar_reports(
                seg.radar,
                rstats,
                decoder=self.decoder,
                recover_short_frames=cfg.radar.recover_short_frames,
            )
            for report in reports:
                boot = report.t_rx_ns
                log = tl.to_log_ns(boot)
                self._track(last, cfg.topics.radar_scan, log)
                stats[seg.index].radar_reports += 1
                stats[seg.index].touch(boot)
                scan = mapping.radar_scan(report, log, cfg.topics.frame_radar, cfg.radar)
                yield (log, (_RANK_RADAR, seq), cfg.topics.radar_scan, "radar_msgs/RadarScan", scan)
                status = mapping.radar_status(report, log, boot)
                status["header"]["frame_id"] = cfg.topics.frame_radar
                yield (
                    log,
                    (_RANK_RADAR, seq + 1),
                    cfg.topics.radar_status,
                    "radar_cam_msgs/RadarStatus",
                    status,
                )
                seq += 2

    def static_stream(self, log_ns: int) -> Iterator[_Item]:
        cfg = self.cfg
        info = mapping.camera_info(cfg.camera, log_ns, cfg.topics.frame_camera)
        yield (
            log_ns,
            (_RANK_STATIC, 0),
            cfg.topics.camera_info,
            "sensor_msgs/CameraInfo",
            info,
        )
        tf = mapping.tf_static(cfg.transforms, log_ns)
        yield (log_ns, (_RANK_STATIC, 1), cfg.topics.tf_static, "tf2_msgs/TFMessage", tf)

    def event_stream(
        self, events: list[dict[str, Any]], pre: list[_Finding], log_ns: int
    ) -> Iterator[_Item]:
        topic = self.cfg.topics.events
        last: dict[str, int] = {}
        for i, (level, text) in enumerate(pre):
            yield (
                log_ns,
                (_RANK_STATIC, 2 + i),
                topic,
                "rcl_interfaces/Log",
                _finding(text, level, log_ns),
            )
        for seq, ev in enumerate(events):
            log = self.timeline.to_log_ns(ev["boottime_ns"])
            self._track(last, topic, log)
            yield (
                log,
                (_RANK_EVENT, seq),
                topic,
                "rcl_interfaces/Log",
                mapping.event_message(ev, log),
            )

    def _post_findings(self, rstats: RadarStats, extra: dict[str, str]) -> list[_Finding]:
        """Findings only known after streaming, for the file just written."""
        out: list[_Finding] = list(self.file_findings)
        new = self.timeline.warnings[self._seen["timeline"] :]
        new += self.video_warnings[self._seen["video"] :]
        self._seen = {"video": len(self.video_warnings), "timeline": len(self.timeline.warnings)}
        out += [(_WARN, w) for w in new]
        for topic, n in self.out_of_order.items():
            delta = n - self._seen_ooo.get(topic, 0)
            if delta:
                out.append((_WARN, f"{topic}: {delta} messages out of order within stream"))
        self._seen_ooo = dict(self.out_of_order)
        bad = {k: getattr(rstats, k) for k in _RADAR_PROBLEMS if getattr(rstats, k)}
        if bad:
            listing = ", ".join(f"{k}={v}" for k, v in bad.items())
            msg = f"radar decoding problems: {listing}"
            # trailing bytes alone (a stream cut mid-frame) is expected, not a warning
            level = _INFO if set(bad) == {"trailing_bytes"} else _WARN
            out.append((level, msg))
            self.findings.append((level, msg))
        if rstats.recovered_frames > 0:
            msg = (
                f"recovered {rstats.recovered_frames} short radar data frames: "
                "reported count exceeds targets present"
            )
            out.append((_INFO, msg))
            self.findings.append((_INFO, msg))
        self.findings += self.file_findings
        if self.samples:
            worst = max(abs(self.timeline.to_wall_ns(b) - w * 1000) / 1e6 for b, w in self.samples)
            extra["crosscheck_max_diff_ms"] = f"{worst:.3f}"
        else:
            extra["crosscheck_max_diff_ms"] = "n/a"
        extra["crosscheck_samples"] = str(len(self.samples))
        for w in crosscheck(self.timeline, self.samples, self.cfg.time.crosscheck_tolerance_ms):
            out.append((_WARN, w))
            self.findings.append((_WARN, w))
        return out

    def write_file(
        self,
        path: Path,
        segs: list[SegmentFiles],
        *,
        last_file: bool,
        first_file: bool = True,
        events: list[dict[str, Any]] | None = None,
    ) -> RadarStats:
        cfg = self.cfg
        events = events or []
        self.file_findings = []
        self.samples = []
        stats = {
            s.index: _SegStats(
                index=s.index,
                listed=s.index in self.session.listed_segment_indexes,
                session_json=self.listing.get(s.index, {}),
            )
            for s in segs
        }
        rstats = RadarStats()
        topics = cfg.topics
        with McapSink(path, cfg.mcap) as sink:
            for topic, datatype in (
                (topics.image, "sensor_msgs/CompressedImage"),
                (topics.camera_info, "sensor_msgs/CameraInfo"),
                (topics.camera_metadata, "radar_cam_msgs/CameraMetadata"),
                (topics.radar_scan, "radar_msgs/RadarScan"),
                (topics.radar_status, "radar_cam_msgs/RadarStatus"),
                (topics.tf_static, "tf2_msgs/TFMessage"),
                (topics.events, "rcl_interfaces/Log"),
            ):
                sink.register(topic, datatype)

            video = self.video_stream(segs, stats)
            radar = self.radar_stream(segs, rstats, stats)
            # Peek each stream's first message: the static messages go at the earliest one.
            head_v = next(video, None)
            head_r = next(radar, None)
            firsts = [h[0] for h in (head_v, head_r) if h is not None]
            firsts += [self.timeline.to_log_ns(ev["boottime_ns"]) for ev in events]
            if firsts:
                t0 = min(firsts)
            else:
                t0 = self.timeline.to_log_ns(min(a.boottime_ns for a in self.session.anchors))
            streams: list[Iterable[_Item]] = [
                self.static_stream(t0),
                _chain(head_v, video),
                _chain(head_r, radar),
            ]
            pre: list[_Finding] = []
            if first_file:
                pre = [(_WARN, w) for w in self.session.warnings]
                pre += [(_WARN, w) for w in self.event_warnings]
                if self.quality == "unsynced":
                    pre.append((_WARN, _UNSYNCED))
            streams.append(self.event_stream(events, pre, t0))

            merged = heapq.merge(*streams, key=lambda m: (m[0], m[1]))
            end_ns = t0
            for log_ns, _order, topic, _datatype, message in merged:
                sink.write(topic, message, log_ns)
                end_ns = max(end_ns, log_ns)

            if last_file:
                flush_trailing(self.decoder, rstats)
            extra: dict[str, str] = {}
            for level, text in self._post_findings(rstats, extra):
                sink.write(topics.events, _finding(text, level, end_ns), end_ns)
            self._write_metadata(sink, segs, stats, rstats, extra)
        return rstats

    def _write_metadata(
        self,
        sink: McapSink,
        segs: list[SegmentFiles],
        stats: dict[int, _SegStats],
        rstats: RadarStats,
        extra: dict[str, str],
    ) -> None:
        cfg, s = self.cfg, self.session
        warnings = self.warnings()
        sink.add_metadata(
            "radar_cam_2_mcap",
            {
                "contract_version": radar_cam_2_mcap.CONTRACT_VERSION,
                "converter_version": radar_cam_2_mcap.__version__,
                "time_quality": self.quality,
                "time_base": self.timeline.base,
                "warnings": _json(warnings),
                "notes": _json(self.notes()),
                "image_format": cfg.image.format,
                "image_quality": str(cfg.image.quality),
                "image_subsampling": str(cfg.image.subsampling),
                "mcap_compression": cfg.mcap.compression,
                "mcap_chunk_size": str(cfg.mcap.chunk_size),
                **extra,
            },
        )
        sink.add_metadata(
            "source_session",
            {
                "session_id": s.session_id,
                "boot_id": s.boot_id or "",
                "state": s.state,
                "stop_reason": s.stop_reason or "",
                "ntp_synced": str(s.ntp_synced).lower(),
                "wall_clock_source": s.wall_clock_source or "",
                "anchors": _json([asdict(a) for a in s.anchors]),
                "recorder_config": _json(s.recorder_config),
                "devices": _json(s.devices),
                "source_warnings": _json(s.warnings),
            },
        )
        sink.add_metadata(
            "segments",
            {
                "segments": _json(
                    [
                        {
                            "index": st.index,
                            "listed": st.listed,
                            "frames": st.frames,
                            "radar_reports": st.radar_reports,
                            "first_boot_ns": st.first_boot_ns,
                            "last_boot_ns": st.last_boot_ns,
                            "session_json": st.session_json,
                        }
                        for st in stats.values()
                    ]
                )
            },
        )
        sink.add_metadata("radar_stats", {k: str(v) for k, v in asdict(rstats).items()})


def _finding(text: str, level: int, log_ns: int) -> dict[str, Any]:
    return mapping.event_message({"kind": "converter", "level": level, "reason": text}, log_ns)


def _chain(head: _Item | None, rest: Iterator[_Item]) -> Iterator[_Item]:
    if head is not None:
        yield head
    yield from rest


def output_paths(session: SessionInfo, out_dir: Path, per_segment: bool) -> list[Path]:
    """Final output paths of a session (one per segment with ``per_segment``)."""
    out_dir = Path(out_dir)
    if per_segment:
        return [
            out_dir / f"{session.session_id}-seg{seg.index:04d}.mcap" for seg in session.segments
        ]
    return [out_dir / f"{session.session_id}.mcap"]


def temp_glob(final: Path) -> str:
    """Glob pattern matching the temp files written for ``final`` (see ``_make_temp``)."""
    return f"{final.name}.*.tmp"


def _make_temp(final: Path) -> Path:
    """Create a unique temp file next to ``final``; its mode follows the umask like ``open``."""
    fd, name = tempfile.mkstemp(dir=final.parent, prefix=final.name + ".", suffix=".tmp")
    try:
        mask = os.umask(0)
        os.umask(mask)
        os.fchmod(fd, 0o666 & ~mask)
    finally:
        os.close(fd)
    return Path(name)


def convert_session(
    session_dir: Path, out_dir: Path, cfg: Config, opts: ConvertOptions
) -> ConvertResult:
    """Convert one session folder; see the module docstring."""
    session = load_session(Path(session_dir))
    timeline = Timeline(session.anchors, opts.time_base or cfg.time.base)
    out_dir = Path(out_dir)

    finals = output_paths(session, out_dir, opts.per_segment)
    if opts.per_segment:
        groups = [(final, [seg]) for final, seg in zip(finals, session.segments, strict=True)]
    else:
        groups = [(finals[0], list(session.segments))]
    if not opts.overwrite:
        for final, _ in groups:
            if final.exists():
                raise OutputExistsError(f"{final} exists (use --overwrite to replace it)")
    out_dir.mkdir(parents=True, exist_ok=True)

    run = _Run(session, cfg, opts, timeline)
    event_groups = run.assign_events([segs for _, segs in groups])
    pairs: list[tuple[Path, Path]] = []
    try:
        for i, (final, segs) in enumerate(groups):
            tmp = _make_temp(final)
            pairs.append((tmp, final))
            run.write_file(
                tmp,
                segs,
                last_file=i == len(groups) - 1,
                first_file=i == 0,
                events=event_groups[i],
            )
        for tmp, final in pairs:
            os.replace(tmp, final)
    except BaseException:
        for tmp, _ in pairs:
            tmp.unlink(missing_ok=True)
        raise
    return ConvertResult(
        outputs=[final for _, final in pairs], warnings=run.warnings(), notes=run.notes()
    )
