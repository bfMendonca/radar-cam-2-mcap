"""Single-session conversion: session folder to one MCAP (or one MCAP per segment).

Streams (video, radar, static) are generators merged by ``log_ns`` with ``heapq.merge``, so
memory stays bounded by roughly one frame per stream. ``heapq.merge`` assumes every input is
sorted; each stream is sorted as long as the timeline is monotonic. If anchors make a stream's
log time go backwards the messages are NOT reordered or altered: the inversions are counted per
topic and reported as a warning, and the file is then not strictly time-ordered.

``/camera/camera_info`` is written once per file (at the time of the file's first message),
not repeated with every frame. Same for ``/tf_static``.
"""

from __future__ import annotations

import heapq
import json
import os
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
from radar_cam_2_mcap.session import SegmentFiles, SessionInfo, load_session
from radar_cam_2_mcap.sink import McapSink
from radar_cam_2_mcap.timebase import Timeline, time_quality
from radar_cam_2_mcap.video import iter_video_frames

__all__ = ["ConvertOptions", "ConvertResult", "OutputExistsError", "convert_session"]

# Merge item: (log_ns, (rank, seq), topic, datatype, message). rank orders equal timestamps:
# static 0, then video 1, then radar 2 (then events, Task 8); seq is per stream.
_Item = tuple[int, tuple[int, int], str, str, dict[str, Any]]

_RANK_STATIC, _RANK_VIDEO, _RANK_RADAR = 0, 1, 2

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

    def warnings(self) -> list[str]:
        out = [*self.session.warnings, *self.timeline.warnings, *self.video_warnings]
        out += [
            f"{topic}: {n} messages out of order within stream"
            for topic, n in self.out_of_order.items()
        ]
        if self.quality == "unsynced":
            out.append(_UNSYNCED)
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
        for seg in segs:
            for frame in iter_video_frames(
                seg,
                cfg.image,
                allow_prefix=self.opts.allow_prefix,
                warnings=self.video_warnings,
            ):
                boot = frame.meta.sensor_ts_ns
                log = tl.to_log_ns(boot)
                self._track(last, cfg.topics.image, log)
                stats[seg.index].frames += 1
                stats[seg.index].touch(boot)
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

    def write_file(self, path: Path, segs: list[SegmentFiles], *, last_file: bool) -> RadarStats:
        cfg = self.cfg
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
            ):
                sink.register(topic, datatype)

            video = self.video_stream(segs, stats)
            radar = self.radar_stream(segs, rstats, stats)
            # Peek each stream's first message: the static messages go at the earliest one.
            head_v = next(video, None)
            head_r = next(radar, None)
            firsts = [h for h in (head_v, head_r) if h is not None]
            if firsts:
                t0 = min(h[0] for h in firsts)
            else:
                t0 = self.timeline.to_log_ns(min(a.boottime_ns for a in self.session.anchors))
            streams: list[Iterable[_Item]] = [
                self.static_stream(t0),
                _chain(head_v, video),
                _chain(head_r, radar),
            ]
            # EXTENSION POINT (Task 8): add the /events stream to ``streams`` here.

            merged = heapq.merge(*streams, key=lambda m: (m[0], m[1]))
            for log_ns, _order, topic, _datatype, message in merged:
                sink.write(topic, message, log_ns)

            if last_file:
                flush_trailing(self.decoder, rstats)
            self._write_metadata(sink, segs, stats, rstats)
        return rstats

    def _write_metadata(
        self,
        sink: McapSink,
        segs: list[SegmentFiles],
        stats: dict[int, _SegStats],
        rstats: RadarStats,
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
                "image_format": cfg.image.format,
                "image_quality": str(cfg.image.quality),
                "image_subsampling": str(cfg.image.subsampling),
                "mcap_compression": cfg.mcap.compression,
                "mcap_chunk_size": str(cfg.mcap.chunk_size),
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


def _chain(head: _Item | None, rest: Iterator[_Item]) -> Iterator[_Item]:
    if head is not None:
        yield head
    yield from rest


def convert_session(
    session_dir: Path, out_dir: Path, cfg: Config, opts: ConvertOptions
) -> ConvertResult:
    """Convert one session folder; see the module docstring."""
    session = load_session(Path(session_dir))
    timeline = Timeline(session.anchors, opts.time_base or cfg.time.base)
    out_dir = Path(out_dir)

    if opts.per_segment:
        groups = [
            (out_dir / f"{session.session_id}-seg{seg.index:04d}.mcap", [seg])
            for seg in session.segments
        ]
    else:
        groups = [(out_dir / f"{session.session_id}.mcap", list(session.segments))]
    if not opts.overwrite:
        for final, _ in groups:
            if final.exists():
                raise OutputExistsError(f"{final} exists (use --overwrite to replace it)")
    out_dir.mkdir(parents=True, exist_ok=True)

    run = _Run(session, cfg, opts, timeline)
    pairs: list[tuple[Path, Path]] = []
    try:
        for i, (final, segs) in enumerate(groups):
            tmp = final.with_name(final.name + ".tmp")
            pairs.append((tmp, final))
            run.write_file(tmp, segs, last_file=i == len(groups) - 1)
        for tmp, final in pairs:
            os.replace(tmp, final)
    except BaseException:
        for tmp, _ in pairs:
            tmp.unlink(missing_ok=True)
        raise
    return ConvertResult(outputs=[final for _, final in pairs], warnings=run.warnings())
