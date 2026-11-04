"""Video segment reading: meta.json parsing, H.264 frame counting, decode and image encoding.

``meta.json`` is the authority for per-frame timestamps; the H.264 stream is decoded and paired
with it in order.
"""

from __future__ import annotations

import io
import json
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import av
from PIL import Image

from radar_cam_2_mcap.config import ImageConfig
from radar_cam_2_mcap.session import SegmentFiles

__all__ = [
    "FrameCountMismatch",
    "FrameMeta",
    "VideoFrame",
    "count_h264_frames",
    "iter_video_frames",
    "read_frame_meta",
]

_CHUNK = 8 * 1024 * 1024
_START_CODE = b"\x00\x00\x01"
_PNG_COMPRESS_LEVEL = 6


class FrameCountMismatch(Exception):
    """The H.264 stream and meta.json disagree on the number of frames."""


@dataclass(frozen=True)
class FrameMeta:
    index: int
    sensor_ts_ns: int
    frame_wall_clock_us: int | None
    fields: dict[str, Any]


@dataclass(frozen=True)
class VideoFrame:
    meta: FrameMeta
    data: bytes
    fmt: Literal["jpeg", "png"]
    width: int
    height: int


def read_frame_meta(meta: Path) -> list[FrameMeta]:
    """Parse rpicam-vid's meta.json, tolerating a cut-off last object and a missing ``]``."""
    text = meta.read_text(encoding="utf-8", errors="replace")
    decoder = json.JSONDecoder()
    pos = text.find("[")
    pos = len(text) if pos < 0 else pos + 1
    frames: list[FrameMeta] = []
    while True:
        while pos < len(text) and text[pos] in " \t\r\n,":
            pos += 1
        if pos >= len(text) or text[pos] != "{":
            break
        try:
            obj, pos = decoder.raw_decode(text, pos)
        except ValueError:
            break  # truncated tail
        if not isinstance(obj, dict) or "SensorTimestamp" not in obj:
            continue
        wall = obj.get("FrameWallClock")
        frames.append(
            FrameMeta(
                index=len(frames),
                sensor_ts_ns=int(obj["SensorTimestamp"]),
                frame_wall_clock_us=int(wall) if wall is not None else None,
                fields=obj,
            )
        )
    return frames


def count_h264_frames(path: Path) -> int:
    """Count slice NAL units (types 1 and 5) in an Annex B stream without decoding."""
    count = 0
    carry = b""
    with path.open("rb") as fh:
        while True:
            chunk = fh.read(_CHUNK)
            if not chunk:
                break
            buf = carry + chunk
            limit = len(buf) - 3  # a start code at i needs its header byte at i + 3 inside buf
            i = buf.find(_START_CODE)
            while 0 <= i < limit:
                if buf[i + 3] & 0x1F in (1, 5):
                    count += 1
                i = buf.find(_START_CODE, i + 3)
            carry = buf[max(limit, 0) :]
    return count


def _encode(rgb: Any, image: ImageConfig) -> tuple[bytes, Literal["jpeg", "png"]]:
    img = Image.fromarray(rgb, "RGB")
    out = io.BytesIO()
    if image.format == "png":
        img.save(out, format="PNG", compress_level=_PNG_COMPRESS_LEVEL)
        return out.getvalue(), "png"
    img.save(out, format="JPEG", quality=image.quality, subsampling=image.subsampling)
    return out.getvalue(), "jpeg"


def iter_video_frames(
    seg: SegmentFiles,
    image: ImageConfig,
    *,
    allow_prefix: bool,
    warnings: list[str],
) -> Iterator[VideoFrame]:
    """Yield decoded, encoded frames paired with their meta.json entries, in order."""
    if seg.h264 is None or seg.meta is None:
        missing = "h264" if seg.h264 is None else "meta.json"
        warnings.append(f"segment {seg.index:04d}: {missing} missing, no video frames converted")
        return
    metas = read_frame_meta(seg.meta)
    n_stream = count_h264_frames(seg.h264)
    if n_stream != len(metas):
        if not allow_prefix:
            raise FrameCountMismatch(
                f"segment {seg.index:04d}: h264 has {n_stream} frames but meta.json has "
                f"{len(metas)}; use --allow-prefix to convert the common prefix"
            )
        longer, diff = (
            ("h264", n_stream - len(metas))
            if n_stream > len(metas)
            else (
                "meta.json",
                len(metas) - n_stream,
            )
        )
        warnings.append(
            f"segment {seg.index:04d}: {longer} is longer by {diff} frames "
            f"(h264 {n_stream}, meta.json {len(metas)}); converting the common prefix"
        )
    expected = min(n_stream, len(metas))
    produced = 0
    if expected > 0:
        with av.open(str(seg.h264), format="h264") as container:
            for frame in container.decode(video=0):
                if produced >= expected:
                    break
                rgb = frame.to_ndarray(format="rgb24")
                data, fmt = _encode(rgb, image)
                yield VideoFrame(
                    meta=metas[produced],
                    data=data,
                    fmt=fmt,
                    width=int(frame.width),
                    height=int(frame.height),
                )
                produced += 1
    if produced < expected:
        warnings.append(
            f"segment {seg.index:04d}: decoder produced {produced} of {expected} expected frames"
        )
