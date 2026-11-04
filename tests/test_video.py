from __future__ import annotations

import io
import itertools
import re
import shutil
from pathlib import Path

import pytest
from PIL import Image, JpegImagePlugin

from radar_cam_2_mcap.config import ImageConfig
from radar_cam_2_mcap.session import SegmentFiles
from radar_cam_2_mcap.video import (
    FrameCountMismatch,
    VideoFrame,
    count_h264_frames,
    iter_video_frames,
    read_frame_meta,
)

FIX = Path(__file__).parent / "fixtures" / "complete_2seg" / "video"


def _seg(h264: Path | None, meta: Path | None, index: int = 0) -> SegmentFiles:
    return SegmentFiles(index=index, h264=h264, meta=meta, radar=None)


def _fixture_seg() -> SegmentFiles:
    return _seg(FIX / "0000.h264", FIX / "0000.meta.json")


def _truncated_meta_copy(tmp_path: Path, keep: int) -> SegmentFiles:
    """Copy of segment 0 whose meta.json keeps only `keep` frames."""
    metas = read_frame_meta(FIX / "0000.meta.json")
    import json

    objs = [m.fields for m in metas[:keep]]
    meta = tmp_path / "0000.meta.json"
    meta.write_text(json.dumps(objs))
    h264 = tmp_path / "0000.h264"
    shutil.copy(FIX / "0000.h264", h264)
    return _seg(h264, meta)


def test_read_frame_meta_counts_and_first_timestamp() -> None:
    metas = read_frame_meta(FIX / "0000.meta.json")
    assert len(metas) == count_h264_frames(FIX / "0000.h264")
    assert metas[0].index == 0
    assert metas[0].sensor_ts_ns == 249076560000
    assert metas[0].frame_wall_clock_us == 1793353171516005
    assert metas[0].fields["ExposureTime"] == 8385


def test_read_frame_meta_truncated_tail(tmp_path: Path) -> None:
    full = (FIX / "0000.meta.json").read_text()
    total = len(read_frame_meta(FIX / "0000.meta.json"))
    cut = tmp_path / "m.json"
    cut.write_text(full[: len(full) // 2])
    metas = read_frame_meta(cut)
    assert 0 < len(metas) < total
    assert [m.index for m in metas] == list(range(len(metas)))


def test_read_frame_meta_skips_objects_without_sensor_timestamp(tmp_path: Path) -> None:
    p = tmp_path / "m.json"
    p.write_text('[{"a": 1}, {"SensorTimestamp": 5}, {"SensorTimestamp": 9}]')
    metas = read_frame_meta(p)
    assert [(m.index, m.sensor_ts_ns, m.frame_wall_clock_us) for m in metas] == [
        (0, 5, None),
        (1, 9, None),
    ]


def test_count_h264_frames_matches_meta_on_complete_fixture() -> None:
    for name in ("0000", "0001"):
        assert count_h264_frames(FIX / f"{name}.h264") == len(
            read_frame_meta(FIX / f"{name}.meta.json")
        )


def _reference_count(data: bytes) -> int:
    return sum(
        1
        for m in re.finditer(rb"\x00\x00\x01(.)", data, re.DOTALL)
        if m.group(1)[0] & 0x1F in (1, 5)
    )


_SYNTHETIC = [
    b"\x00\x00\x01\x65aa\x00\x00\x01\x67bb\x00\x00\x00\x01\x41cc\x00\x00\x01\x41",
    b"\x00\x00\x00\x01\x67x\x00\x00\x00\x01\x68y\x00\x00\x00\x01\x65z\x00\x00\x01\x01",
    b"\x00\x00\x01\x41\x00\x00\x01\x41\x00\x00\x01\x65",
    b"zz\x00\x00\x01\x21q\x00\x00\x02\x01\x00\x00\x01\x25",
]


@pytest.mark.parametrize("chunk_size", [1, 2, 3, 4, 5, 7])
@pytest.mark.parametrize("data", _SYNTHETIC)
def test_count_h264_frames_across_chunk_boundary(
    tmp_path: Path, data: bytes, chunk_size: int
) -> None:
    expected = _reference_count(data)
    assert expected > 0
    # Shift the stream so each start code is split at every possible offset.
    for pad in range(8):
        p = tmp_path / "x.h264"
        p.write_bytes(b"\xaa" * pad + data)
        assert count_h264_frames(p, chunk_size=chunk_size) == expected


@pytest.mark.parametrize("chunk_size", [1000, 4093, 65536])
def test_count_h264_frames_small_chunks_on_fixture(chunk_size: int) -> None:
    h264 = FIX / "0000.h264"
    n_meta = len(read_frame_meta(FIX / "0000.meta.json"))
    assert count_h264_frames(h264) == n_meta
    assert count_h264_frames(h264, chunk_size=chunk_size) == n_meta


def test_decoded_frames_pair_with_meta_in_order() -> None:
    warnings: list[str] = []
    frames = list(
        iter_video_frames(_fixture_seg(), ImageConfig(), allow_prefix=False, warnings=warnings)
    )
    metas = read_frame_meta(FIX / "0000.meta.json")
    assert [f.meta.index for f in frames] == list(range(len(metas)))
    ts = [f.meta.sensor_ts_ns for f in frames]
    assert all(a < b for a, b in itertools.pairwise(ts))
    assert all(f.fmt == "jpeg" and f.data[:2] == b"\xff\xd8" for f in frames)
    assert all(f.width == 1296 and f.height == 972 for f in frames)
    assert warnings == []
    assert frames[0].meta.sensor_ts_ns == 249076560000
    assert frames[-1].meta.sensor_ts_ns == metas[-1].sensor_ts_ns
    assert [f.meta.sensor_ts_ns for f in frames] == [m.sensor_ts_ns for m in metas]
    assert len({f.data for f in frames}) > 1  # distinct decoded pictures


def test_iter_is_lazy() -> None:
    it = iter_video_frames(_fixture_seg(), ImageConfig(), allow_prefix=False, warnings=[])
    first = next(it)
    assert isinstance(first, VideoFrame)
    assert first.meta.index == 0


def test_mismatch_raises_without_allow_prefix(tmp_path: Path) -> None:
    seg = _truncated_meta_copy(tmp_path, 10)
    with pytest.raises(FrameCountMismatch) as e:
        list(iter_video_frames(seg, ImageConfig(), allow_prefix=False, warnings=[]))
    assert "10" in str(e.value)


def test_allow_prefix_converts_common_prefix_with_warning(tmp_path: Path) -> None:
    seg = _truncated_meta_copy(tmp_path, 10)
    warnings: list[str] = []
    frames = list(iter_video_frames(seg, ImageConfig(), allow_prefix=True, warnings=warnings))
    assert len(frames) == 10
    assert len(warnings) == 1
    assert "h264" in warnings[0]


def test_allow_prefix_meta_longer_than_stream(tmp_path: Path) -> None:
    seg = _truncated_meta_copy(tmp_path, 10)
    assert seg.h264 is not None
    # Cut the stream at the start of the 6th slice NAL: keeps 5 frames.
    data = (FIX / "0000.h264").read_bytes()
    pos = -1
    seen = 0
    i = 0
    while seen < 5:
        i = data.index(b"\x00\x00\x01", i) + 3
        if data[i] & 0x1F in (1, 5):
            seen += 1
    pos = data.index(b"\x00\x00\x01", i)
    while data[pos + 3] & 0x1F not in (1, 5):
        pos = data.index(b"\x00\x00\x01", pos + 3)
    seg.h264.write_bytes(data[:pos])
    warnings: list[str] = []
    frames = list(iter_video_frames(seg, ImageConfig(), allow_prefix=True, warnings=warnings))
    assert len(frames) == 5
    assert len(warnings) == 1 and "meta" in warnings[0]


def test_missing_h264_yields_nothing_with_warning() -> None:
    warnings: list[str] = []
    seg = _seg(None, FIX / "0000.meta.json", index=3)
    assert list(iter_video_frames(seg, ImageConfig(), allow_prefix=False, warnings=warnings)) == []
    assert len(warnings) == 1


def test_png_format_roundtrip() -> None:
    it = iter_video_frames(
        _fixture_seg(), ImageConfig(format="png"), allow_prefix=False, warnings=[]
    )
    f = next(it)
    assert f.fmt == "png"
    assert f.data[:8] == b"\x89PNG\r\n\x1a\n"
    assert Image.open(io.BytesIO(f.data)).size == (1296, 972)


def test_jpeg_uses_420_subsampling() -> None:
    f = next(iter_video_frames(_fixture_seg(), ImageConfig(), allow_prefix=False, warnings=[]))
    img = Image.open(io.BytesIO(f.data))
    assert isinstance(img, JpegImagePlugin.JpegImageFile)
    assert JpegImagePlugin.get_sampling(img) == 2


def _corrupt_frame(src: Path, dst: Path, frame: int) -> None:
    """Copy ``src`` with the payload of slice NAL number ``frame`` (0-based) overwritten."""
    data = bytearray(src.read_bytes())
    i = seen = 0
    while True:
        i = data.index(b"\x00\x00\x01", i) + 3
        if data[i] & 0x1F in (1, 5):
            if seen == frame:
                break
            seen += 1
    end = data.index(b"\x00\x00\x01", i)
    data[i + 1 : end] = b"\xff" * (end - i - 1)
    dst.write_bytes(bytes(data))


def test_undecodable_frame_skips_its_meta_entry(tmp_path: Path) -> None:
    """A frame the decoder drops must not shift the timestamps of the frames after it."""
    h264 = tmp_path / "0000.h264"
    _corrupt_frame(FIX / "0000.h264", h264, 10)
    seg = _seg(h264, FIX / "0000.meta.json")
    warnings: list[str] = []
    frames = list(iter_video_frames(seg, ImageConfig(), allow_prefix=False, warnings=warnings))
    metas = read_frame_meta(FIX / "0000.meta.json")
    assert [f.meta.index for f in frames] == [i for i in range(len(metas)) if i != 10]
    reference = {
        f.meta.index: f.data
        for f in iter_video_frames(_fixture_seg(), ImageConfig(), allow_prefix=False, warnings=[])
    }
    # frames before the bad one are untouched; later P-frames decode (concealed) at their own
    # meta entry, which is the point: frame 11 carries meta 11, not meta 12
    assert all(f.data == reference[f.meta.index] for f in frames if f.meta.index < 10)
    assert all(f.meta.sensor_ts_ns == metas[f.meta.index].sensor_ts_ns for f in frames)
    assert len(warnings) == 1
    assert "1 frame(s) not decoded" in warnings[0] and "10" in warnings[0]
    assert "not shifted" in warnings[0]


def test_lost_tail_frames_reported(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """Frames that never come out of the decoder at the end are reported, not hidden."""
    import av.video.stream

    real = av.video.stream.VideoStream.decode

    def decode(self, packet=None):  # type: ignore[no-untyped-def]
        frames = real(self, packet)
        return [f for f in frames if f.pts is None or f.pts < 37]

    monkeypatch.setattr(av.video.stream.VideoStream, "decode", decode)
    warnings: list[str] = []
    frames = list(
        iter_video_frames(_fixture_seg(), ImageConfig(), allow_prefix=False, warnings=warnings)
    )
    assert [f.meta.index for f in frames] == list(range(37))
    assert len(warnings) == 1 and "3 frame(s) not decoded" in warnings[0]


def test_frame_without_packet_index_is_a_pairing_error(monkeypatch: pytest.MonkeyPatch) -> None:
    import av.video.stream

    from radar_cam_2_mcap.video import FramePairingError

    real = av.video.stream.VideoStream.decode

    def decode(self, packet=None):  # type: ignore[no-untyped-def]
        frames = real(self, packet)
        for f in frames:
            f.pts = None
        return frames

    monkeypatch.setattr(av.video.stream.VideoStream, "decode", decode)
    with pytest.raises(FramePairingError):
        list(iter_video_frames(_fixture_seg(), ImageConfig(), allow_prefix=False, warnings=[]))
