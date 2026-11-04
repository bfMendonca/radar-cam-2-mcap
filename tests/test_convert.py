from __future__ import annotations

import hashlib
import itertools
import json
import shutil
from collections import Counter
from pathlib import Path
from typing import Any

import pytest
from mcap.reader import make_reader
from mcap_ros2.reader import read_ros2_messages

import radar_cam_2_mcap
from radar_cam_2_mcap.config import Config, McapConfig, TimeConfig
from radar_cam_2_mcap.convert import ConvertOptions, OutputExistsError, convert_session
from radar_cam_2_mcap.radar import (
    RadarStats,
    RecoveringFrameDecoder,
    flush_trailing,
    iter_radar_reports,
)
from radar_cam_2_mcap.session import load_session
from radar_cam_2_mcap.video import FrameCountMismatch, read_frame_meta

FIXTURES = Path(__file__).parent / "fixtures"
CFG = Config()


def _tree_hash(root: Path) -> str:
    h = hashlib.sha256()
    for p in sorted(root.rglob("*")):
        h.update(str(p.relative_to(root)).encode())
        if p.is_file():
            h.update(p.read_bytes())
    return h.hexdigest()


def _frame_count(session: Path) -> int:
    return sum(
        len(read_frame_meta(s.meta)) for s in load_session(session).segments if s.meta and s.h264
    )


def _radar_count(session: Path) -> int:
    dec = RecoveringFrameDecoder(recover=True)
    stats = RadarStats()
    n = 0
    for seg in load_session(session).segments:
        if seg.radar:
            n += sum(1 for _ in iter_radar_reports(seg.radar, stats, decoder=dec))
    flush_trailing(dec, stats)
    return n


def _topic_counts(path: Path) -> Counter[str]:
    return Counter(m.channel.topic for m in read_ros2_messages(path))


def _log_times(path: Path) -> list[int]:
    with path.open("rb") as fh:
        return [m.log_time for _s, _c, m in make_reader(fh).iter_messages()]


def _metadata(path: Path) -> dict[str, dict[str, str]]:
    with path.open("rb") as fh:
        return {r.name: dict(r.metadata) for r in make_reader(fh).iter_metadata()}


def _convert(name: str, tmp_path: Path, **kw: Any) -> tuple[Path, Any]:
    res = convert_session(FIXTURES / name, tmp_path / "out", CFG, ConvertOptions(**kw))
    return res.outputs[0], res


@pytest.mark.parametrize("name", ["complete_2seg", "in_progress_unlisted", "interrupted"])
def test_convert_counts_and_order(name: str, tmp_path: Path) -> None:
    out, res = _convert(name, tmp_path)
    sid = load_session(FIXTURES / name).session_id
    assert out.name == f"{sid}.mcap"
    counts = _topic_counts(out)
    frames = _frame_count(FIXTURES / name)
    scans = _radar_count(FIXTURES / name)
    assert counts["/camera/image/compressed"] == frames
    assert counts["/camera/metadata"] == frames
    assert counts["/radar/scan"] == scans
    assert counts["/radar/status"] == scans
    assert counts["/camera/camera_info"] == 1
    assert counts["/tf_static"] == 1
    times = _log_times(out)
    assert times == sorted(times)
    assert not list((tmp_path / "out").glob("*.tmp"))
    assert res.outputs == [out]


def test_convert_complete_fixture(tmp_path: Path) -> None:
    out, res = _convert("complete_2seg", tmp_path)
    assert res.warnings == []
    meta = _metadata(out)
    assert meta["radar_cam_2_mcap"]["contract_version"] == "0.1"
    assert radar_cam_2_mcap.CONTRACT_VERSION == "0.1"
    assert meta["radar_cam_2_mcap"]["converter_version"] == radar_cam_2_mcap.__version__
    assert meta["radar_cam_2_mcap"]["time_quality"] == "ntp"
    assert meta["radar_cam_2_mcap"]["time_base"] == "wall"
    assert meta["radar_cam_2_mcap"]["image_format"] == "jpeg"
    assert json.loads(meta["radar_cam_2_mcap"]["warnings"]) == []
    src = meta["source_session"]
    assert src["session_id"] == "s000066"
    assert src["state"] == "complete"
    assert len(json.loads(src["anchors"])) == 6
    segs = json.loads(meta["segments"]["segments"])
    assert [s["index"] for s in segs] == [0, 1]
    assert all(s["listed"] for s in segs)
    assert sum(s["frames"] for s in segs) == _frame_count(FIXTURES / "complete_2seg")
    stats = meta["radar_stats"]
    assert {"reads", "data_frames", "trailing_bytes", "recovered_frames"} <= set(stats)


def test_first_image_stamp_equals_log_time_and_jpeg_roundtrip(tmp_path: Path) -> None:
    out, _ = _convert("complete_2seg", tmp_path)
    first = next(m for m in read_ros2_messages(out, topics=["/camera/image/compressed"]))
    img = first.ros_msg
    assert img.format == "jpeg"
    assert bytes(img.data)[:2] == b"\xff\xd8"
    stamp = img.header.stamp.sec * 1_000_000_000 + img.header.stamp.nanosec
    assert stamp == first.log_time_ns == first.publish_time_ns
    assert img.header.frame_id == CFG.topics.frame_camera


def test_static_messages_at_first_message_time(tmp_path: Path) -> None:
    out, _ = _convert("complete_2seg", tmp_path)
    with out.open("rb") as fh:
        msgs = list(make_reader(fh).iter_messages())
    first = min(m.log_time for _s, _c, m in msgs)
    for _s, c, m in msgs:
        if c.topic in ("/camera/camera_info", "/tf_static"):
            assert m.log_time == first


def test_unsynced_fixture_is_flagged(tmp_path: Path) -> None:
    out, res = _convert("interrupted", tmp_path)
    assert _metadata(out)["radar_cam_2_mcap"]["time_quality"] == "unsynced"
    assert any("unsynced" in w for w in res.warnings)
    assert any("interrupted" in w for w in res.warnings)


def test_in_progress_unlisted_segment_converts_with_warnings(tmp_path: Path) -> None:
    src = FIXTURES / "in_progress_unlisted"
    out, res = _convert("in_progress_unlisted", tmp_path)
    assert any("not listed" in w for w in res.warnings)
    assert any("in progress" in w for w in res.warnings)
    segs = json.loads(_metadata(out)["segments"]["segments"])
    by_index = {s["index"]: s for s in segs}
    assert by_index[3]["listed"] is False
    n3 = len(read_frame_meta(src / "video" / "0003.meta.json"))
    assert n3 > 0
    assert by_index[3]["frames"] == n3
    boots = {
        m.ros_msg.boot_ns
        for m in read_ros2_messages(out, topics=["/camera/metadata"])
        if m.ros_msg.segment == 3
    }
    assert len(boots) == n3


def test_per_segment_option_writes_one_file_per_segment(tmp_path: Path) -> None:
    res = convert_session(
        FIXTURES / "complete_2seg", tmp_path / "o", CFG, ConvertOptions(per_segment=True)
    )
    assert [p.name for p in res.outputs] == ["s000066-seg0000.mcap", "s000066-seg0001.mcap"]
    total: Counter[str] = Counter()
    for p in res.outputs:
        c = _topic_counts(p)
        assert c["/camera/camera_info"] == 1 and c["/tf_static"] == 1
        total.update(c)
    whole, _ = _convert("complete_2seg", tmp_path)
    ref = _topic_counts(whole)
    for topic in ("/camera/image/compressed", "/radar/scan", "/radar/status"):
        assert total[topic] == ref[topic]


def test_time_base_boot_uses_boot_clock(tmp_path: Path) -> None:
    out, _ = _convert("complete_2seg", tmp_path, time_base="boot")
    assert _metadata(out)["radar_cam_2_mcap"]["time_base"] == "boot"
    for m in read_ros2_messages(out, topics=["/camera/metadata"]):
        assert m.log_time_ns == m.ros_msg.boot_ns
    for m in read_ros2_messages(out, topics=["/radar/status"]):
        assert m.log_time_ns == m.ros_msg.boot_ns


def test_time_base_falls_back_to_config(tmp_path: Path) -> None:
    cfg = Config(time=TimeConfig(base="boot"))
    res = convert_session(FIXTURES / "complete_2seg", tmp_path, cfg, ConvertOptions())
    for m in read_ros2_messages(res.outputs[0], topics=["/camera/metadata"]):
        assert m.log_time_ns == m.ros_msg.boot_ns


def test_source_folder_unchanged(tmp_path: Path) -> None:
    src = FIXTURES / "complete_2seg"
    before = _tree_hash(src)
    convert_session(src, tmp_path / "o", CFG, ConvertOptions())
    assert _tree_hash(src) == before


@pytest.mark.parametrize("compression", ["zstd", "lz4", "none"])
def test_converted_file_uses_configured_compression(tmp_path: Path, compression: str) -> None:
    cfg = Config(mcap=McapConfig(compression=compression, chunk_size=64 * 1024))  # type: ignore[arg-type]
    res = convert_session(FIXTURES / "complete_2seg", tmp_path, cfg, ConvertOptions())
    with res.outputs[0].open("rb") as fh:
        summary = make_reader(fh).get_summary()
    assert summary is not None
    expected = "" if compression == "none" else compression
    assert {c.compression for c in summary.chunk_indexes} == {expected}
    meta = _metadata(res.outputs[0])["radar_cam_2_mcap"]
    assert meta["mcap_compression"] == compression
    assert meta["mcap_chunk_size"] == str(64 * 1024)


def test_output_exists_refused_unless_overwrite(tmp_path: Path) -> None:
    out = tmp_path / "o"
    convert_session(FIXTURES / "complete_2seg", out, CFG, ConvertOptions())
    target = out / "s000066.mcap"
    stamp = target.stat().st_mtime_ns
    with pytest.raises(OutputExistsError):
        convert_session(FIXTURES / "complete_2seg", out, CFG, ConvertOptions())
    assert target.stat().st_mtime_ns == stamp
    convert_session(FIXTURES / "complete_2seg", out, CFG, ConvertOptions(overwrite=True))
    assert not list(out.glob("*.tmp"))


def test_failure_leaves_no_final_file(tmp_path: Path) -> None:
    src = tmp_path / "s"
    shutil.copytree(FIXTURES / "complete_2seg", src)
    meta = src / "video" / "0001.meta.json"
    objs = [m.fields for m in read_frame_meta(meta)][:-3]
    meta.write_text(json.dumps(objs))
    out = tmp_path / "o"
    with pytest.raises(FrameCountMismatch):
        convert_session(src, out, CFG, ConvertOptions())
    assert list(out.iterdir()) == []
    # allow_prefix converts, with a warning
    res = convert_session(src, out, CFG, ConvertOptions(allow_prefix=True))
    assert any("common prefix" in w for w in res.warnings)


def test_failure_does_not_replace_existing_file(tmp_path: Path) -> None:
    src = tmp_path / "s"
    shutil.copytree(FIXTURES / "complete_2seg", src)
    out = tmp_path / "o"
    convert_session(src, out, CFG, ConvertOptions())
    good = (out / "s000066.mcap").read_bytes()
    meta = src / "video" / "0001.meta.json"
    meta.write_text(json.dumps([m.fields for m in read_frame_meta(meta)][:-3]))
    with pytest.raises(FrameCountMismatch):
        convert_session(src, out, CFG, ConvertOptions(overwrite=True))
    assert (out / "s000066.mcap").read_bytes() == good
    assert not list(out.glob("*.tmp"))


def _expected_wall(anchors: list[dict[str, int]], boot: int) -> int:
    """Piecewise-linear offset interpolation written out by hand."""
    for a, b in itertools.pairwise(anchors):
        if a["boottime_ns"] <= boot <= b["boottime_ns"]:
            oa = a["realtime_ns"] - a["boottime_ns"]
            ob = b["realtime_ns"] - b["boottime_ns"]
            frac = (boot - a["boottime_ns"]) / (b["boottime_ns"] - a["boottime_ns"])
            return boot + oa + round((ob - oa) * frac)
    raise AssertionError("boot outside anchors")


def test_multi_anchor_piecewise_interpolation_across_segments(tmp_path: Path) -> None:
    src = tmp_path / "s"
    shutil.copytree(FIXTURES / "complete_2seg", src)
    sj = src / "session.json"
    data = json.loads(sj.read_text())
    a0 = data["clock_samples"][0]
    a1 = dict(data["clock_samples"][1])
    a1["realtime_ns"] += 3_000_000_000
    mid = {
        "label": "mid",
        "boottime_ns": 400_000_000_000,
        "monotonic_ns": 400_000_000_000,
        "realtime_ns": 400_000_000_000 + (a0["realtime_ns"] - a0["boottime_ns"]) + 700_000_000,
    }
    anchors = [a0, mid, a1]
    data["clock_samples"] = anchors
    sj.write_text(json.dumps(data))
    res = convert_session(src, tmp_path / "o", CFG, ConvertOptions())
    firsts: dict[int, Any] = {}
    for m in read_ros2_messages(res.outputs[0], topics=["/camera/metadata"]):
        if m.ros_msg.frame_index == 0:
            firsts[m.ros_msg.segment] = m
    assert set(firsts) == {0, 1}
    for seg, m in firsts.items():
        boot = m.ros_msg.boot_ns
        want = _expected_wall(anchors, boot)
        assert abs(m.log_time_ns - want) <= 1, seg
    # segment 0 lies before the middle anchor, segment 1 after: different offsets
    off0 = firsts[0].log_time_ns - firsts[0].ros_msg.boot_ns
    off1 = firsts[1].log_time_ns - firsts[1].ros_msg.boot_ns
    assert off1 - off0 > 1_000_000_000


def test_out_of_order_stream_is_counted_not_altered(tmp_path: Path) -> None:
    # Anchors whose offsets change faster than the clock make log time non-monotonic.
    src = tmp_path / "s"
    shutil.copytree(FIXTURES / "complete_2seg", src)
    sj = src / "session.json"
    data = json.loads(sj.read_text())
    a0, a1 = data["clock_samples"][0], data["clock_samples"][1]
    steep = dict(a1)
    steep["realtime_ns"] = a1["realtime_ns"] - 600_000_000_000
    data["clock_samples"] = [a0, steep]
    sj.write_text(json.dumps(data))
    res = convert_session(src, tmp_path / "o", CFG, ConvertOptions())
    assert any("out of order within stream" in w for w in res.warnings)
