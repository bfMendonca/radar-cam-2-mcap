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


COMPLETE_NOTES = [
    "radar decoding problems: trailing_bytes=12",
    "recovered 14 short radar data frames: reported count exceeds targets present",
]


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


ROS2_SCHEMA_NAMES = {
    "sensor_msgs/msg/CompressedImage",
    "sensor_msgs/msg/CameraInfo",
    "radar_cam_msgs/msg/CameraMetadata",
    "radar_msgs/msg/RadarScan",
    "radar_cam_msgs/msg/RadarStatus",
    "rcl_interfaces/msg/Log",
    "tf2_msgs/msg/TFMessage",
}


def test_schema_names_are_ros2_style(tmp_path: Path) -> None:
    out, _ = _convert("complete_2seg", tmp_path)
    with out.open("rb") as fh:
        summary = make_reader(fh).get_summary()
    assert summary is not None
    schemas = list(summary.schemas.values())
    assert {s.name for s in schemas} == ROS2_SCHEMA_NAMES
    assert len(schemas) == len(ROS2_SCHEMA_NAMES)
    assert {s.encoding for s in schemas} == {"ros2msg"}


def test_all_messages_decode_with_ros2_reader(tmp_path: Path) -> None:
    out, _ = _convert("complete_2seg", tmp_path)
    with out.open("rb") as fh:
        expected = sum(1 for _ in make_reader(fh).iter_messages())
    seen: set[str] = set()
    n = 0
    for item in read_ros2_messages(str(out)):
        assert item.ros_msg is not None
        seen.add(item.schema.name)
        n += 1
    assert n == expected
    assert seen == ROS2_SCHEMA_NAMES


def test_convert_complete_fixture(tmp_path: Path) -> None:
    out, res = _convert("complete_2seg", tmp_path)
    # The fixture is a trimmed excerpt, so the 0->1 gap is ~300 s (WARN); nothing else warns.
    assert res.warnings == ["segment gap 0->1: 299632291000 ns"]
    assert res.notes == COMPLETE_NOTES
    meta = _metadata(out)
    assert meta["radar_cam_2_mcap"]["contract_version"] == "0.2"
    assert radar_cam_2_mcap.CONTRACT_VERSION == "0.2"
    assert meta["radar_cam_2_mcap"]["converter_version"] == radar_cam_2_mcap.__version__
    assert meta["radar_cam_2_mcap"]["time_quality"] == "ntp"
    assert meta["radar_cam_2_mcap"]["time_base"] == "wall"
    assert meta["radar_cam_2_mcap"]["image_format"] == "jpeg"
    assert json.loads(meta["radar_cam_2_mcap"]["warnings"]) == res.warnings
    assert json.loads(meta["radar_cam_2_mcap"]["notes"]) == res.notes
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
    assert not out.exists()  # the frame-count pre-pass fails before anything is created
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


# ---- Task 8: /events, findings and cross-check ------------------------------------------


def _events(path: Path) -> list[Any]:
    return list(read_ros2_messages(path, topics=["/events"]))


def _copy(name: str, tmp_path: Path) -> Path:
    dst = tmp_path / "src"
    shutil.copytree(FIXTURES / name, dst)
    return dst


def _boot_events(name: str) -> list[dict[str, Any]]:
    lines = (FIXTURES / name / "events.jsonl").read_text().splitlines()
    return [json.loads(x) for x in lines]


def test_events_topic_has_recorder_events_and_converter_findings(tmp_path: Path) -> None:
    out, _ = _convert("in_progress_unlisted", tmp_path)
    msgs = _events(out)
    times = _log_times(out)
    rec = [m for m in msgs if m.ros_msg.name == "radar_cam_recorder"]
    src = _boot_events("in_progress_unlisted")
    assert len(rec) == len(src)
    session = load_session(FIXTURES / "in_progress_unlisted")
    from radar_cam_2_mcap.timebase import Timeline

    tl = Timeline(session.anchors, "wall")
    for m, ev in zip(rec, src, strict=True):
        assert m.log_time_ns == tl.to_log_ns(ev["boottime_ns"])
        assert m.ros_msg.level == (30 if ev["kind"] == "incident" else 20)
    assert any(m.ros_msg.msg.startswith("camera exited") for m in rec)
    assert any(m.ros_msg.msg.startswith("segment gap") for m in rec)
    conv = [m for m in msgs if m.ros_msg.name == "radar_cam_2_mcap"]
    unlisted = [m for m in conv if "not listed" in m.ros_msg.msg]
    assert unlisted and unlisted[0].log_time_ns == min(times)
    assert all(m.ros_msg.level in (20, 30) for m in conv)


def _shift_wall_clock(src: Path, delta_us: int) -> None:
    for meta in (src / "video").glob("*.meta.json"):
        data = json.loads(meta.read_text())
        for fr in data:
            if "FrameWallClock" in fr:
                fr["FrameWallClock"] += delta_us
        meta.write_text(json.dumps(data))


def test_crosscheck_warning_reaches_events_and_metadata(tmp_path: Path) -> None:
    src = _copy("complete_2seg", tmp_path)
    _shift_wall_clock(src, 80_000)
    res = convert_session(src, tmp_path / "o", CFG, ConvertOptions())
    out = res.outputs[0]
    assert any("wall-clock samples differ" in w for w in res.warnings)
    md = _metadata(out)["radar_cam_2_mcap"]
    assert any("wall-clock samples differ" in w for w in json.loads(md["warnings"]))
    assert float(md["crosscheck_max_diff_ms"]) > 50
    assert int(md["crosscheck_samples"]) > 0
    last = max(_log_times(out))
    hits = [m for m in _events(out) if "wall-clock samples differ" in m.ros_msg.msg]
    assert len(hits) == 1 and hits[0].log_time_ns == last and hits[0].ros_msg.level == 30


@pytest.mark.parametrize("name", ["complete_2seg", "in_progress_unlisted", "interrupted"])
def test_unshifted_fixtures_have_no_crosscheck_warning(name: str, tmp_path: Path) -> None:
    out, res = _convert(name, tmp_path)
    assert not any("wall-clock samples differ" in w for w in res.warnings)
    md = _metadata(out)["radar_cam_2_mcap"]
    assert float(md["crosscheck_max_diff_ms"]) <= 50
    assert int(md["crosscheck_samples"]) > 0


def test_segment_gap_event_carries_gap_ns(tmp_path: Path) -> None:
    out, _ = _convert("complete_2seg", tmp_path)
    last0 = read_frame_meta(FIXTURES / "complete_2seg/video/0000.meta.json")[-1].sensor_ts_ns
    first1 = read_frame_meta(FIXTURES / "complete_2seg/video/0001.meta.json")[0].sensor_ts_ns
    gap = first1 - last0
    hits = [m for m in _events(out) if m.ros_msg.msg.startswith("segment gap 0->1")]
    assert len(hits) == 1
    assert hits[0].ros_msg.msg == f"segment gap 0->1: {gap} ns"
    assert hits[0].ros_msg.level == (20 if gap < 10_000_000_000 else 30)
    assert hits[0].log_time_ns == max(_log_times(out))


@pytest.mark.parametrize("name", ["complete_2seg", "in_progress_unlisted", "interrupted"])
def test_log_time_non_decreasing_with_events(name: str, tmp_path: Path) -> None:
    out, _ = _convert(name, tmp_path)
    times = _log_times(out)
    assert times == sorted(times)
    assert _topic_counts(out)["/events"] > 0


def _append_radar(src: Path, text: str) -> None:
    csv = sorted((src / "radar").glob("*.csv"))[-1]
    csv.write_text(csv.read_text() + text)


def test_radar_summary_event_lists_nonzero_counters(tmp_path: Path) -> None:
    src = _copy("complete_2seg", tmp_path)
    _append_radar(src, "999999999999999,zz\n")  # garbled hex line
    out, res = _convert_path(src, tmp_path)
    hits = [m for m in _events(out) if m.ros_msg.msg.startswith("radar decoding problems")]
    assert len(hits) == 1
    assert "garbled_lines=1" in hits[0].ros_msg.msg
    assert "framing_errors" not in hits[0].ros_msg.msg
    assert hits[0].ros_msg.level == 30
    assert any("garbled_lines=1" in w for w in res.warnings)


def _convert_path(src: Path, tmp_path: Path, **kw: Any) -> tuple[Path, Any]:
    res = convert_session(src, tmp_path / "o", CFG, ConvertOptions(**kw))
    return res.outputs[0], res


def test_recovered_frames_info_event(tmp_path: Path) -> None:
    import ld2451
    from ld2451.protocol import encode_report

    def tgt(a: int) -> ld2451.Target:
        return ld2451.Target(angle_deg=a, distance_m=20, approaching=True, speed_kmh=30, snr=100)

    full = encode_report([tgt(1), tgt(2)])
    short = full[:-9] + full[-4:]
    base = RadarStats()
    dec = RecoveringFrameDecoder(recover=True)
    for seg in load_session(FIXTURES / "complete_2seg").segments:
        if seg.radar:
            for _ in iter_radar_reports(seg.radar, base, decoder=dec):
                pass
    flush_trailing(dec, base)
    n = base.recovered_frames + 1
    src = _copy("complete_2seg", tmp_path)
    _append_radar(src, f"999999999999999,{short.hex()}\n")
    out, _ = _convert_path(src, tmp_path)
    hits = [m for m in _events(out) if m.ros_msg.msg.startswith("recovered")]
    assert len(hits) == 1
    assert hits[0].ros_msg.msg == (
        f"recovered {n} short radar data frames: reported count exceeds targets present"
    )
    assert hits[0].ros_msg.level == 20


def test_per_segment_events_follow_segment_spans(tmp_path: Path) -> None:
    res = convert_session(
        FIXTURES / "complete_2seg", tmp_path / "o", CFG, ConvertOptions(per_segment=True)
    )
    src = _boot_events("complete_2seg")
    session = load_session(FIXTURES / "complete_2seg")
    from radar_cam_2_mcap.timebase import Timeline

    tl = Timeline(session.anchors, "wall")
    start1 = read_frame_meta(FIXTURES / "complete_2seg/video/0001.meta.json")[0].sensor_ts_ns
    expect0 = [e for e in src if e["boottime_ns"] < start1]
    expect1 = [e for e in src if e["boottime_ns"] >= start1]
    assert expect0 and expect1
    for path, expect in zip(res.outputs, (expect0, expect1), strict=True):
        got = [m for m in _events(path) if m.ros_msg.name == "radar_cam_recorder"]
        assert [m.log_time_ns for m in got] == [tl.to_log_ns(e["boottime_ns"]) for e in expect]
        times = _log_times(path)
        assert times == sorted(times)
    gap = [m for m in _events(res.outputs[1]) if m.ros_msg.msg.startswith("segment gap 0->1")]
    assert len(gap) == 1
    assert not [m for m in _events(res.outputs[0]) if m.ros_msg.msg.startswith("segment gap")]


def test_event_without_boottime_skipped_with_warning(tmp_path: Path) -> None:
    src = _copy("complete_2seg", tmp_path)
    ev = src / "events.jsonl"
    ev.write_text(ev.read_text() + json.dumps({"kind": "incident", "reason": "no clock"}) + "\n")
    out, res = _convert_path(src, tmp_path)
    assert any("without boottime_ns" in w for w in res.warnings)
    msgs = _events(out)
    assert not [m for m in msgs if m.ros_msg.msg.startswith("no clock")]
    rec = [m for m in msgs if m.ros_msg.name == "radar_cam_recorder"]
    assert len(rec) == len(_boot_events("complete_2seg"))
    assert [m for m in msgs if "without boottime_ns" in m.ros_msg.msg]


_UNSYNCED_MSG = (
    "session time is unsynced: wall-clock timestamps are only as good as the Pi's clock "
    "at recording time"
)
_PINNED = {
    "complete_2seg": (["segment gap 0->1: 299632291000 ns"], COMPLETE_NOTES),
    "in_progress_unlisted": (
        [
            "session is still in progress (recorder did not close it)",
            "segment 3 is on disk but not listed in session.json",
            "boot time follows the last clock anchor (no stop anchor?); using its constant offset",
            _UNSYNCED_MSG,
            "segment gap 0->1: 282269616000 ns",
            "segment gap 1->2: 300550242000 ns",
            "segment gap 2->3: 300713524000 ns",
        ],
        [],
    ),
    "interrupted": (
        [
            "session was interrupted; the last segment may be truncated",
            "segment 1 is on disk but not listed in session.json",
            "boot time precedes the first clock anchor; using its constant offset",
            "boot time follows the last clock anchor (no stop anchor?); using its constant offset",
            _UNSYNCED_MSG,
            "segment gap 0->1: 287020921000 ns",
        ],
        [],
    ),
}


@pytest.mark.parametrize("name", list(_PINNED))
def test_fixture_warning_note_split_is_pinned(name: str, tmp_path: Path) -> None:
    out, res = _convert(name, tmp_path)
    warnings, notes = _PINNED[name]
    assert res.warnings == warnings
    assert res.notes == notes
    md = _metadata(out)["radar_cam_2_mcap"]
    assert json.loads(md["warnings"]) == warnings
    assert json.loads(md["notes"]) == notes
    levels = {m.ros_msg.msg: m.ros_msg.level for m in _events(out)}
    for w in warnings:
        assert levels[w] == 30
    for n in notes:
        assert levels[n] == 20


def test_radar_summary_with_only_trailing_bytes_is_a_note(tmp_path: Path) -> None:
    _out, res = _convert("complete_2seg", tmp_path)
    assert "radar decoding problems: trailing_bytes=12" in res.notes
    assert not any("radar decoding" in w for w in res.warnings)


def test_radar_summary_with_framing_errors_is_a_warning(tmp_path: Path) -> None:
    src = _copy("complete_2seg", tmp_path)
    # data header, length 2, two payload bytes, wrong footer
    # then an unfinished frame at the very end of the stream (trailing bytes)
    _append_radar(src, "999999999999999,f4f3f2f1020000000badf00d\n999999999999999,f4f3f2f11000\n")
    out, res = _convert_path(src, tmp_path)
    hits = [w for w in res.warnings if w.startswith("radar decoding problems")]
    assert len(hits) == 1
    assert "framing_errors=" in hits[0] and "trailing_bytes=" in hits[0]
    assert hits[0] not in res.notes
    levels = [m.ros_msg.level for m in _events(out) if m.ros_msg.msg == hits[0]]
    assert levels == [30]


def test_per_file_notes_metadata(tmp_path: Path) -> None:
    res = convert_session(
        FIXTURES / "complete_2seg", tmp_path / "o", CFG, ConvertOptions(per_segment=True)
    )
    n0, n1 = (json.loads(_metadata(p)["radar_cam_2_mcap"]["notes"]) for p in res.outputs)
    assert set(n0) <= set(n1) <= set(res.notes)
    assert res.notes == n1  # metadata lists accumulate over the run
    assert any(n.startswith("recovered") for n in res.notes)


def test_first_sample_not_suppressed_by_missing_wall_clock(tmp_path: Path) -> None:
    src = _copy("complete_2seg", tmp_path)
    meta = src / "video" / "0000.meta.json"
    data = json.loads(meta.read_text())
    del data[0]["FrameWallClock"]
    meta.write_text(json.dumps(data))
    out, _ = _convert_path(src, tmp_path)
    md = _metadata(out)["radar_cam_2_mcap"]
    base_tmp = tmp_path / "base"
    base_tmp.mkdir()
    _, _ = _convert("complete_2seg", base_tmp)
    base = _metadata(next((base_tmp / "out").glob("*.mcap")))["radar_cam_2_mcap"]
    # frames 0 (no clock, skipped) and 1 (first with a clock: sampled) and 25 -> same count
    # as the unmodified run, where frame 0 is the segment's first sample
    assert md["crosscheck_samples"] == base["crosscheck_samples"]


def _rename_segment(src: Path, old: int, new: int) -> None:
    for f in list((src / "video").glob(f"{old:04d}.*")) + list(
        (src / "radar").glob(f"{old:04d}.*")
    ):
        f.rename(f.with_name(f.name.replace(f"{old:04d}", f"{new:04d}", 1)))


def test_gap_label_uses_actual_index_when_segment_missing(tmp_path: Path) -> None:
    src = _copy("complete_2seg", tmp_path)
    _rename_segment(src, 1, 2)
    _out, res = _convert_path(src, tmp_path)
    assert any(w.startswith("segment gap 0->2: ") for w in res.warnings + res.notes)
    assert not any("segment gap 0->1" in w for w in res.warnings + res.notes)


def test_zero_frame_segment_is_skipped_in_gap_labels(tmp_path: Path) -> None:
    src = _copy("complete_2seg", tmp_path)
    _rename_segment(src, 1, 2)
    # segment 1: radar and meta only, no h264, hence no frames
    shutil.copy(src / "video" / "0002.meta.json", src / "video" / "0001.meta.json")
    shutil.copy(src / "radar" / "0002.csv", src / "radar" / "0001.csv")
    out, res = _convert_path(src, tmp_path)
    segs = {s["index"]: s for s in json.loads(_metadata(out)["segments"]["segments"])}
    assert segs[1]["frames"] == 0 and segs[1]["radar_reports"] > 0
    gaps = [w for w in res.warnings + res.notes if w.startswith("segment gap")]
    assert len(gaps) == 1 and gaps[0].startswith("segment gap 0->2: ")


def test_temp_file_name_is_unique_per_writer(tmp_path: Path) -> None:
    """A temp file at the old fixed name (another writer's) is never opened or replaced."""
    out = tmp_path / "o"
    out.mkdir()
    other = out / "s000066.mcap.tmp"
    other.write_bytes(b"another writer")
    res = convert_session(FIXTURES / "complete_2seg", out, CFG, ConvertOptions())
    assert other.read_bytes() == b"another writer"
    assert res.outputs == [out / "s000066.mcap"]
    assert sorted(p.name for p in out.iterdir()) == ["s000066.mcap", "s000066.mcap.tmp"]


def test_output_file_mode_follows_umask(tmp_path: Path) -> None:
    import os
    import stat

    old = os.umask(0o022)
    try:
        res = convert_session(FIXTURES / "interrupted", tmp_path / "o", CFG, ConvertOptions())
    finally:
        os.umask(old)
    assert stat.S_IMODE(res.outputs[0].stat().st_mode) == 0o644


def test_camera_info_size_mismatch_warns_once(tmp_path: Path) -> None:
    from radar_cam_2_mcap.config import CameraIntrinsics

    cfg = Config(camera=CameraIntrinsics(width=640, height=480))
    res = convert_session(FIXTURES / "complete_2seg", tmp_path / "o", cfg, ConvertOptions())
    hits = [w for w in res.warnings if "differs from camera_info" in w]
    assert hits == [
        "decoded frame size 1296x972 differs from camera_info 640x480 ([camera] width/height); "
        "intrinsics may not match the images"
    ]
    assert not any(
        "differs from camera_info" in w
        for w in convert_session(
            FIXTURES / "complete_2seg", tmp_path / "o2", CFG, ConvertOptions()
        ).warnings
    )


def test_frame_count_mismatch_found_before_any_writing(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The bad segment is the last one; the pre-pass fails before segment 0 is decoded."""
    from radar_cam_2_mcap import convert

    src = _copy("complete_2seg", tmp_path)
    meta = src / "video" / "0001.meta.json"
    meta.write_text(json.dumps([m.fields for m in read_frame_meta(meta)][:-3]))

    def no_write(*a: object, **k: object) -> None:
        raise AssertionError("conversion started before the frame-count check")

    monkeypatch.setattr(convert._Run, "write_file", no_write)
    with pytest.raises(FrameCountMismatch, match="segment 0001"):
        convert_session(src, tmp_path / "o", CFG, ConvertOptions())
    with pytest.raises(AssertionError):  # --allow-prefix skips the pre-pass
        convert_session(src, tmp_path / "o", CFG, ConvertOptions(allow_prefix=True))


def test_per_segment_without_segments_fails_clearly(tmp_path: Path) -> None:
    from radar_cam_2_mcap.session import SessionError

    src = _copy("interrupted", tmp_path)
    shutil.rmtree(src / "video")
    shutil.rmtree(src / "radar")
    out = tmp_path / "o"
    with pytest.raises(SessionError, match="no convertible segments"):
        convert_session(src, out, CFG, ConvertOptions(per_segment=True))
    assert not out.exists() or list(out.iterdir()) == []


def test_failure_mid_write_removes_temp_and_keeps_existing(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The pre-pass now catches count mismatches, so force a failure while writing."""
    from radar_cam_2_mcap import mapping

    out = tmp_path / "o"
    convert_session(FIXTURES / "complete_2seg", out, CFG, ConvertOptions())
    good = (out / "s000066.mcap").read_bytes()
    real = mapping.compressed_image
    calls = itertools.count()

    def flaky(*a: Any, **k: Any) -> Any:
        if next(calls) == 30:
            raise RuntimeError("disk on fire")
        return real(*a, **k)

    monkeypatch.setattr(mapping, "compressed_image", flaky)
    with pytest.raises(RuntimeError, match="disk on fire"):
        convert_session(FIXTURES / "complete_2seg", out, CFG, ConvertOptions(overwrite=True))
    assert (out / "s000066.mcap").read_bytes() == good
    assert sorted(p.name for p in out.iterdir()) == ["s000066.mcap"]
