from __future__ import annotations

from pathlib import Path

import pytest
from mcap.reader import make_reader
from mcap_ros2.reader import read_ros2_messages

from radar_cam_2_mcap.config import McapConfig
from radar_cam_2_mcap.mapping import STAMP, radar_status
from radar_cam_2_mcap.sink import McapSink


def _log_msg(text: str) -> dict[str, object]:
    return {
        "stamp": STAMP(5),
        "level": 20,
        "name": "x",
        "msg": text,
        "file": "",
        "function": "",
        "line": 0,
    }


def test_sink_roundtrip(tmp_path: Path) -> None:
    path = tmp_path / "a.mcap"
    with McapSink(path, McapConfig()) as sink:
        sink.register("/events", "rcl_interfaces/Log")
        sink.write("/events", _log_msg("hello"), 1_000_000_123)
        sink.write("/events", _log_msg("again"), 2_000_000_000, 2_000_000_001)
        sink.add_metadata("m", {"k": "v"})
    msgs = list(read_ros2_messages(path))
    assert [m.ros_msg.msg for m in msgs] == ["hello", "again"]
    assert msgs[0].log_time_ns == 1_000_000_123
    assert msgs[0].publish_time_ns == 1_000_000_123
    assert msgs[1].publish_time_ns == 2_000_000_001
    with path.open("rb") as fh:
        meta = [r for r in make_reader(fh).iter_metadata()]
    assert meta[0].name == "m"
    assert meta[0].metadata == {"k": "v"}


def test_sink_write_unregistered_topic_raises(tmp_path: Path) -> None:
    with McapSink(tmp_path / "a.mcap", McapConfig()) as sink, pytest.raises(KeyError):
        sink.write("/nope", {}, 1)


@pytest.mark.parametrize("compression", ["zstd", "lz4", "none"])
def test_sink_compression_option(tmp_path: Path, compression: str) -> None:
    path = tmp_path / "a.mcap"
    cfg = McapConfig(compression=compression, chunk_size=1024)  # type: ignore[arg-type]
    with McapSink(path, cfg) as sink:
        sink.register("/events", "rcl_interfaces/Log")
        for i in range(50):
            sink.write("/events", _log_msg("x" * 100), 1000 + i)
    with path.open("rb") as fh:
        summary = make_reader(fh).get_summary()
    assert summary is not None
    expected = "" if compression == "none" else compression
    assert summary.chunk_indexes
    assert {c.compression for c in summary.chunk_indexes} == {expected}


def test_sink_all_types_register(tmp_path: Path) -> None:
    stat = radar_status.__name__
    assert stat
    from radar_cam_2_mcap.mapping import MSGDEFS

    with McapSink(tmp_path / "a.mcap", McapConfig()) as sink:
        for i, dt in enumerate(MSGDEFS):
            sink.register(f"/t{i}", dt)
