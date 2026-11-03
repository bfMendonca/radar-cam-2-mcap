from __future__ import annotations

import json
from pathlib import Path

import pytest

from radar_cam_2_mcap.session import SessionError, load_events, load_session

FIXTURES = Path(__file__).parent / "fixtures"


def test_complete_session_lists_segments() -> None:
    info = load_session(FIXTURES / "complete_2seg")
    assert info.session_id == "s000066"
    assert info.state == "complete"
    assert info.listed_segment_indexes == [0, 1]
    assert [s.index for s in info.segments] == [0, 1]
    for seg in info.segments:
        assert seg.h264 is not None and seg.h264.is_file()
        assert seg.meta is not None and seg.meta.is_file()
        assert seg.radar is not None and seg.radar.is_file()
    assert info.warnings == []
    assert info.ntp_synced is True
    assert info.wall_clock_source is None
    assert info.anchors[0].label == "start"
    assert info.anchors[0].boottime_ns == 246242494032


def test_unlisted_segment_is_found_with_warning() -> None:
    info = load_session(FIXTURES / "in_progress_unlisted")
    assert info.state == "in_progress"
    assert info.listed_segment_indexes == [0, 1, 2]
    assert [s.index for s in info.segments] == [0, 1, 2, 3]
    assert any("3" in w and "session.json" in w for w in info.warnings)


def test_interrupted_session_loads() -> None:
    info = load_session(FIXTURES / "interrupted")
    assert info.state == "interrupted"
    assert info.stop_reason == "interrupted (found at next start)"
    assert [s.index for s in info.segments] == [0, 1]


def test_missing_optional_keys_do_not_crash(tmp_path: Path) -> None:
    (tmp_path / "session.json").write_text(
        json.dumps({"format_version": 1, "session_id": "s000001", "state": "complete"})
    )
    info = load_session(tmp_path)
    assert info.wall_clock_source is None
    assert info.devices == {}
    assert info.anchors == []
    assert info.segments == []
    assert info.ntp_synced is False


def test_unknown_format_version_refused(tmp_path: Path) -> None:
    (tmp_path / "session.json").write_text(json.dumps({"format_version": 2}))
    with pytest.raises(SessionError, match="2"):
        load_session(tmp_path)


def test_events_tolerate_partial_last_line(tmp_path: Path) -> None:
    path = tmp_path / "events.jsonl"
    path.write_text('{"kind": "state", "n": 1}\n{"kind": "incident", "n": 2}\n{"kind": "inc')
    assert [e["n"] for e in load_events(path)] == [1, 2]
    assert load_events(tmp_path / "missing.jsonl") == []
