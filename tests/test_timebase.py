from pathlib import Path

import pytest

from radar_cam_2_mcap.session import ClockAnchor, SessionError, SessionInfo
from radar_cam_2_mcap.timebase import Timeline, crosscheck, time_quality

S = 1_000_000_000
MS = 1_000_000
BASE = 1_790_000_000 * S


def anchors() -> list[ClockAnchor]:
    return [
        ClockAnchor("start", 0, BASE),
        ClockAnchor("stop", 300 * S, BASE + 300 * S + MS),
    ]


def make_session(source: str | None, ntp: bool) -> SessionInfo:
    return SessionInfo(
        path=Path("."), session_id="s", state="stopped", stop_reason=None,
        boot_id=None, ntp_synced=ntp, wall_clock_source=source, anchors=[],
        recorder_config={}, devices={}, segments=[], listed_segment_indexes=[],
    )


def test_interpolates_between_anchors() -> None:
    tl = Timeline(anchors())
    assert tl.to_log_ns(150 * S) == BASE + 150 * S + MS // 2
    assert tl.warnings == []


def test_exact_at_large_values() -> None:
    tl = Timeline(anchors())
    assert tl.to_log_ns(0) == BASE
    assert tl.to_log_ns(300 * S) == BASE + 300 * S + MS
    # offset delta 1 ms over 300 s; at 1 s boot -> 1e6/300 = 3333.33 ns, floored
    assert tl.to_log_ns(S) == BASE + S + 3333


def test_before_first_anchor_uses_first_offset_and_warns() -> None:
    tl = Timeline([ClockAnchor("start", 10 * S, BASE + 10 * S)])
    assert tl.to_log_ns(5 * S) == BASE + 5 * S
    tl.to_log_ns(4 * S)
    assert len(tl.warnings) == 1


def test_after_last_anchor_uses_last_offset() -> None:
    tl = Timeline(anchors())
    assert tl.to_log_ns(400 * S) == BASE + 400 * S + MS
    tl.to_log_ns(500 * S)
    assert len(tl.warnings) == 1


def test_unsorted_anchors_are_sorted() -> None:
    tl = Timeline(list(reversed(anchors())))
    assert tl.to_log_ns(150 * S) == BASE + 150 * S + MS // 2


def test_boot_base_returns_boot_ns() -> None:
    tl = Timeline(anchors(), base="boot")
    assert tl.to_log_ns(123) == 123
    assert tl.to_wall_ns(0) == BASE


def test_no_anchors_raises() -> None:
    with pytest.raises(SessionError):
        Timeline([])


def test_time_quality_prefers_recorder_value_then_ntp_then_unsynced() -> None:
    assert time_quality(make_session("rtc", False)) == "rtc"
    assert time_quality(make_session("ntp", False)) == "ntp"
    assert time_quality(make_session(None, True)) == "ntp"
    assert time_quality(make_session("unsynced", True)) == "ntp"
    assert time_quality(make_session("bogus", False)) == "unsynced"
    assert time_quality(make_session(None, False)) == "unsynced"


def test_crosscheck_flags_disagreement_above_tolerance() -> None:
    tl = Timeline([ClockAnchor("start", 0, BASE)])
    t = 10 * S
    wall_us = (BASE + t) // 1000
    assert crosscheck(tl, [(t, wall_us + 60_000)], 50.0) != []
    assert crosscheck(tl, [(t, wall_us + 40_000)], 50.0) == []
    out = crosscheck(tl, [(t, wall_us + 60_000), (t, wall_us - 80_000), (t, wall_us)], 50.0)
    assert len(out) == 1
    assert "2" in out[0] and "80" in out[0]


def test_crosscheck_uses_wall_mapping_when_base_is_boot() -> None:
    tl = Timeline([ClockAnchor("start", 0, BASE)], base="boot")
    t = 10 * S
    assert crosscheck(tl, [(t, (BASE + t) // 1000)], 50.0) == []
