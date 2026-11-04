from __future__ import annotations

import logging
from pathlib import Path

import ld2451
from ld2451.protocol import encode_report

from radar_cam_2_mcap.radar import RadarStats, RecoveringFrameDecoder, iter_radar_reports

FIXTURES = Path(__file__).parent / "fixtures"
HEADER = "t_rx_ns,hex\n"
TARGETS_RECOVERED = 73
DISCARDED_RECOVERED = 19


def _write(path: Path, rows: list[tuple[int, bytes]], tail: str = "") -> Path:
    path.write_text(HEADER + "".join(f"{t},{d.hex()}\n" for t, d in rows) + tail)
    return path


def _target(angle: int = 10, dist: int = 20, approaching: bool = True) -> ld2451.Target:
    return ld2451.Target(
        angle_deg=angle, distance_m=dist, approaching=approaching, speed_kmh=30, snr=100
    )


def test_fragmented_data_frame_decodes_once_with_first_byte_time(tmp_path: Path) -> None:
    frame = encode_report([_target(), _target(-5, 40, False)])
    assert frame[:4] == bytes.fromhex("f4f3f2f1")
    a, b, c = frame[:3], frame[3:9], frame[9:]
    rows = [(1000, a), (2000, b), (3000, c + b"\xf5")]
    stats = RadarStats()
    reports = list(iter_radar_reports(_write(tmp_path / "r.csv", rows), stats))
    assert len(reports) == 1
    assert reports[0].t_rx_ns == 1000
    assert len(reports[0].targets) == 2
    assert stats.reads == 3
    assert stats.data_frames == 1
    assert stats.framing_errors == 0
    assert stats.bad_payloads == 0
    assert stats.bytes_discarded == 1  # the stray f5


def test_command_ack_frames_are_counted_not_reported(tmp_path: Path) -> None:
    # Lines 1-16 of the real fixture are startup command ACKs only (no data frames).
    src = FIXTURES / "radar_targets" / "radar" / "0000.csv"
    lines = src.read_text().splitlines(keepends=True)[:17]
    tmp = tmp_path / "acks.csv"
    tmp.write_text("".join(lines))
    stats = RadarStats()
    reports = list(iter_radar_reports(tmp, stats))
    assert stats.ack_frames > 0
    assert stats.reads == 16
    assert len(reports) == stats.data_frames


def test_empty_frame_yields_report_with_no_targets(tmp_path: Path) -> None:
    stats = RadarStats()
    reports = list(iter_radar_reports(_write(tmp_path / "r.csv", [(5, encode_report([]))]), stats))
    assert len(reports) == 1
    assert reports[0].targets == ()
    assert reports[0].t_rx_ns == 5
    assert stats.data_frames == 1


def test_partial_last_line_ignored(tmp_path: Path) -> None:
    full = encode_report([_target()])
    path = _write(tmp_path / "r.csv", [(1, full)], tail=f"2,{full.hex()[:10]}")
    stats = RadarStats()
    reports = list(iter_radar_reports(path, stats))
    assert len(reports) == 1
    assert stats.reads == 1
    assert stats.partial_lines == 1
    assert stats.garbled_lines == 0

    # Even-length truncation of a full line must not produce a second report.
    path2 = _write(tmp_path / "r2.csv", [(1, full)], tail=f"2,{full.hex()}")
    assert len(list(iter_radar_reports(path2, RadarStats()))) == 1


def test_garbled_line_skipped_with_warning(tmp_path: Path, caplog: object) -> None:
    full = encode_report([_target()])
    path = tmp_path / "r.csv"
    path.write_text(HEADER + f"1,zz\n2,{full.hex()}\n")
    import pytest

    assert isinstance(caplog, pytest.LogCaptureFixture)
    stats = RadarStats()
    with caplog.at_level(logging.WARNING):
        reports = list(iter_radar_reports(path, stats))
    assert len(reports) == 1
    assert stats.garbled_lines == 1
    assert stats.partial_lines == 0
    assert "garbled" in caplog.text


def test_invalid_payload_counted(tmp_path: Path) -> None:
    # 3-byte payload: not empty, shorter than header + target.
    bad = (
        bytes.fromhex("f4f3f2f1")
        + (3).to_bytes(2, "little")
        + b"\x01\x02\x03"
        + bytes.fromhex("f8f7f6f5")
    )
    stats = RadarStats()
    reports = list(
        iter_radar_reports(_write(tmp_path / "r.csv", [(1, bad), (2, encode_report([]))]), stats)
    )
    assert len(reports) == 1
    assert stats.bad_payloads == 1


def test_shared_decoder_joins_frame_across_files(tmp_path: Path) -> None:
    frame = encode_report([_target()])
    file_a = _write(tmp_path / "a.csv", [(100, b"\x00"), (200, frame[:6])])
    file_b = _write(tmp_path / "b.csv", [(300, frame[6:])])

    shared = RecoveringFrameDecoder()
    stats = RadarStats()
    reports = list(iter_radar_reports(file_a, stats, decoder=shared))
    reports += list(iter_radar_reports(file_b, stats, decoder=shared))
    assert len(reports) == 1
    assert reports[0].t_rx_ns == 200
    assert stats.data_frames == 1
    assert stats.reads == 3
    assert stats.bytes_discarded == 1  # the stray 0x00, counted once

    fresh = RadarStats()
    got = list(iter_radar_reports(file_a, fresh)) + list(iter_radar_reports(file_b, fresh))
    assert got == []
    assert fresh.data_frames == 0
    assert fresh.bytes_discarded > 0


def test_real_fixture_decodes_targets() -> None:
    path = FIXTURES / "radar_targets" / "radar" / "0000.csv"
    stats = RadarStats()
    reports = list(iter_radar_reports(path, stats))
    targets = [t for r in reports for t in r.targets]
    # Counts observed once by running iter_radar_reports over the committed fixture.
    # Cross-check: a naive header..footer scan finds 42 spans, 25 with a consistent declared
    # length (decoded here) and 17 whose declared length is 5 bytes (one target) longer than
    # the bytes on the wire. The decoder cannot frame those; each causes a bad-footer resync
    # (16 seen, the 17th is still buffered at end of file). So framing_errors counts resync
    # events, not distinct frames.
    #
    # Task 5b: with short-frame recovery (default) 16 of the 17 short frames are recovered
    # (the 17th is still buffered at end of file: the decoder only judges a frame once its
    # declared length has arrived). Previously pinned without recovery: 25 reports, 41
    # targets, 16 framing_errors, 371 bytes_discarded. Each recovered frame removes one
    # resync event (16 -> 0) and the bytes that resync discarded (371 -> 19: the 352 bytes
    # of the 16 recovered frames are no longer lost).
    assert len(reports) == 41
    assert len(targets) == TARGETS_RECOVERED
    assert stats.recovered_frames == 16
    assert stats.data_frames == 41
    assert stats.ack_frames == 6
    assert stats.reads == 98
    assert stats.framing_errors == 0
    assert stats.bad_payloads == 0
    assert stats.partial_lines == 0
    assert stats.garbled_lines == 0
    assert stats.bytes_discarded == DISCARDED_RECOVERED
    assert targets
    for t in targets:
        assert -128 <= t.angle_deg <= 127
        assert 0 <= t.distance_m <= 255
        assert 0 <= t.speed_kmh <= 255
        assert 0 <= t.snr <= 255


def _short(targets: list[ld2451.Target]) -> bytes:
    """A frame as the radar sometimes sends it: count and length declare len(targets), but the
    last target is missing and the footer comes 5 bytes early."""
    full = encode_report(targets)
    return full[:-9] + full[-4:]


def test_short_frame_is_recovered_and_next_frame_decodes(tmp_path: Path) -> None:
    t1, t2, t3 = _target(1, 11), _target(2, 22), _target(3, 33)
    short = _short([t1, t2])
    assert short.hex() == encode_report([t1, t2])[:-9].hex() + "f8f7f6f5"
    rows = [(100, short), (200, encode_report([t3])), (300, encode_report([t1]))]
    stats = RadarStats()
    reports = list(iter_radar_reports(_write(tmp_path / "r.csv", rows), stats))
    assert [r.t_rx_ns for r in reports] == [100, 200, 300]
    assert reports[0].targets == (t1,)
    assert reports[0].reported_count == 2
    assert reports[0].header_bytes[0] == 2
    assert reports[0].raw == short
    assert reports[1].targets == (t3,)
    assert reports[2].targets == (t1,)
    assert stats.recovered_frames == 1
    assert stats.data_frames == 3
    assert stats.framing_errors == 0
    assert stats.bytes_discarded == 0


def test_short_frame_split_across_reads_keeps_first_byte_time(tmp_path: Path) -> None:
    short = _short([_target(), _target(5, 50)])
    nxt = encode_report([_target(7, 70)])
    stream = short + nxt
    rows = [(10, stream[:5]), (20, stream[5:19]), (30, stream[19:])]
    reports = list(iter_radar_reports(_write(tmp_path / "r.csv", rows), RadarStats()))
    assert [r.t_rx_ns for r in reports] == [10, 20]
    assert len(reports[0].targets) == 1
    assert reports[1].targets[0].distance_m == 70


def test_recovery_disabled_drops_short_frame(tmp_path: Path) -> None:
    rows = [(100, _short([_target(), _target(2, 3)])), (200, encode_report([_target()]))]
    stats = RadarStats()
    reports = list(
        iter_radar_reports(_write(tmp_path / "r.csv", rows), stats, recover_short_frames=False)
    )
    assert [r.t_rx_ns for r in reports] == [200]
    assert stats.recovered_frames == 0
    assert stats.framing_errors >= 1


def test_frame_without_footer_is_not_recovered(tmp_path: Path) -> None:
    full = encode_report([_target(), _target(2, 3)])
    nofooter = full[:-4] + b"\x00\x00\x00\x00"
    rows = [(100, nofooter), (200, encode_report([_target()]))]
    stats = RadarStats()
    reports = list(iter_radar_reports(_write(tmp_path / "r.csv", rows), stats))
    assert [r.t_rx_ns for r in reports] == [200]
    assert stats.recovered_frames == 0
    assert stats.framing_errors >= 1


def test_recovery_needs_whole_targets(tmp_path: Path) -> None:
    # Footer 3 bytes early, not a multiple of the 5-byte target size: not recovered.
    full = encode_report([_target(), _target(2, 3)])
    bad = full[:-7] + full[-4:]
    rows = [(100, bad), (200, encode_report([_target()]))]
    stats = RadarStats()
    reports = list(iter_radar_reports(_write(tmp_path / "r.csv", rows), stats))
    assert [r.t_rx_ns for r in reports] == [200]
    assert stats.recovered_frames == 0
    assert stats.framing_errors >= 1


def test_recovery_with_shared_decoder_across_files(tmp_path: Path) -> None:
    short = _short([_target(), _target(2, 3)])
    nxt = encode_report([_target(4, 5)])
    stream = short + nxt
    a = _write(tmp_path / "a.csv", [(100, stream[:8])])
    b = _write(tmp_path / "b.csv", [(200, stream[8:])])
    shared = RecoveringFrameDecoder()
    stats = RadarStats()
    reports = list(iter_radar_reports(a, stats, decoder=shared))
    reports += list(iter_radar_reports(b, stats, decoder=shared))
    assert [r.t_rx_ns for r in reports] == [100, 200]
    assert stats.recovered_frames == 1


def test_plain_decoder_with_recovery_is_rejected(tmp_path: Path) -> None:
    import pytest

    path = _write(tmp_path / "r.csv", [(1, encode_report([]))])
    with pytest.raises(ValueError, match="RecoveringFrameDecoder"):
        list(iter_radar_reports(path, RadarStats(), decoder=ld2451.FrameDecoder()))
    assert list(
        iter_radar_reports(
            path, RadarStats(), decoder=ld2451.FrameDecoder(), recover_short_frames=False
        )
    )
