from __future__ import annotations

import itertools
import logging
import random
from pathlib import Path

import ld2451
from ld2451.protocol import encode_report

from radar_cam_2_mcap.radar import (
    RadarStats,
    RecoveringFrameDecoder,
    flush_trailing,
    iter_radar_reports,
)

FIXTURES = Path(__file__).parent / "fixtures"
HEADER = "t_rx_ns,hex\n"
TARGETS_RECOVERED = 73
DISCARDED_RECOVERED = 19
TRAILING = 17  # the 17th malformed frame, bytes still buffered at EOF


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
    # resync event; those are the cases Task 5b recovers.
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
    assert stats.trailing_bytes == TRAILING
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


def test_bad_frame_then_short_frame_in_one_chunk() -> None:
    t1, t2 = _target(1, 11), _target(2, 22)
    bad = encode_report([t1, t2])[:-4] + b"\x00\x00\x00\x00"
    stream = bad + _short([t1, t2]) + encode_report([t1]) + encode_report([t2])
    dec = RecoveringFrameDecoder()
    frames = dec.feed(stream, 7)
    assert len(frames) == 3
    assert dec.recovered_frames == 1
    assert [len(decode(f).targets) for f in frames] == [1, 1, 1]


def decode(frame: object) -> ld2451.RadarReport:
    from ld2451.framing import RawFrame
    from ld2451.protocol import decode_report

    assert isinstance(frame, RawFrame)
    return decode_report(frame.payload, frame.t_rx_ns, frame.raw)


def test_short_frame_with_many_targets_and_smallest_k() -> None:
    ts = [_target(i, 10 + i) for i in range(5)]
    full = encode_report(ts)
    # 4 targets declared 5, 1 missing: recovered with 4.
    dec = RecoveringFrameDecoder()
    (f, _) = dec.feed(full[:-9] + full[-4:] + encode_report([]), 1)
    assert decode(f).targets == tuple(ts[:4])
    assert decode(f).reported_count == 5
    # Two valid footer positions (k=1 at byte 28, k=2 at byte 23): the smallest k wins, so the
    # recovered frame is 32 bytes, not 27.
    inner = full[:23] + DATA_FOOTER_BYTES + b"\x00" + DATA_FOOTER_BYTES
    tail = encode_report([ts[0]])
    dec2 = RecoveringFrameDecoder()
    frames = dec2.feed(inner + tail, 1)
    assert [len(f.raw) for f in frames] == [32, len(tail)]
    assert dec2.recovered_frames == 1


DATA_FOOTER_BYTES = bytes.fromhex("f8f7f6f5")


def _stream_pieces(rng: random.Random) -> list[bytes]:
    """Frames mixing ok, short, bad-footer, bad-length, garbage and command frames."""
    pieces: list[bytes] = []
    for _ in range(rng.randint(5, 12)):
        n = rng.randint(1, 5)
        ts = [_target(rng.randint(-20, 20), rng.randint(1, 200)) for _ in range(n)]
        full = encode_report(ts)
        kind = rng.choice(["ok", "short", "badfooter", "garbage", "cmd", "badlen"])
        if kind == "ok":
            pieces.append(full)
        elif kind == "short":
            pieces.append(full[:-9] + full[-4:])
        elif kind == "badfooter":
            pieces.append(full[:-4] + b"\x00\x00\x00\x00")
        elif kind == "garbage":
            pieces.append(bytes(rng.randint(0, 255) & 0xEF for _ in range(rng.randint(1, 8))))
        elif kind == "cmd":
            pieces.append(ld2451.protocol.encode_ack(0x00FF, 0))
        else:
            pieces.append(bytes.fromhex("f4f3f2f1ffff"))
    return pieces


def _results(stream: bytes, cuts: list[int]) -> list[tuple[int, int, tuple, bytes]]:
    """(offset of first byte's read start mapped to stream offset, kind, targets, raw)."""
    dec = RecoveringFrameDecoder()
    bounds = [0, *cuts, len(stream)]
    res = []
    for a, b in itertools.pairwise(bounds):
        # t_rx_ns = stream offset of the read start; the expected value is the read holding
        # the frame's first byte, computed independently below.
        for f in dec.feed(stream[a:b], a):
            try:
                rep = decode(f)
                res.append((f.t_rx_ns, rep.reported_count, rep.targets, f.raw))
            except ld2451.LD2451Error:
                res.append((f.t_rx_ns, -1, (), f.raw))
    res.append((-1, dec.recovered_frames, (), b""))
    return res


def test_results_independent_of_chunking() -> None:
    for seed in range(40):
        rng = random.Random(seed)
        stream = b"".join(_stream_pieces(rng))
        one = _results(stream, [])
        # raw frame -> its start offset in the stream (frames are unique enough via search)
        singles = _results(stream, list(range(1, len(stream))))
        rnd_cuts = sorted(rng.sample(range(1, len(stream)), min(10, len(stream) - 1)))
        rand = _results(stream, rnd_cuts)
        strip = lambda r: [(c, t, raw) for _, c, t, raw in r]  # noqa: E731
        assert strip(one) == strip(singles) == strip(rand), seed
        # t_rx_ns: the read carrying the first byte of each frame.
        for res, cuts in ((singles, list(range(1, len(stream)))), (rand, rnd_cuts)):
            starts = [0, *cuts]
            pos = 0
            for t, _, _, raw in res[:-1]:
                pos = stream.index(raw, pos)
                expected = max(s for s in starts if s <= pos)
                assert t == expected, (seed, pos)
                pos += len(raw)
        assert all(t == 0 for t, _, _, _ in one[:-1])


def test_trailing_bytes_for_lone_short_frame_and_shared_decoder(tmp_path: Path) -> None:
    short = _short([_target(), _target(2, 3)])
    path = _write(tmp_path / "r.csv", [(1, short)])
    stats = RadarStats()
    # Complete declared length has not arrived: 5 bytes short, still buffered.
    assert list(iter_radar_reports(path, stats)) == []
    assert stats.trailing_bytes == len(short)
    assert stats.recovered_frames == 0

    shared = RecoveringFrameDecoder()
    stats2 = RadarStats()
    list(iter_radar_reports(path, stats2, decoder=shared))
    assert stats2.trailing_bytes == 0
    flush_trailing(shared, stats2)
    assert stats2.trailing_bytes == len(short)
    assert shared.buffered == 0


def test_conflicting_decoder_setting_rejected(tmp_path: Path) -> None:
    import pytest

    path = _write(tmp_path / "r.csv", [(1, encode_report([]))])
    dec = RecoveringFrameDecoder(recover=False)
    with pytest.raises(ValueError, match="conflicts"):
        list(iter_radar_reports(path, RadarStats(), decoder=dec))
    assert dec.recover is False
    assert list(iter_radar_reports(path, RadarStats(), decoder=dec, recover_short_frames=False))


def test_private_driver_attributes_still_exist() -> None:
    dec = ld2451.FrameDecoder()
    for name in ("_buf", "_base", "_marks", "_discard", "_find_header", "_time_at_base"):
        assert hasattr(dec, name), f"ld2451.FrameDecoder.{name} is gone: the pinned driver changed"
    assert hasattr(dec, "_keep_partial_header_tail"), "pinned ld2451 driver changed"
    for name in ("bytes_discarded", "bad_footers", "bad_lengths", "data_frames", "command_frames"):
        assert hasattr(dec, name), f"ld2451.FrameDecoder.{name} is gone: pinned driver changed"


def _plain_vs_recover_off(reads: list[tuple[int, bytes]]) -> None:
    plain = ld2451.FrameDecoder()
    off = RecoveringFrameDecoder(recover=False)
    for t, data in reads:
        assert plain.feed(data, t) == off.feed(data, t)
    for name in ("bytes_discarded", "bad_footers", "bad_lengths", "data_frames", "command_frames"):
        assert getattr(plain, name) == getattr(off, name), name
    assert plain.buffered == off.buffered


def test_recover_off_identical_to_plain_decoder() -> None:
    src = FIXTURES / "radar_targets" / "radar" / "0000.csv"
    from ld2451.recorder import parse_line

    reads = []
    for line in src.read_text().splitlines(keepends=True):
        rec = parse_line(line)
        if rec is not None:
            reads.append(rec)
    _plain_vs_recover_off(reads)
    for seed in range(300):
        rng = random.Random(seed)
        pieces = _stream_pieces(rng)
        if seed % 2:
            pieces.append(bytes(rng.randrange(256) for _ in range(rng.randint(1, 200))))
        stream = b"".join(pieces)
        reads = []
        pos = 0
        while pos < len(stream):
            n = rng.randint(1, 30)
            reads.append((pos, stream[pos : pos + n]))
            pos += n
        _plain_vs_recover_off(reads)
