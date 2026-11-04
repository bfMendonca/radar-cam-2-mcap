"""Radar CSV decoding: ``t_rx_ns,hex`` reads to ``RadarReport`` through the ld2451 driver."""

from __future__ import annotations

import logging
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path

import ld2451
from ld2451.protocol import decode_report
from ld2451.recorder import parse_line

__all__ = ["RadarStats", "iter_radar_reports"]

log = logging.getLogger(__name__)


@dataclass
class RadarStats:
    """Counters for radar decoding. Complete once the report generator is exhausted.

    ``framing_errors`` is the decoder's bad footers plus bad lengths: resync events, not
    distinct frames. ``bad_payloads`` counts frames whose payload ``decode_report`` rejected.
    ``partial_lines`` counts a last line without a newline, ``garbled_lines`` lines whose hex
    did not parse; both are skipped.
    """

    reads: int = 0
    data_frames: int = 0
    ack_frames: int = 0
    bytes_discarded: int = 0
    framing_errors: int = 0
    bad_payloads: int = 0
    partial_lines: int = 0
    garbled_lines: int = 0


def iter_radar_reports(
    path: Path,
    stats: RadarStats,
    *,
    decoder: ld2451.FrameDecoder | None = None,
) -> Iterator[ld2451.RadarReport]:
    """Yield a ``RadarReport`` for each data frame in one radar CSV segment.

    Each line's bytes go to a frame decoder with the line's ``t_rx_ns``, so a frame split
    across reads carries the time of the read holding its first byte. Command (ACK) frames
    are counted, not yielded.

    Pass one shared ``decoder`` for consecutive segment files: the recorder rotates files
    mid-stream and a frame can straddle two of them. Without it a fresh decoder is used.

    ``stats`` accumulates: this call adds its own deltas (decoder counters are cumulative
    when the decoder is shared), so reusing one ``RadarStats`` across files gives totals.
    The header, blank and comment lines are skipped silently. A garbled line (unparsable
    hex) is skipped with a logged warning and counted in ``garbled_lines``. A last line
    without a trailing newline (power cut mid-write) is ignored and counted in
    ``partial_lines``. Closing the generator early is treated as an abort: the deltas
    gathered so far are still added, but the file is not fully accounted for.
    """
    dec = decoder if decoder is not None else ld2451.FrameDecoder()
    start = (dec.bytes_discarded, dec.bad_footers, dec.bad_lengths, dec.command_frames)
    try:
        with open(path, encoding="ascii", errors="replace", newline="") as fh:
            for lineno, line in enumerate(fh, 1):
                if not line.endswith("\n"):
                    log.warning("%s:%d: partial last line ignored", path, lineno)
                    stats.partial_lines += 1
                    break
                try:
                    record = parse_line(line)
                except ValueError:
                    log.warning("%s:%d: garbled radar line, skipped", path, lineno)
                    stats.garbled_lines += 1
                    continue
                if record is None:
                    continue
                t_rx_ns, data = record
                stats.reads += 1
                for frame in dec.feed(data, t_rx_ns):
                    if frame.kind is not ld2451.FrameKind.DATA:
                        continue
                    try:
                        report = decode_report(frame.payload, frame.t_rx_ns, frame.raw)
                    except (ld2451.LD2451Error, ValueError):
                        stats.bad_payloads += 1
                        continue
                    stats.data_frames += 1
                    yield report
    finally:
        stats.bytes_discarded += dec.bytes_discarded - start[0]
        stats.framing_errors += (dec.bad_footers - start[1]) + (dec.bad_lengths - start[2])
        stats.ack_frames += dec.command_frames - start[3]
