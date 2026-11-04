"""Radar CSV decoding: ``t_rx_ns,hex`` reads to ``RadarReport`` through the ld2451 driver."""

from __future__ import annotations

import logging
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path

import ld2451
from ld2451.framing import FrameKind, RawFrame
from ld2451.protocol import (
    CMD_FOOTER,
    DATA_FOOTER,
    MAX_CMD_PAYLOAD,
    MAX_DATA_PAYLOAD,
    TARGET_SIZE,
    decode_report,
)
from ld2451.recorder import parse_line

__all__ = ["RadarStats", "RecoveringFrameDecoder", "flush_trailing", "iter_radar_reports"]

log = logging.getLogger(__name__)


@dataclass
class RadarStats:
    """Counters for radar decoding. Complete once the report generator is exhausted.

    ``framing_errors`` is the decoder's bad footers plus bad lengths: resync events, not
    distinct frames. ``bad_payloads`` counts frames whose payload ``decode_report`` rejected.
    ``partial_lines`` counts a last line without a newline, ``garbled_lines`` lines whose hex
    did not parse; both are skipped. ``recovered_frames`` counts short data frames recovered
    by ``RecoveringFrameDecoder``; they are also in ``data_frames`` and never in
    ``framing_errors``. ``trailing_bytes`` is what was still buffered (an unfinished frame) at
    the end of the stream; see ``flush_trailing``.
    """

    reads: int = 0
    data_frames: int = 0
    ack_frames: int = 0
    bytes_discarded: int = 0
    framing_errors: int = 0
    bad_payloads: int = 0
    partial_lines: int = 0
    garbled_lines: int = 0
    recovered_frames: int = 0
    trailing_bytes: int = 0


_PREFIX = 6  # header + length field
_FOOTER = len(DATA_FOOTER)
_LAYOUT = {
    FrameKind.DATA: (DATA_FOOTER, MAX_DATA_PAYLOAD),
    FrameKind.COMMAND: (CMD_FOOTER, MAX_CMD_PAYLOAD),
}


def _short_frame_end(buf: bytes | bytearray, length: int) -> int | None:
    """Total size of a short data frame that starts at ``buf[0]``, or None.

    A survey of 21 real sessions found about 13% of data frames malformed in 8 sessions, and
    in 93% of those the count byte and length field declare N targets while N-1 are present:
    the footer sits exactly 5 bytes before the declared end. Looks for the footer at
    ``declared_end - 5k`` for k = 1 up to the declared target count and accepts the smallest k
    whose payload (count byte, alarm byte, targets) is whole 5-byte targets. The caller has
    already established that the footer is not at the declared end.
    """
    declared_end = _PREFIX + length + _FOOTER
    for k in range(1, max(0, (length - 2) // TARGET_SIZE) + 1):
        start = declared_end - _FOOTER - TARGET_SIZE * k
        # Alignment: only holds when length - 2 is itself a multiple of 5 (else never).
        if (start - _PREFIX - 2) % TARGET_SIZE == 0 and buf[start : start + _FOOTER] == DATA_FOOTER:
            return start + _FOOTER
    return None


class RecoveringFrameDecoder(ld2451.FrameDecoder):
    """``FrameDecoder`` that also yields short data frames (see ``_short_frame_end``).

    Before the base class would count a bad footer and resync, a data frame whose declared
    length has fully arrived but whose footer is missing there is checked for an early footer.
    If one is found the frame is yielded (payload = the bytes actually present, so the
    radar's original count byte stays in it) and consumed without counting a framing error;
    the following frame is untouched and keeps the time of its own read. ``recovered_frames``
    counts these. Relies on the base class's private buffer helpers (driver pinned in
    pyproject); with ``recover = False`` it behaves exactly like ``FrameDecoder``.
    """

    def __init__(
        self, max_buffer: int = ld2451.framing.DEFAULT_MAX_BUFFER, *, recover: bool = True
    ) -> None:
        super().__init__(max_buffer)
        self.recover = recover
        self.recovered_frames = 0

    def _next_frame(self) -> RawFrame | None:
        """The base class's resync loop, with a short-frame check at every header position.

        Mirrors ``FrameDecoder._next_frame`` (bad lengths, bad footers and partial headers are
        handled the same way and counted in the same counters) so the result does not depend on
        how the bytes were chunked into reads.
        """
        while True:
            idx, kind = self._find_header()
            if kind is None:
                self._keep_partial_header_tail()
                return None
            if self.recover and kind is FrameKind.DATA:
                frame = self._try_recover(idx)
                if frame is not None:
                    return frame
            self._discard(idx)
            if len(self._buf) < _PREFIX:
                return None
            footer, max_payload = _LAYOUT[kind]
            length = int.from_bytes(self._buf[4:6], "little")
            if length > max_payload:
                self.bad_lengths += 1
                self._discard(1)
                continue
            total = _PREFIX + length + _FOOTER
            if len(self._buf) < total:
                return None
            if self._buf[total - _FOOTER : total] != footer:
                self.bad_footers += 1
                self._discard(1)
                continue
            raw = bytes(self._buf[:total])
            frame = RawFrame(
                kind=kind,
                payload=raw[_PREFIX : _PREFIX + length],
                raw=raw,
                t_rx_ns=self._time_at_base(),
            )
            self._consume(total)
            if kind is FrameKind.DATA:
                self.data_frames += 1
            else:
                self.command_frames += 1
            return frame

    def _consume(self, n: int) -> None:
        """Remove a decoded frame from the buffer without counting it as discarded."""
        del self._buf[:n]
        self._base += n
        if not self._buf:
            self._marks.clear()

    def _try_recover(self, idx: int) -> RawFrame | None:
        """Recover a short DATA frame whose header is at ``self._buf[idx]``, or None."""
        if len(self._buf) < idx + _PREFIX:
            return None
        length = int.from_bytes(self._buf[idx + 4 : idx + 6], "little")
        total = _PREFIX + length + _FOOTER
        if length > MAX_DATA_PAYLOAD or len(self._buf) < idx + total:
            return None
        if self._buf[idx + total - _FOOTER : idx + total] == DATA_FOOTER:
            return None
        end = _short_frame_end(self._buf[idx:], length)
        if end is None:
            return None
        self._discard(idx)  # garbage before the header, counted as usual
        raw = bytes(self._buf[:end])
        frame = RawFrame(
            kind=FrameKind.DATA,
            payload=raw[_PREFIX : end - _FOOTER],
            raw=raw,
            t_rx_ns=self._time_at_base(),
        )
        self._consume(end)
        self.recovered_frames += 1
        return frame


def flush_trailing(decoder: ld2451.FrameDecoder, stats: RadarStats) -> None:
    """Account for bytes still buffered in a shared ``decoder`` after the last file.

    Adds the buffered count to ``stats.trailing_bytes`` and resets the decoder. The
    orchestrator must call this once after the last segment when it passes its own decoder
    to ``iter_radar_reports`` (which only flushes a decoder it created itself).
    """
    stats.trailing_bytes += decoder.buffered
    decoder.reset()


def iter_radar_reports(
    path: Path,
    stats: RadarStats,
    *,
    decoder: ld2451.FrameDecoder | None = None,
    recover_short_frames: bool = True,
) -> Iterator[ld2451.RadarReport]:
    """Yield a ``RadarReport`` for each data frame in one radar CSV segment.

    Each line's bytes go to a frame decoder with the line's ``t_rx_ns``, so a frame split
    across reads carries the time of the read holding its first byte. Command (ACK) frames
    are counted, not yielded.

    Pass one shared ``decoder`` for consecutive segment files: the recorder rotates files
    mid-stream and a frame can straddle two of them. Without it a fresh decoder is used.

    With ``recover_short_frames`` (default) frames whose count and length overstate the
    targets present by whole targets, footer early, are yielded with the targets actually
    there; ``reported_count`` keeps the radar's count. This needs a ``RecoveringFrameDecoder``
    (created here when ``decoder`` is None). A supplied decoder is never mutated: if it does
    not match ``recover_short_frames`` (a plain ``FrameDecoder`` counts as recovery off) a
    ``ValueError`` is raised. A decoder created here has its leftover buffered bytes added to
    ``trailing_bytes``; for a shared decoder call ``flush_trailing`` after the last file.

    ``stats`` accumulates: this call adds its own deltas (decoder counters are cumulative
    when the decoder is shared), so reusing one ``RadarStats`` across files gives totals.
    The header, blank and comment lines are skipped silently. A garbled line (unparsable
    hex) is skipped with a logged warning and counted in ``garbled_lines``. A last line
    without a trailing newline (power cut mid-write) is ignored and counted in
    ``partial_lines``. Closing the generator early is treated as an abort: the deltas
    gathered so far are still added, but the file is not fully accounted for.
    """
    dec: ld2451.FrameDecoder
    if decoder is None:
        dec = RecoveringFrameDecoder(recover=recover_short_frames)
    else:
        dec = decoder
        have = dec.recover if isinstance(dec, RecoveringFrameDecoder) else False
        if have != recover_short_frames:
            raise ValueError(
                f"recover_short_frames={recover_short_frames} conflicts with the supplied "
                "decoder: use a RecoveringFrameDecoder(recover=...) with the same setting"
            )
    recovered_start = dec.recovered_frames if isinstance(dec, RecoveringFrameDecoder) else 0
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
        if decoder is None:
            stats.trailing_bytes += dec.buffered
        if isinstance(dec, RecoveringFrameDecoder):
            stats.recovered_frames += dec.recovered_frames - recovered_start
