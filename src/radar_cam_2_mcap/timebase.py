"""Boot-clock to wall-clock conversion, time quality and cross-checks."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Literal

from radar_cam_2_mcap.session import ClockAnchor, SessionError, SessionInfo


class Timeline:
    """Maps CLOCK_BOOTTIME nanoseconds to wall-clock nanoseconds.

    The offset ``realtime - boottime`` is interpolated piecewise-linearly between
    anchors using integer arithmetic. Outside the anchor range the nearest
    anchor's constant offset is used and a warning is recorded once per direction.
    """

    def __init__(
        self,
        anchors: Sequence[ClockAnchor],
        base: Literal["wall", "boot"] = "wall",
    ) -> None:
        if not anchors:
            raise SessionError("no clock anchors: cannot map boot time to wall time")
        self._anchors = sorted(anchors, key=lambda a: a.boottime_ns)
        self._boots = [a.boottime_ns for a in self._anchors]
        self._offsets = [a.realtime_ns - a.boottime_ns for a in self._anchors]
        self.base = base
        self.warnings: list[str] = []
        self._warned_before = False
        self._warned_after = False

    def _offset_ns(self, boot_ns: int) -> int:
        boots, offsets = self._boots, self._offsets
        if boot_ns < boots[0]:
            if not self._warned_before:
                self._warned_before = True
                self.warnings.append(
                    "boot time precedes the first clock anchor; using its constant offset"
                )
            return offsets[0]
        if boot_ns > boots[-1]:
            if not self._warned_after:
                self._warned_after = True
                self.warnings.append(
                    "boot time follows the last clock anchor (no stop anchor?); "
                    "using its constant offset"
                )
            return offsets[-1]
        lo, hi = 0, len(boots) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if boots[mid] <= boot_ns:
                lo = mid
            else:
                hi = mid
        if boots[hi] == boots[lo]:
            return offsets[lo]
        num = (offsets[hi] - offsets[lo]) * (boot_ns - boots[lo])
        return offsets[lo] + num // (boots[hi] - boots[lo])

    def to_wall_ns(self, boot_ns: int) -> int:
        return boot_ns + self._offset_ns(boot_ns)

    def to_log_ns(self, boot_ns: int) -> int:
        if self.base == "boot":
            return boot_ns
        return self.to_wall_ns(boot_ns)


def time_quality(session: SessionInfo) -> Literal["rtc", "ntp", "unsynced"]:
    if session.wall_clock_source == "rtc":
        return "rtc"
    if session.wall_clock_source == "ntp":
        return "ntp"
    return "ntp" if session.ntp_synced else "unsynced"


def crosscheck(
    timeline: Timeline,
    samples: Sequence[tuple[int, int]],
    tolerance_ms: float,
) -> list[str]:
    """Compare mapped boot times with frame wall-clock stamps; one summary warning."""
    bad = 0
    worst_ms = 0.0
    for boot_ns, wall_us in samples:
        diff_ms = abs(timeline.to_wall_ns(boot_ns) - wall_us * 1000) / 1e6
        if diff_ms > tolerance_ms:
            bad += 1
            worst_ms = max(worst_ms, diff_ms)
    if not bad:
        return []
    return [
        f"{bad} of {len(samples)} frame wall-clock samples differ from the anchor "
        f"timeline by more than {tolerance_ms:g} ms (worst {worst_ms:.1f} ms)"
    ]
