"""Constant-space whole-machine CPU sampling for the maintenance thread only."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class CpuUsageSnapshot:
    samples: int
    unavailable_samples: int
    minimum: float | None
    maximum: float | None
    average: float | None


class SystemCpuUsage:
    """Aggregate /proc/stat utilization, normalized to 0–100% across all CPUs.

    Idle and iowait are nonbusy. Guest counters are excluded because Linux
    already includes them in user/nice. The average is weighted by CPU ticks,
    so delayed maintenance samples do not bias the window's mean.
    """

    def __init__(self, path: Path = Path("/proc/stat")) -> None:
        self._path = path
        self._previous: tuple[int, ...] | None = None
        self._reset_window()

    def sample(self) -> None:
        try:
            with self._path.open(encoding="ascii") as stream:
                fields = stream.readline().split()
            if len(fields) < 9 or fields[0] != "cpu":
                raise ValueError("missing aggregate CPU counters")
            counters = tuple(int(value) for value in fields[1:9])
            if any(value < 0 for value in counters):
                raise ValueError("negative CPU counter")
        except (OSError, ValueError):
            self._previous = None
            self._unavailable_samples += 1
            return

        previous = self._previous
        self._previous = counters
        if previous is None:
            return  # Establish a baseline, not utilization since boot.
        deltas = tuple(current - old for current, old in zip(counters, previous))
        total = sum(deltas)
        # Counter resets (including decreasing iowait) invalidate this interval.
        if total <= 0 or any(delta < 0 for delta in deltas):
            self._unavailable_samples += 1
            return
        busy = total - deltas[3] - deltas[4]
        usage = 100.0 * busy / total
        self._samples += 1
        self._busy_ticks += busy
        self._total_ticks += total
        self._minimum = usage if self._minimum is None else min(self._minimum, usage)
        self._maximum = usage if self._maximum is None else max(self._maximum, usage)

    def snapshot(self, *, reset_window: bool = False) -> CpuUsageSnapshot:
        result = CpuUsageSnapshot(
            samples=self._samples,
            unavailable_samples=self._unavailable_samples,
            minimum=self._minimum,
            maximum=self._maximum,
            average=(100.0 * self._busy_ticks / self._total_ticks
                     if self._total_ticks else None),
        )
        if reset_window:
            self._reset_window()
        return result

    def _reset_window(self) -> None:
        # Keep the last counters so adjacent report windows remain continuous.
        self._samples = 0
        self._unavailable_samples = 0
        self._busy_ticks = 0
        self._total_ticks = 0
        self._minimum: float | None = None
        self._maximum: float | None = None
