"""Control-loop configuration, usable without starting or importing ROS."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math

from .cpu_affinity import CpuAffinityRole, CpuAffinitySpec, read_cpu_affinity


@dataclass(frozen=True)
class ControlRuntimeConfig:
    """Validated scheduler and supervision settings."""

    period_sec: float = 0.02
    compute_budget_sec: float = 0.002
    deadline_tolerance_sec: float = 0.001
    maintenance_hz: float = 5.0
    statistics_interval_sec: float = 30.0
    deadline_warning_interval_sec: float = 1.0
    maintenance_guard_sec: float = 0.005
    python_switch_interval_sec: float = 0.001
    cpu_affinity: CpuAffinitySpec = CpuAffinitySpec(
        role=CpuAffinityRole.CONTROL
    )
    realtime_priority: int = 0

    @classmethod
    def from_mapping(cls, raw: object) -> "ControlRuntimeConfig":
        if raw is None:
            raw = {}
        if not isinstance(raw, Mapping):
            raise ValueError("control_runtime must be a YAML map")

        allowed = {
            "period_sec",
            "compute_budget_sec",
            "deadline_tolerance_sec",
            "maintenance_hz",
            "statistics_interval_sec",
            "deadline_warning_interval_sec",
            "maintenance_guard_sec",
            "python_switch_interval_sec",
            "cpu_affinity",
            "realtime_priority",
        }
        unknown = set(raw) - allowed
        if unknown:
            raise ValueError(
                f"control_runtime contains unknown fields: {sorted(unknown)}"
            )

        defaults = cls()
        config = cls(
            period_sec=_number(raw, "period_sec", defaults.period_sec),
            compute_budget_sec=_number(
                raw, "compute_budget_sec", defaults.compute_budget_sec
            ),
            deadline_tolerance_sec=_number(
                raw,
                "deadline_tolerance_sec",
                defaults.deadline_tolerance_sec,
            ),
            maintenance_hz=_number(raw, "maintenance_hz", defaults.maintenance_hz),
            statistics_interval_sec=_number(
                raw,
                "statistics_interval_sec",
                defaults.statistics_interval_sec,
            ),
            deadline_warning_interval_sec=_number(
                raw,
                "deadline_warning_interval_sec",
                defaults.deadline_warning_interval_sec,
            ),
            maintenance_guard_sec=_number(
                raw,
                "maintenance_guard_sec",
                defaults.maintenance_guard_sec,
            ),
            python_switch_interval_sec=_number(
                raw,
                "python_switch_interval_sec",
                defaults.python_switch_interval_sec,
            ),
            cpu_affinity=read_cpu_affinity(
                raw.get("cpu_affinity"),
                "control_runtime.cpu_affinity",
                default=CpuAffinityRole.CONTROL,
            ),
            realtime_priority=_integer(
                raw, "realtime_priority", defaults.realtime_priority
            ),
        )
        config._validate()
        return config

    def _validate(self) -> None:
        if self.period_sec <= 0.0:
            raise ValueError("control_runtime.period_sec must be greater than zero")
        if not 0.0 <= self.compute_budget_sec < self.period_sec:
            raise ValueError(
                "control_runtime.compute_budget_sec must be non-negative and "
                "less than period_sec"
            )
        if self.deadline_tolerance_sec < 0.0:
            raise ValueError(
                "control_runtime.deadline_tolerance_sec must be non-negative"
            )
        if self.maintenance_hz <= 0.0:
            raise ValueError("control_runtime.maintenance_hz must be greater than zero")
        if self.statistics_interval_sec <= 0.0:
            raise ValueError(
                "control_runtime.statistics_interval_sec must be greater than zero"
            )
        if self.deadline_warning_interval_sec <= 0.0:
            raise ValueError(
                "control_runtime.deadline_warning_interval_sec must be greater "
                "than zero"
            )
        if not 0.0 <= self.maintenance_guard_sec < self.period_sec:
            raise ValueError(
                "control_runtime.maintenance_guard_sec must be non-negative and "
                "less than period_sec"
            )
        if self.python_switch_interval_sec <= 0.0:
            raise ValueError(
                "control_runtime.python_switch_interval_sec must be greater than zero"
            )
        if not 0 <= self.realtime_priority <= 99:
            raise ValueError("control_runtime.realtime_priority must be in [0, 99]")


def _number(raw: Mapping[object, object], name: str, default: float) -> float:
    value = raw.get(name, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"control_runtime.{name} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"control_runtime.{name} must be finite")
    return result


def _integer(raw: Mapping[object, object], name: str, default: int) -> int:
    value = raw.get(name, default)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"control_runtime.{name} must be an integer")
    return int(value)
