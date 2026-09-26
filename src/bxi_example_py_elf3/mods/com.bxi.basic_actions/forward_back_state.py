from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING

import numpy as np

from .normal_state import NormalState

if TYPE_CHECKING:
    from bxi_example_py_elf3.framework.mod_api import RobotControlContext


class ForwardBackState(NormalState):
    """Run the normal gait policy with an internal alternating velocity command."""

    def __init__(self, name, state_id, policy, *, speed=0.2, segment_sec=2.0):
        super().__init__(name, state_id, policy)
        if not math.isfinite(speed) or not 0.0 < speed <= 1.0:
            raise ValueError("forward_back speed must be in (0, 1]")
        if not math.isfinite(segment_sec) or segment_sec <= 0.0:
            raise ValueError("forward_back segment_sec must be positive")
        self.speed = float(speed)
        self.segment_sec = float(segment_sec)
        self._entered_at: float | None = None

    def on_bind(self, ctx: RobotControlContext) -> None:
        self.logger.info("forward/back state uses internal velocity; joystick and /cmd_vel ignored")

    def on_enter(self, ctx: RobotControlContext) -> None:
        super().on_enter(ctx)
        self._entered_at = time.monotonic()
        self.logger.info(
            f"forward/back motion started: speed=+/-{self.speed:.3f}, "
            f"segment={self.segment_sec:.3f}s"
        )

    def get_cmd_vel(self, ctx: RobotControlContext) -> np.ndarray:
        elapsed = 0.0 if self._entered_at is None else max(
            0.0, time.monotonic() - self._entered_at
        )
        direction = 1.0 if int(elapsed / self.segment_sec) % 2 == 0 else -1.0
        self._cmd_vel_buffer[:] = (direction * self.speed, 0.0, 0.0)
        return self._publish_cmd_vel(ctx, self._cmd_vel_buffer)

    def on_action(self, ctx: RobotControlContext, action_name: str) -> bool:
        return False
