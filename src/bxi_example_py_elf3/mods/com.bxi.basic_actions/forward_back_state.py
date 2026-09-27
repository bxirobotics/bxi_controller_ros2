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

    def __init__(
        self, name, state_id, policy, *, speed=0.5, backward_speed=0.5,
        segment_sec=2.0, backward_segment_sec=4.0,
    ):
        super().__init__(name, state_id, policy)
        if not math.isfinite(speed) or not 0.0 < speed <= 1.0:
            raise ValueError("forward_back speed must be in (0, 1]")
        if not math.isfinite(backward_speed) or not 0.0 < backward_speed <= 1.0:
            raise ValueError("forward_back backward_speed must be in (0, 1]")
        if not math.isfinite(segment_sec) or segment_sec <= 0.0:
            raise ValueError("forward_back segment_sec must be positive")
        if not math.isfinite(backward_segment_sec) or backward_segment_sec <= 0.0:
            raise ValueError("forward_back backward_segment_sec must be positive")
        self.speed = float(speed)
        self.backward_speed = float(backward_speed)
        self.segment_sec = float(segment_sec)
        self.backward_segment_sec = float(backward_segment_sec)
        self._entered_at: float | None = None

    def on_bind(self, ctx: RobotControlContext) -> None:
        self.logger.info("forward/back state uses internal velocity; joystick and /cmd_vel ignored")

    def on_enter(self, ctx: RobotControlContext) -> None:
        super().on_enter(ctx)
        self._entered_at = time.monotonic()
        self.logger.info(
            f"forward/back motion started: forward=+{self.speed:.3f}, "
            f"backward=-{self.backward_speed:.3f}, "
            f"forward_segment={self.segment_sec:.3f}s, "
            f"backward_segment={self.backward_segment_sec:.3f}s"
        )

    def get_cmd_vel(self, ctx: RobotControlContext) -> np.ndarray:
        elapsed = 0.0 if self._entered_at is None else max(
            0.0, time.monotonic() - self._entered_at
        )
        phase = elapsed % (self.segment_sec + self.backward_segment_sec)
        speed = self.speed if phase < self.segment_sec else -self.backward_speed
        self._cmd_vel_buffer[:] = (speed, 0.0, 0.0)
        return self._publish_cmd_vel(ctx, self._cmd_vel_buffer)

    def on_action(self, ctx: RobotControlContext, action_name: str) -> bool:
        return False
