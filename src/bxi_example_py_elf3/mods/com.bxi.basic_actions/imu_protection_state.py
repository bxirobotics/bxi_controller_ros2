from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from bxi_example_py_elf3.framework.mod_api import RobotControlState
from bxi_example_py_elf3.framework.mod_api.transition import (
    EntryFrameProvider,
    MotorFrame,
    RunningFrameProvider,
)

if TYPE_CHECKING:
    from bxi_example_py_elf3.framework.mod_api import RobotControlContext


class ImuProtectionState(
    RobotControlState, EntryFrameProvider, RunningFrameProvider
):
    """Hold the last joint pose with low stiffness after IMU loss."""

    def __init__(self, name: str, state_id: int, kp: float, kd: float) -> None:
        super().__init__(name, state_id)
        if kp < 0.0 or kd < 0.0:
            raise ValueError("IMU protection kp and kd must be non-negative")
        self._kp_value = np.float32(kp)
        self._kd_value = np.float32(kd)
        self._frame: MotorFrame | None = None

    def on_enter(self, ctx: RobotControlContext) -> None:
        frame = MotorFrame.empty(ctx.robot_layout)
        np.copyto(frame.qpos, ctx.robot_joints.position, casting="same_kind")
        frame.kp.fill(self._kp_value)
        frame.kd.fill(self._kd_value)
        self._frame = frame
        self.logger.error(
            f"IMU protection active: holding current joint pose with "
            f"kp={self._kp_value:.3f}, kd={self._kd_value:.3f}"
        )

    def _current_frame(self, ctx: RobotControlContext) -> MotorFrame:
        if self._frame is None or self._frame.layout != ctx.robot_layout:
            self.on_enter(ctx)
        assert self._frame is not None
        return self._frame

    def get_entry_frame(self, ctx: RobotControlContext) -> MotorFrame:
        return self._current_frame(ctx)

    def sample_running_frame(
        self, ctx: RobotControlContext, dt: float, *, advance: bool
    ) -> MotorFrame:
        return self._current_frame(ctx)

    def on_update(self, ctx: RobotControlContext, dt: float) -> None:
        self._apply_frame(ctx, self._current_frame(ctx))
