from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np

from bxi_example_py_elf3.control.trajectory import minimum_jerk_progress
from bxi_example_py_elf3.policies import HumanoidGaitPolicyLiteIsaaclab
from bxi_example_py_elf3.framework.mod_api import ResourceHandle
from bxi_example_py_elf3.framework.mod_api import RobotControlState
from bxi_example_py_elf3.framework.mod_api.transition import (
    EntryFrameProvider,
    MotorFrame,
    RunningFrameProvider,
)

if TYPE_CHECKING:
    from bxi_example_py_elf3.framework.mod_api import RobotControlContext


class PdBrakeState(RobotControlState, EntryFrameProvider, RunningFrameProvider):
    def __init__(
        self,
        name: str,
        state_id: int,
        policy: ResourceHandle[HumanoidGaitPolicyLiteIsaaclab],
        *,
        startup_ramp_sec: float = 3.0,
        startup_gain_from: float = 0.2,
    ) -> None:
        super().__init__(name, state_id, resources=(policy,))
        if not math.isfinite(startup_ramp_sec) or startup_ramp_sec <= 0.0:
            raise ValueError("pd_brake startup_ramp_sec must be finite and positive")
        if not math.isfinite(startup_gain_from) or not 0.0 < startup_gain_from <= 1.0:
            raise ValueError("pd_brake startup_gain_from must be in (0, 1]")
        self._policy = policy
        self._startup_ramp_sec = float(startup_ramp_sec)
        self._startup_gain_from = float(startup_gain_from)
        self._startup_elapsed = 0.0
        self._startup_position: np.ndarray | None = None
        self._startup_target: MotorFrame | None = None
        self._startup_frame: MotorFrame | None = None

    def on_enter(self, ctx: RobotControlContext) -> None:
        self._startup_frame = None
        if ctx.loop_count != 0:
            return
        self._startup_elapsed = 0.0
        self._startup_position = np.asarray(ctx.robot_joints.position, dtype=np.float32).copy()
        self._startup_target = MotorFrame.empty(ctx.robot_layout)
        ctx.resolve_motor_frame(self._frame(ctx), self._startup_target)
        self._startup_frame = MotorFrame.empty(ctx.robot_layout)
        self.logger.info(
            "PD startup ramp: %.1fs, initial KP/KD %.0f%% of target"
            % (self._startup_ramp_sec, self._startup_gain_from * 100.0)
        )

    def _frame(self, ctx: RobotControlContext) -> MotorFrame:
        policy = self._policy.get()
        return self._motor_frame_from_target(ctx, policy.default_target)

    def get_entry_frame(self, ctx: RobotControlContext) -> MotorFrame:
        return self._frame(ctx)

    def sample_running_frame(
        self, ctx: RobotControlContext, dt: float, *, advance: bool
    ) -> MotorFrame:
        return self._frame(ctx)

    def on_update(self, ctx: RobotControlContext, dt: float) -> None:
        frame = self._startup_frame
        if frame is not None:
            self._startup_elapsed += max(dt, 0.0)
            progress = minimum_jerk_progress(self._startup_elapsed / self._startup_ramp_sec)
            if progress < 1.0:
                target = self._startup_target
                start = self._startup_position
                if target is None or start is None:
                    raise RuntimeError("PD startup ramp target is unavailable")
                np.subtract(target.qpos, start, out=frame.qpos)
                np.multiply(frame.qpos, progress, out=frame.qpos)
                np.add(frame.qpos, start, out=frame.qpos)
                gain_scale = self._startup_gain_from + (1.0 - self._startup_gain_from) * progress
                np.multiply(target.kp, gain_scale, out=frame.kp)
                np.multiply(target.kd, gain_scale, out=frame.kd)
                self._apply_frame(ctx, frame)
                return
            self._startup_frame = None
            self.logger.info("PD startup ramp complete")
        self._apply_frame(ctx, self._frame(ctx))
