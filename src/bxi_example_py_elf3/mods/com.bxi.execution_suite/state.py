"""One playback per entry. Completion and accuracy acceptance are separate."""
import time

import numpy as np

from bxi_example_py_elf3.control.elf3 import JOINT_NAMES
from .observation import sampling, test_event
from bxi_example_py_elf3.framework.mod_api import MotionReplayState, MotorFrame
from .model_test_safety import diagnostic

NORMAL = "com.bxi.basic_actions/normal"
ZERO = "com.bxi.basic_actions/zero_torque"
EXIT_BLEND = dict(type="running_blend", duration=1.0, curve="smootherstep",
                  sample_from=True, sample_to=True, advance_from=False, advance_to=True)
RETURN_NORMAL_BLEND = dict(type="running_blend", duration=0.1, curve="smootherstep",
                           sample_from=False, sample_to=True, advance_from=False, advance_to=True)


class DanceTestState(MotionReplayState):
    def __init__(self, name, state_id, policy, *, recorder, temperature_guard=None):
        super().__init__(name, state_id, policy, finish_state=NORMAL,
                         finish_trigger="dance_playback_complete", end_transition=EXIT_BLEND)
        self.result = "not_started"
        self._last_stamp = None
        self._last_feedback_at = 0.0
        self.temperature_guard = temperature_guard
        self._thermal_notice_sent = False
        self.recorder = recorder
        self._in_update = False

    def on_bind(self, ctx):
        self.recorder.bind(ctx)

    def on_unbind(self, ctx):
        self.recorder.unbind()

    def is_available(self, ctx):
        return getattr(ctx.ros_node, "topic_prefix", "") in ("hardware/", "simulation/")


    def _mark(self, status, reason="", **details):
        severity = "error" if status == "failed" else "warning" if status in ("cancelled", "interrupted") else "info"
        code = ' ['+details['code']+']' if details.get('code') else ''
        message = f"MODEL_TEST {self.name}{code} {status}: {reason}"
        if severity == "error":
            self.logger.error(message)
        elif severity == "warning":
            self.logger.warning(message)
        else:
            self.logger.info(message)
        test_event(self.name, status, reason, severity=severity, **details)

    def on_prepare(self, ctx, from_state):
        if not self.recorder.ready_now():
            self.logger.warning("MODEL_TEST recorder unavailable at entry; capture may be incomplete")
        self.result = "preparing"
        self._last_stamp = ctx.robot_joints.timestamp_ns
        self._last_feedback_at = time.monotonic()
        self._thermal_notice_sent = False
        if self.name.rsplit("/", 1)[-1].startswith("sonic"):
            self.policy.inner.bind_logger(self.logger)
        super().on_prepare(ctx, from_state)

    def on_prepare_cancel(self, ctx, from_state):
        if self.result == "preparing":
            self.result = "cancelled"
            self._mark("cancelled", "entry cancelled before playback", code="ENTRY_CANCELLED", category="operator")

    def on_enter(self, ctx):
        super().on_enter(ctx)
        self.result = "running"
        self._last_stamp = ctx.robot_joints.timestamp_ns
        self._last_feedback_at = time.monotonic()
        self._mark("started")
        if (self.policy.asset_info.get("source_type") != "smpl"
                and not self.policy.asset_info.get("motion_names_observed", False)):
            self.logger.warning("dance NPZ has no joint_names; order follows existing policy/model contract")

    def _fail(self, ctx, reason, *, code=None, category=None, details=None):
        if self.result == "failed":
            return
        self.result = "failed"
        inferred_code, inferred_category = diagnostic(reason)
        self._mark("failed", reason, code=code or inferred_code,
                   category=category or inferred_category, details=details or {})
        # Preserve unavailable-feedback/orientation protections; normal needs
        # a usable observation. A valid-observation model/thermal/accuracy
        # failure returns to normal without continuing the failed action.
        unsafe = ctx.is_orientation_unsafe(ctx.current_quat_xyzw)
        target = ZERO if unsafe or (category or inferred_category) in ("feedback", "sensor") else NORMAL
        ctx.request_state(target, trigger="dance_safety_fault", force=True,
                          transition=RETURN_NORMAL_BLEND if target == NORMAL else None)

    def _check_safety(self, ctx):
        now = time.monotonic()
        if ctx.robot_joints.timestamp_ns != self._last_stamp:
            self._last_stamp = ctx.robot_joints.timestamp_ns
            self._last_feedback_at = now
        if now - self._last_feedback_at > 0.2:
            raise ValueError("joint feedback stale for more than 0.2 s")
        measured = [ctx.robot_joints.position[ctx.robot_layout.index(n)] for n in JOINT_NAMES]
        if not np.isfinite(measured).all():
            raise ValueError("joint feedback contains non-finite angles")
        if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
            raise ValueError("unsafe orientation")
        if self.temperature_guard is not None:
            fault = self.temperature_guard.check(ctx)
            if fault is not None:
                self._fail(ctx, **fault)
                raise ValueError(fault["reason"])
            if (not self._thermal_notice_sent
                    and getattr(ctx.ros_node, "topic_prefix", "") == "simulation/"):
                self._thermal_notice_sent = True
                self._mark("not_assessed", "thermal: MuJoCo has no motor thermal model",
                           code="THERMAL_NOT_SIMULATED", category="temperature")

    def sample_running_frame(self, ctx, dt, *, advance):
        try:
            self._check_safety(ctx)
            with sampling(ctx, self._in_update):
                frame = super().sample_running_frame(ctx, dt, advance=advance)
            return self._validate_frame(frame)
        except Exception as exc:
            # Preview calls also occur inside entry/exit transitions, outside
            # on_update. Never return an invalid frame to a blend/publisher.
            self._fail(ctx, str(exc))
            last = ctx.last_motor_frame
            return MotorFrame.create(last.layout, last.qpos.copy(), last.kp.copy(), last.kd.copy(),
                                     vel=last.vel.copy(), torque=last.torque.copy())

    def _validate_frame(self, frame):
        names = frame.layout.names
        if len(names) != 29 or set(names) != set(JOINT_NAMES):
            raise ValueError("dance target lacks the complete named body29")
        if not all(np.isfinite(a).all() for a in (frame.qpos, frame.kp, frame.kd)):
            raise ValueError("nonfinite dance target/gains")
        if np.any(frame.kp < 0) or np.any(frame.kd < 0):
            raise ValueError("negative dance gains")
        return frame

    def on_update(self, ctx, dt):
        if self.result != "running":
            return
        self._in_update = True
        try:
            frame = self.sample_running_frame(ctx, dt, advance=True)
        finally:
            self._in_update = False
        if self.result != "running":
            return
        self._apply_frame(ctx, frame)
        if self.policy.finished():
            # A complete cursor says nothing about measured precision.
            self.result = "playback_complete"
            self._mark(self.result, "final motion frame consumed")
            if ctx.request_state(NORMAL, trigger=self.finish_trigger, transition=EXIT_BLEND) is False:
                self._fail(ctx, "normal return state unavailable")

    def on_action(self, ctx, action_name):
        if action_name != "cancel":
            return False
        if self.result != "playback_complete":
            self.result = "cancelled"
            self._mark("cancelled", "operator requested normal", code="OPERATOR_RETURN_NORMAL", category="operator")
        if ctx.request_state(NORMAL, trigger="operator_return_normal", transition=RETURN_NORMAL_BLEND) is False:
            self._fail(ctx, "normal return state unavailable")
        return True

    def on_exit(self, ctx):
        if self.result == "running":
            self.result = "interrupted"
            self._mark("interrupted", "state changed before final motion frame")
