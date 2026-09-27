"""Suspended tests driven by the shared state-machine actuator publisher."""

from __future__ import annotations

import math
import time

import numpy as np
from ament_index_python.packages import get_package_share_path

from bxi_example_py_elf3.control.elf3 import (
    JOINT_KD,
    JOINT_KP,
    JOINT_NAMES,
    JOINT_NOMINAL_POS,
    JOINT_POSITION_MAX,
    JOINT_POSITION_MIN,
    JOINT_VIBRATION_SIGNS,
    SUSPENDED_RUN_NOMINAL_POS,
)
from bxi_example_py_elf3.control.limb_sequence import (
    WHOLE_BODY_TEST_GROUPS,
    build_safe_ranges,
    full_range_waypoints,
    velocity_limited_duration,
)
from bxi_example_py_elf3.control.trajectory import (
    load_joint_trajectory,
    minimum_jerk_progress,
)
from bxi_example_py_elf3.framework.joints import JointLayout
from bxi_example_py_elf3.framework.mod_api import RobotControlState


TEST_LAYOUT = JointLayout(JOINT_NAMES, label="ELF3 suspended tests")
TEST_IDLE = "com.bxi.suspended_tests/idle"
ZERO_TORQUE = "com.bxi.basic_actions/zero_torque"
PD_BRAKE = "com.bxi.basic_actions/pd_brake"
JOINT_MARGIN_RAD = 0.02
FEEDBACK_TIMEOUT_SEC = 0.2
COMMAND_GAP_TIMEOUT_SEC = 0.05
RETURN_SEC = 0.5
IDLE_CENTER_TOLERANCE_RAD = math.radians(5.0)


class TestSession:
    def __init__(self):
        self.ready = False
        self.faulted = False
        self.last_feedback_stamp = None
        self.last_feedback_at = 0.0
        self.last_control_at = 0.0

    def measured(self, ctx):
        now = time.monotonic()
        stamp = ctx.robot_joints.timestamp_ns
        if stamp != self.last_feedback_stamp:
            self.last_feedback_stamp = stamp
            self.last_feedback_at = now
        if now - self.last_feedback_at > FEEDBACK_TIMEOUT_SEC:
            raise ValueError("joint feedback is stale")
        positions = ctx.robot_joints.position
        measured = np.asarray(
            [positions[ctx.robot_layout.index(name)] for name in JOINT_NAMES],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(measured)):
            raise ValueError("joint feedback contains a non-finite position")
        return measured


class SuspendedState(RobotControlState):
    def __init__(self, name, state_id, session):
        super().__init__(name, state_id)
        self.session = session
        self.stopping = False
        self.stop_started_at = 0.0
        self.stop_from = JOINT_NOMINAL_POS.copy()
        self.last_command = JOINT_NOMINAL_POS.copy()

    def is_available(self, ctx):
        return self.session.ready and not self.session.faulted

    def _fault(self, ctx, reason):
        self.session.ready = False
        self.session.faulted = True
        self.logger.error("suspended test fault: %s; switching to zero torque" % reason)
        ctx.request_state(ZERO_TORQUE, trigger="test_safety_fault", force=True)

    def _command(self, ctx, position, kp=JOINT_KP):
        position = np.asarray(position, dtype=np.float64)
        if (
            position.shape != JOINT_NOMINAL_POS.shape
            or not np.all(np.isfinite(position))
            or np.any(position < JOINT_POSITION_MIN + JOINT_MARGIN_RAD)
            or np.any(position > JOINT_POSITION_MAX - JOINT_MARGIN_RAD)
        ):
            self._fault(ctx, "command exceeds finite software joint limits")
            return
        self.last_command[:] = position
        self._apply_frame(
            ctx, self._motor_frame(ctx, position, kp, JOINT_KD, layout=TEST_LAYOUT)
        )

    def on_action(self, ctx, action_name):
        if action_name != "stop" or self.stopping:
            return False
        self.stopping = True
        self.stop_started_at = time.monotonic()
        self.stop_from[:] = self.last_command
        self.logger.info("stopping suspended test; returning to center")
        return True

    def _update_stop(self, ctx, center):
        elapsed = time.monotonic() - self.stop_started_at
        progress = minimum_jerk_progress(elapsed / RETURN_SEC)
        self._command(ctx, self.stop_from + progress * (center - self.stop_from))
        if elapsed >= RETURN_SEC:
            ctx.request_state(TEST_IDLE, trigger="test_stopped", force=True)

    def _checked_feedback(self, ctx):
        try:
            now = time.monotonic()
            if (
                self.session.last_control_at
                and now - self.session.last_control_at > COMMAND_GAP_TIMEOUT_SEC
            ):
                raise ValueError("test control cycle gap exceeded 50 ms")
            self.session.last_control_at = now
            return self.session.measured(ctx)
        except (KeyError, ValueError) as exc:
            self._fault(ctx, str(exc))
            return None


class SuspendedIdleState(SuspendedState):
    def __init__(self, name, state_id, session):
        super().__init__(name, state_id, session)
        self.from_test = False
        self.start = JOINT_NOMINAL_POS.copy()
        self.entered_at = 0.0

    def is_available(self, ctx):
        if self.session.faulted:
            return False
        node = getattr(ctx, "ros_node", None)
        runtime = getattr(node, "runtime", None)
        if getattr(runtime, "current_state_name", None) == PD_BRAKE:
            return str(getattr(node, "topic_prefix", "")).startswith("simulation/")
        return True

    def on_prepare(self, ctx, from_state):
        self.from_test = from_state.name.startswith("com.bxi.suspended_tests/")

    def on_enter(self, ctx):
        self.entered_at = time.monotonic()
        self.session.last_control_at = 0.0
        self.session.ready = False
        if self.from_test:
            return
        measured = self._checked_feedback(ctx)
        if measured is not None:
            self.start[:] = measured
            self.logger.warning(
                "suspended test preparation started; hold the robot suspended"
            )

    def on_update(self, ctx, dt):
        measured = self._checked_feedback(ctx)
        if measured is None:
            return
        if self.from_test:
            self._command(ctx, JOINT_NOMINAL_POS)
            if np.max(np.abs(measured - JOINT_NOMINAL_POS)) <= IDLE_CENTER_TOLERANCE_RAD:
                self.session.ready = True
            elif time.monotonic() - self.entered_at > 5.0:
                errors = np.abs(measured - JOINT_NOMINAL_POS)
                index = int(np.argmax(errors))
                self._fault(
                    ctx,
                    "robot did not settle after the test stopped: %s error=%.2f deg"
                    % (JOINT_NAMES[index], math.degrees(errors[index])),
                )
            return
        elapsed = time.monotonic() - self.entered_at
        progress = minimum_jerk_progress(elapsed / 10.0)
        position = self.start + progress * (JOINT_NOMINAL_POS - self.start)
        self._command(ctx, position, JOINT_KP * min(elapsed / 10.0, 1.0))
        if elapsed >= 10.0 and not self.session.ready:
            if np.max(np.abs(measured - JOINT_NOMINAL_POS)) <= IDLE_CENTER_TOLERANCE_RAD:
                self.session.ready = True
                self.logger.info("suspended test preparation complete; X/Y/A ready")
            elif elapsed >= 15.0:
                errors = np.abs(measured - JOINT_NOMINAL_POS)
                index = int(np.argmax(errors))
                self._fault(
                    ctx,
                    "robot did not settle at the test center: %s error=%.2f deg"
                    % (JOINT_NAMES[index], math.degrees(errors[index])),
                )


class SuspendedRunningState(SuspendedState):
    def __init__(self, name, state_id, session):
        super().__init__(name, state_id, session)
        path = get_package_share_path("bxi_example_py_elf3") / "data/data.txt"
        self.trajectory = load_joint_trajectory(path, 50.0)
        self.phase = "settle"
        self.phase_started_at = 0.0
        self.next_frame_at = 0.0
        self.blend_from = SUSPENDED_RUN_NOMINAL_POS.copy()
        self.blend_to = SUSPENDED_RUN_NOMINAL_POS.copy()

    def on_enter(self, ctx):
        self.stopping = False
        self.session.last_control_at = 0.0
        self.phase = "center_blend"
        self.phase_started_at = time.monotonic()
        self.trajectory.reset()
        self.blend_from[:] = JOINT_NOMINAL_POS
        self.blend_to[:] = SUSPENDED_RUN_NOMINAL_POS
        self.last_command[:] = JOINT_NOMINAL_POS

    def on_update(self, ctx, dt):
        measured = self._checked_feedback(ctx)
        if measured is None:
            return
        if self.stopping:
            self._update_stop(ctx, JOINT_NOMINAL_POS)
            return
        now = time.monotonic()
        if self.phase == "center_blend":
            progress = (now - self.phase_started_at) / RETURN_SEC
            self._command(
                ctx,
                self.blend_from
                + minimum_jerk_progress(progress) * (self.blend_to - self.blend_from),
            )
            if progress >= 1.0:
                self.phase = "settle"
                self.phase_started_at = now
            return
        if self.phase == "settle":
            self._command(ctx, SUSPENDED_RUN_NOMINAL_POS)
            if now - self.phase_started_at >= 2.0:
                error = np.max(np.abs(measured - SUSPENDED_RUN_NOMINAL_POS))
                if error <= math.radians(5.0):
                    self.phase = "blend"
                    self.phase_started_at = now
                    self.blend_from[:] = SUSPENDED_RUN_NOMINAL_POS
                    self.blend_to[:] = self.trajectory.next()
                elif now - self.phase_started_at > 5.0:
                    self._fault(ctx, "running center did not settle")
            return
        if self.phase == "blend":
            progress = (now - self.phase_started_at) / RETURN_SEC
            self._command(
                ctx,
                self.blend_from
                + minimum_jerk_progress(progress) * (self.blend_to - self.blend_from),
            )
            if progress >= 1.0:
                self.phase = "playback"
                self.next_frame_at = now + 0.02
            return
        if now >= self.next_frame_at:
            if self.trajectory.next_is_first_frame:
                self.phase = "blend"
                self.phase_started_at = now
                self.blend_from[:] = self.last_command
                self.blend_to[:] = self.trajectory.next()
            else:
                self._command(ctx, self.trajectory.next())
            self.next_frame_at = now + 0.02
        else:
            self._command(ctx, self.last_command)


class SuspendedVibrationState(SuspendedState):
    def __init__(self, name, state_id, session):
        super().__init__(name, state_id, session)
        self.entered_at = 0.0
        self.duration_sec = 300.0
        self.start_hz = 10.0
        self.end_hz = 20.0
        self.amplitude_rad = 0.23

    def on_enter(self, ctx):
        self.stopping = False
        self.session.last_control_at = 0.0
        self.entered_at = time.monotonic()
        measured = self._checked_feedback(ctx)
        if measured is None:
            return
        if np.max(np.abs(measured - JOINT_NOMINAL_POS)) > 0.1:
            self._fault(ctx, "vibration start pose is not centered")
            return
        envelope = self.amplitude_rad * np.abs(JOINT_VIBRATION_SIGNS)
        if np.any(JOINT_NOMINAL_POS - envelope < JOINT_POSITION_MIN + JOINT_MARGIN_RAD) or np.any(
            JOINT_NOMINAL_POS + envelope > JOINT_POSITION_MAX - JOINT_MARGIN_RAD
        ):
            self._fault(ctx, "vibration envelope exceeds joint limits")

    def on_update(self, ctx, dt):
        measured = self._checked_feedback(ctx)
        if measured is None:
            return
        if self.stopping:
            self._update_stop(ctx, JOINT_NOMINAL_POS)
            return
        elapsed = time.monotonic() - self.entered_at
        if elapsed >= self.duration_sec:
            self.on_action(ctx, "stop")
            return
        frequency_slope = (self.end_hz - self.start_hz) / self.duration_sec
        phase = 2.0 * math.pi * (
            self.start_hz * elapsed + 0.5 * frequency_slope * elapsed * elapsed
        )
        command = JOINT_NOMINAL_POS + (
            JOINT_VIBRATION_SIGNS * self.amplitude_rad * math.sin(phase)
        )
        self._command(ctx, command)


class SuspendedLimbTestState(SuspendedState):
    def __init__(
        self, name, state_id, session, *, range_speed_deg_s=180.0,
        move_sec=1.5, hold_sec=0.2, collision_margin_deg=10.0,
        mechanical_margin_deg=2.0, tracking_tolerance_deg=2.0,
        start_tolerance_deg=5.0,
    ):
        super().__init__(name, state_id, session)
        if not all(
            math.isfinite(value)
            for value in (
                range_speed_deg_s, move_sec, hold_sec, collision_margin_deg,
                mechanical_margin_deg, tracking_tolerance_deg, start_tolerance_deg,
            )
        ):
            raise ValueError("full-range test parameters must be finite")
        if not 0.0 < range_speed_deg_s <= 180.0:
            raise ValueError("range_speed_deg_s must be in (0, 180]")
        if move_sec <= 0.0 or hold_sec < 0.0:
            raise ValueError("move_sec must be positive and hold_sec non-negative")
        if collision_margin_deg < 0.0 or mechanical_margin_deg < 0.0:
            raise ValueError("joint range margins must be non-negative")
        if tracking_tolerance_deg <= 0.0 or start_tolerance_deg <= 0.0:
            raise ValueError("joint feedback tolerances must be positive")
        self.hold_sec = hold_sec
        self.tracking_tolerance_rad = math.radians(tracking_tolerance_deg)
        self.start_tolerance_rad = math.radians(start_tolerance_deg)
        safe_ranges = build_safe_ranges(
            collision_margin_deg=collision_margin_deg,
            mechanical_margin_deg=mechanical_margin_deg,
        )
        self.segments = []
        previous = JOINT_NOMINAL_POS.copy()
        for group in WHOLE_BODY_TEST_GROUPS:
            motion_names, waypoints = full_range_waypoints(
                JOINT_NOMINAL_POS, group, safe_ranges
            )
            for target in waypoints:
                duration = velocity_limited_duration(
                    previous, target, motion_names,
                    minimum_move_sec=move_sec,
                    range_speed_deg_s=range_speed_deg_s,
                )
                self.segments.append((previous.copy(), target.copy(), motion_names, duration))
                previous = target
        self.segment_index = 0
        self.segment_started_at = 0.0
        self.holding = False
        self.next_start_warning_at = 0.0

    def _start_pose_error(self, ctx):
        measured = self.session.measured(ctx)
        errors = np.abs(measured - JOINT_NOMINAL_POS)
        index = int(np.argmax(errors))
        return JOINT_NAMES[index], float(errors[index])

    def is_available(self, ctx):
        if not super().is_available(ctx):
            return False
        try:
            joint, error = self._start_pose_error(ctx)
        except (KeyError, ValueError):
            return False
        if error <= self.start_tolerance_rad:
            return True
        now = time.monotonic()
        if now >= self.next_start_warning_at:
            self.logger.warning(
                "full-range test waiting for center: %s error=%.2f deg, limit=%.2f deg"
                % (joint, math.degrees(error), math.degrees(self.start_tolerance_rad))
            )
            self.next_start_warning_at = now + 2.0
        return False

    def on_enter(self, ctx):
        self.stopping = False
        self.session.last_control_at = 0.0
        measured = self._checked_feedback(ctx)
        if measured is None:
            return
        errors = np.abs(measured - JOINT_NOMINAL_POS)
        index = int(np.argmax(errors))
        if errors[index] > self.start_tolerance_rad:
            self._fault(
                ctx,
                "full-range test start pose is not centered: %s error=%.2f deg, limit=%.2f deg"
                % (JOINT_NAMES[index], math.degrees(errors[index]), math.degrees(self.start_tolerance_rad)),
            )
            return
        self.segment_index = 0
        self.segment_started_at = time.monotonic()
        self.holding = False
        self.logger.warning(
            "A-key MuJoCo collision checks are disabled; using joint limits and feedback checks"
        )

    def on_update(self, ctx, dt):
        measured = self._checked_feedback(ctx)
        if measured is None:
            return
        if self.stopping:
            self._update_stop(ctx, JOINT_NOMINAL_POS)
            return
        if self.segment_index >= len(self.segments):
            self.on_action(ctx, "stop")
            return
        start, target, names, duration = self.segments[self.segment_index]
        elapsed = time.monotonic() - self.segment_started_at
        if self.holding:
            self._command(ctx, target)
            if elapsed < self.hold_sec:
                return
            indices = [JOINT_NAMES.index(name) for name in names]
            error = float(np.max(np.abs(measured[indices] - target[indices])))
            if error > self.tracking_tolerance_rad:
                self.logger.warning(
                    "joint-test segment %d tracking error %.3f rad"
                    % (self.segment_index + 1, error)
                )
            self.segment_index += 1
            self.segment_started_at = time.monotonic()
            self.holding = False
            return
        progress = elapsed / duration
        self._command(ctx, start + minimum_jerk_progress(progress) * (target - start))
        if progress >= 1.0:
            self.segment_started_at = time.monotonic()
            self.holding = True
