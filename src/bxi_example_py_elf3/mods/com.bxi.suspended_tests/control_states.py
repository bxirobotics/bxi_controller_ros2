"""Suspended tests driven by the shared state-machine actuator publisher."""

from __future__ import annotations

import math
import time
from collections import Counter
from pathlib import Path

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_path

from bxi_example_py_elf3.control.elf3 import (
    JOINT_KD,
    JOINT_KP,
    JOINT_NAMES,
    JOINT_NOMINAL_POS,
    JOINT_POSITION_MAX,
    JOINT_POSITION_MIN,
    JOINT_VIBRATION_SIGNS,
    ROBOT_NAME,
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
SEQUENCE_JOINT = "com.bxi.suspended_tests/sequence_joint"
SEQUENCE_VIBRATION = "com.bxi.suspended_tests/sequence_vibration"
SEQUENCE_RUNNING = "com.bxi.suspended_tests/sequence_running"
SEQUENCE_STATES = frozenset((SEQUENCE_JOINT, SEQUENCE_VIBRATION, SEQUENCE_RUNNING))
ZERO_TORQUE = "com.bxi.basic_actions/zero_torque"
PD_BRAKE = "com.bxi.basic_actions/pd_brake"
JOINT_MARGIN_RAD = 0.02
FEEDBACK_TIMEOUT_SEC = 0.2
COMMAND_GAP_TIMEOUT_SEC = 0.05
RETURN_SEC = 0.5
ZERO_PREPARE_SEC = 3.0
IDLE_CENTER_TOLERANCE_RAD = math.radians(5.0)
TEMPERATURE_TIMEOUT_SEC = 0.5
DEFAULT_TORQUE_LIMIT_C = 90.0
TEST_STOP_MARGIN_C = 15.0
VIBRATION_DURATION_SEC = 300.0
VIBRATION_START_HZ = 10.0
VIBRATION_END_HZ = 20.0
VIBRATION_AMPLITUDE_RAD = 0.23
RUNNING_TRAJECTORY_RATE_HZ = 50.0


class MotorTemperaturePolicy:
    def __init__(self, joint_models, models, defaulted_models=()):
        self.joint_models = joint_models
        self.models = models
        self.defaulted_models = frozenset(defaulted_models)

    def limits_for(self, joint):
        model = self.joint_models.get(joint)
        limits = self.models.get(model)
        if limits is None:
            return model or "unconfigured", DEFAULT_TORQUE_LIMIT_C, None, True
        torque, shutdown = limits
        return model, torque, shutdown, model in self.defaulted_models


def _load_optional_mapping(path: Path, allowed_keys):
    try:
        with path.open(encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
    except FileNotFoundError:
        data = {}
    if data is None:
        data = {}
    if not isinstance(data, dict) or set(data) - allowed_keys:
        raise ValueError("%s has invalid top-level fields" % path.name)
    return data


def _validate_optional_range(label, value, *, nonnegative=False):
    if value is None:
        return
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError("%s must be [min, max] or null" % label)
    if any(isinstance(bound, bool) or not isinstance(bound, (int, float))
           or not math.isfinite(bound) for bound in value):
        raise ValueError("%s bounds must be finite numbers" % label)
    if value[0] >= value[1] or (nonnegative and value[0] < 0.0):
        raise ValueError("%s bounds must be ordered%s" % (
            label, " and nonnegative" if nonnegative else "",
        ))


def _validate_optional_torque(label, value):
    if value is not None and (isinstance(value, bool)
            or not isinstance(value, (int, float)) or not math.isfinite(value)
            or value <= 0.0):
        raise ValueError("%s must be a positive finite number or null" % label)


def load_temperature_policy(config_dir: Path):
    joint_config = _load_optional_mapping(
        config_dir / "robot_joints.yaml", {"robot", "motor_model_counts", "joints"},
    )
    if joint_config.get("robot") not in (None, ROBOT_NAME):
        raise ValueError("robot_joints.yaml robot must be %s" % ROBOT_NAME)
    joints = joint_config.get("joints")
    if joints is None:
        joints = {}
    if not isinstance(joints, dict):
        raise ValueError("robot_joints.yaml joints must be a mapping")
    joint_models = {}
    for joint, info in joints.items():
        if not isinstance(joint, str) or not joint.strip():
            raise ValueError("joint names must be nonempty strings")
        if not isinstance(info, dict):
            raise ValueError("joint %s information must be a mapping" % joint)
        _validate_optional_range(
            "joint %s position_limit_rad" % joint, info.get("position_limit_rad"),
        )
        _validate_optional_torque(
            "joint %s max_command_torque_nm" % joint,
            info.get("max_command_torque_nm"),
        )
        model = info.get("motor_model")
        if model is not None and (not isinstance(model, str) or not model.strip()):
            raise ValueError("joint %s motor_model must be a nonempty string" % joint)
        joint_models[joint] = model

    expected_counts = joint_config.get("motor_model_counts")
    if expected_counts is None:
        expected_counts = {}
    if not isinstance(expected_counts, dict):
        raise ValueError("robot_joints.yaml motor_model_counts must be a mapping")
    for model, count in expected_counts.items():
        if not isinstance(model, str) or not model.strip():
            raise ValueError("motor_model_counts model names must be nonempty strings")
        if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
            raise ValueError("motor_model_counts[%s] must be a positive integer" % model)
    actual_counts = Counter(model for model in joint_models.values() if model is not None)
    count_errors = [
        "%s expected=%s actual=%d" % (
            model,
            expected_counts.get(model, "missing"),
            actual_counts.get(model, 0),
        )
        for model in sorted(set(expected_counts) | set(actual_counts))
        if expected_counts.get(model) != actual_counts.get(model, 0)
    ]
    if count_errors:
        raise ValueError("motor model joint count mismatch: %s" % "; ".join(count_errors))

    motor_config = _load_optional_mapping(
        config_dir / "motor_models.yaml", {"models"},
    )
    models = motor_config.get("models")
    if models is None:
        models = {}
    if not isinstance(models, dict):
        raise ValueError("motor_models.yaml models must be a mapping")
    validated_models = {}
    defaulted_models = set()
    for name, info in models.items():
        if not isinstance(name, str) or not name.strip():
            raise ValueError("motor model names must be nonempty strings")
        if not isinstance(info, dict):
            raise ValueError("motor model %s information must be a mapping" % name)
        _validate_optional_torque(
            "motor model %s max_torque_nm" % name, info.get("max_torque_nm"),
        )
        mit_ranges = info.get("mit_ranges")
        if mit_ranges is not None:
            if not isinstance(mit_ranges, dict) or set(mit_ranges) - {
                "position_rad", "velocity_rad_s", "kp", "kd", "torque_nm",
            }:
                raise ValueError("motor model %s mit_ranges has invalid fields" % name)
            for field, value in mit_ranges.items():
                _validate_optional_range(
                    "motor model %s mit_ranges.%s" % (name, field), value,
                    nonnegative=field in ("kp", "kd"),
                )
        thresholds = info.get("temperature")
        if thresholds is None:
            thresholds = {}
        if not isinstance(thresholds, dict) or set(thresholds) - {"torque_limit_c", "shutdown_c"}:
            raise ValueError("model %s temperature must contain only torque_limit_c and shutdown_c" % name)
        torque = thresholds.get("torque_limit_c")
        shutdown = thresholds.get("shutdown_c")
        for label, value in (("torque_limit_c", torque), ("shutdown_c", shutdown)):
            if value is not None and (isinstance(value, bool)
                    or not isinstance(value, (int, float)) or not math.isfinite(value)
                    or not 0.0 < value <= 150.0):
                raise ValueError("model %s %s must be in (0, 150]" % (name, label))
        effective_torque = float(torque) if torque is not None else DEFAULT_TORQUE_LIMIT_C
        if effective_torque <= TEST_STOP_MARGIN_C:
            raise ValueError(
                "model %s torque_limit_c must exceed %.1f C test stop margin"
                % (name, TEST_STOP_MARGIN_C)
            )
        if torque is None:
            defaulted_models.add(name)
        if shutdown is not None and shutdown <= effective_torque:
            raise ValueError("model %s shutdown_c must exceed torque_limit_c" % name)
        validated_models[name] = (effective_torque, float(shutdown) if shutdown is not None else None)
    return MotorTemperaturePolicy(dict(joint_models), validated_models, defaulted_models)


class TestSession:
    def __init__(self):
        self.ready = False
        self.faulted = False
        self.center_kp_scale = 1.1
        self.center_kd_scale = 1.05
        self.last_feedback_stamp = None
        self.last_feedback_at = 0.0
        self.last_control_at = 0.0
        self.temperature_policy = None
        self.temperature_config_error = "motor temperature configuration is missing"
        self.sequence_id = 0
        self.sequence_active = False
        self.sequence_stage = None
        self.sequence_started_at = 0.0
        self.sequence_logger = None
        self.sequence_fault_reason = None
        self.sequence_exit_requested = False
        self.sequence_cancel_requested = False

    def start_sequence(self, logger, *, joint_command):
        self.sequence_id += 1
        self.sequence_active = True
        self.sequence_stage = SEQUENCE_JOINT
        self.sequence_started_at = time.monotonic()
        self.sequence_logger = logger
        self.sequence_fault_reason = None
        self.sequence_exit_requested = False
        self.sequence_cancel_requested = False
        if self.temperature_policy is None:
            temperature_command = self.temperature_config_error
        else:
            models = sorted(set(self.temperature_policy.joint_models.values()) - {None})
            temperature_command = ", ".join(
                "%s test_stop=%.1f C, torque=%.1f C, shutdown=%s%s" % (
                    model,
                    self.temperature_policy.models.get(model, (DEFAULT_TORQUE_LIMIT_C, None))[0]
                    - TEST_STOP_MARGIN_C,
                    self.temperature_policy.models.get(model, (DEFAULT_TORQUE_LIMIT_C, None))[0],
                    ("%.1f C" % self.temperature_policy.models[model][1]
                     if model in self.temperature_policy.models
                     and self.temperature_policy.models[model][1] is not None
                     else "unconfigured"),
                    " (default)" if model in self.temperature_policy.defaulted_models
                    or model not in self.temperature_policy.models else "",
                )
                for model in models
            ) or "all joints: test_stop=%.1f C, torque=%.1f C (default)" % (
                DEFAULT_TORQUE_LIMIT_C - TEST_STOP_MARGIN_C, DEFAULT_TORQUE_LIMIT_C,
            )
            unmapped = sum(
                self.temperature_policy.joint_models.get(joint) is None
                for joint in JOINT_NAMES
            )
            if unmapped:
                temperature_command += "; %d unmapped joints: test_stop=%.1f C, torque=%.1f C (default)" % (
                    unmapped, DEFAULT_TORQUE_LIMIT_C - TEST_STOP_MARGIN_C,
                    DEFAULT_TORQUE_LIMIT_C,
                )
        logger.warning("========== FULL B TEST #%d START ==========" % self.sequence_id)
        logger.warning(
            "B TEST #%d COMMAND: input=B; joint range -> vibration %.0f s "
            "(%.0f..%.0f Hz, amplitude %.2f rad) -> running trajectory "
            "(%.0f Hz, until exit=B or LB+RB+B or safety stop); %s"
            % (
                self.sequence_id, VIBRATION_DURATION_SEC, VIBRATION_START_HZ,
                VIBRATION_END_HZ, VIBRATION_AMPLITUDE_RAD,
                RUNNING_TRAJECTORY_RATE_HZ, joint_command,
            )
        )
        logger.warning("B TEST #%d MOTOR LIMITS: %s" % (self.sequence_id, temperature_command))
        logger.warning("B TEST #%d STAGE 1/3: joint range started" % self.sequence_id)

    def enter_sequence_stage(self, stage, label):
        if self.sequence_active:
            self.sequence_stage = stage
            self.sequence_logger.warning(
                "B TEST #%d %s started" % (self.sequence_id, label)
            )

    def observe_sequence_state(self, state_name):
        if not self.sequence_active:
            return
        if state_name in SEQUENCE_STATES:
            self.sequence_exit_requested = False
            return
        elapsed = time.monotonic() - self.sequence_started_at
        if self.sequence_fault_reason is not None:
            self.sequence_logger.error(
                "========== B TEST #%d FAILED after %.1f s at %s: %s =========="
                % (self.sequence_id, elapsed, self.sequence_stage, self.sequence_fault_reason)
            )
        elif (self.sequence_exit_requested or self.sequence_cancel_requested) and self.sequence_stage == SEQUENCE_RUNNING:
            self.sequence_logger.warning(
                "========== B TEST #%d SUCCESS after %.1f s: operator ended running stage; "
                "final_state=%s ==========" % (self.sequence_id, elapsed, state_name)
            )
        else:
            reason = (
                "operator exited before running stage"
                if self.sequence_exit_requested or self.sequence_cancel_requested
                else "unexpected state transition"
            )
            self.sequence_logger.error(
                "========== B TEST #%d INCOMPLETE after %.1f s at %s: %s; final_state=%s =========="
                % (self.sequence_id, elapsed, self.sequence_stage, reason, state_name)
            )
        self.sequence_active = False
        self.sequence_logger = None
        self.sequence_cancel_requested = False

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
        self.command_limit_slack_rad = 0.0
        self._warned_limit_joints = set()
        self.return_state = TEST_IDLE

    def is_available(self, ctx):
        return self.session.ready and not self.session.faulted

    def _fault(self, ctx, reason):
        self.session.ready = False
        self.session.faulted = True
        if self.session.sequence_active and self.session.sequence_fault_reason is None:
            self.session.sequence_fault_reason = reason
        self.logger.error("suspended test fault: %s; switching to zero torque" % reason)
        ctx.request_state(ZERO_TORQUE, trigger="test_safety_fault", force=True)

    def _command(self, ctx, position, kp=JOINT_KP, kd=JOINT_KD):
        position = np.asarray(position, dtype=np.float64)
        if position.shape != JOINT_NOMINAL_POS.shape or not np.all(np.isfinite(position)):
            self._fault(ctx, "command has invalid joint count or non-finite position")
            return
        original_min = JOINT_POSITION_MIN + JOINT_MARGIN_RAD
        original_max = JOINT_POSITION_MAX - JOINT_MARGIN_RAD
        below = np.maximum(original_min - position, 0.0)
        above = np.maximum(position - original_max, 0.0)
        over = below + above
        exceeded = np.flatnonzero(over > 0.0)
        if np.any(over > self.command_limit_slack_rad):
            details = "; ".join(
                "%s target=%.2f deg, original=[%.2f, %.2f] deg, over=%.2f deg (%s, %s)"
                % (
                    JOINT_NAMES[index],
                    math.degrees(position[index]),
                    math.degrees(original_min[index]),
                    math.degrees(original_max[index]),
                    math.degrees(over[index]),
                    "below min" if below[index] else "above max",
                    "fault" if over[index] > self.command_limit_slack_rad else "within allowance",
                )
                for index in exceeded
            )
            self._fault(
                ctx,
                "command exceeds software joint limits by more than %.2f deg: %s"
                % (math.degrees(self.command_limit_slack_rad), details),
            )
            return
        newly_exceeded = [
            int(index) for index in exceeded
            if index not in self._warned_limit_joints
        ]
        if newly_exceeded:
            self._warned_limit_joints.update(newly_exceeded)
            self.logger.warning(
                "command outside original software limits (within %.2f deg allowance): %s"
                % (
                    math.degrees(self.command_limit_slack_rad),
                    "; ".join(
                        "%s target=%.2f deg, original=[%.2f, %.2f] deg, over=%.2f deg"
                        % (
                            JOINT_NAMES[index],
                            math.degrees(position[index]),
                            math.degrees(original_min[index]),
                            math.degrees(original_max[index]),
                            math.degrees(over[index]),
                        )
                        for index in newly_exceeded
                    ),
                )
            )
        self.last_command[:] = position
        self._apply_frame(
            ctx, self._motor_frame(ctx, position, kp, kd, layout=TEST_LAYOUT)
        )

    def on_action(self, ctx, action_name):
        if action_name == "cancel_sequence" and self.name in SEQUENCE_STATES:
            self.session.sequence_cancel_requested = True
            self.return_state = TEST_IDLE
            if self.stopping:
                self.stop_started_at = time.monotonic()
                self.stop_from[:] = self.last_command
                return True
            return self.on_action(ctx, "stop")
        if action_name == "stop" and self.stopping:
            return True
        if action_name != "stop":
            return False
        self.stopping = True
        self.stop_started_at = time.monotonic()
        self.stop_from[:] = self.last_command
        self.logger.info("stopping suspended test; returning to center")
        return True

    def _update_stop(self, ctx, center):
        elapsed = time.monotonic() - self.stop_started_at
        if elapsed >= RETURN_SEC:
            measured = self.session.measured(ctx)
            error = np.max(np.abs(measured - center))
            if error <= IDLE_CENTER_TOLERANCE_RAD:
                accepted = ctx.request_state(
                    self.return_state,
                    trigger="test_stopped" if self.return_state == TEST_IDLE else "test_stage_complete",
                    force=self.return_state == TEST_IDLE,
                )
                if accepted is False:
                    self._fault(ctx, "next test stage %s is unavailable" % self.return_state)
                return
            if elapsed >= RETURN_SEC + 5.0:
                self._fault(
                    ctx, "test did not settle at center: %.2f deg"
                    % math.degrees(error),
                )
                return
        progress = minimum_jerk_progress(elapsed / RETURN_SEC)
        self._command(
            ctx,
            self.stop_from + progress * (center - self.stop_from),
            JOINT_KP * self.session.center_kp_scale,
            JOINT_KD * self.session.center_kd_scale,
        )

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
    def __init__(
        self, name, state_id, session, *, prepare_sec=3.0,
        prepare_kp_scale=1.1, center_kd_scale=1.05,
        command_limit_slack_deg=10.0,
    ):
        super().__init__(name, state_id, session)
        if not math.isfinite(prepare_sec) or not 3.0 <= prepare_sec <= 20.0:
            raise ValueError("test idle prepare_sec must be in [3, 20]")
        if not math.isfinite(prepare_kp_scale) or not 0.5 <= prepare_kp_scale <= 1.2:
            raise ValueError("test idle prepare_kp_scale must be in [0.5, 1.2]")
        if not math.isfinite(center_kd_scale) or not 0.5 <= center_kd_scale <= 1.2:
            raise ValueError("test idle center_kd_scale must be in [0.5, 1.2]")
        if not math.isfinite(command_limit_slack_deg) or not 0.0 <= command_limit_slack_deg <= 10.0:
            raise ValueError("test idle command_limit_slack_deg must be in [0, 10]")
        self.prepare_sec = float(prepare_sec)
        self.prepare_kp_scale = float(prepare_kp_scale)
        self.center_kd_scale = float(center_kd_scale)
        self.command_limit_slack_rad = math.radians(command_limit_slack_deg)
        session.center_kp_scale = self.prepare_kp_scale
        session.center_kd_scale = self.center_kd_scale
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
                "suspended test preparation started: %.1fs, kp_scale=%.2f, kd_scale=%.2f; "
                "hold the robot suspended"
                % (self.prepare_sec, self.prepare_kp_scale, self.center_kd_scale)
            )

    def on_update(self, ctx, dt):
        measured = self._checked_feedback(ctx)
        if measured is None:
            return
        if self.from_test:
            self._command(
                ctx,
                JOINT_NOMINAL_POS,
                JOINT_KP * self.session.center_kp_scale,
                JOINT_KD * self.session.center_kd_scale,
            )
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
        progress = minimum_jerk_progress(elapsed / self.prepare_sec)
        position = self.start + progress * (JOINT_NOMINAL_POS - self.start)
        self._command(
            ctx,
            position,
            JOINT_KP * self.prepare_kp_scale * min(elapsed / self.prepare_sec, 1.0),
            JOINT_KD * self.center_kd_scale,
        )
        if elapsed >= self.prepare_sec and not self.session.ready:
            if np.max(np.abs(measured - JOINT_NOMINAL_POS)) <= IDLE_CENTER_TOLERANCE_RAD:
                self.session.ready = True
                self.logger.info("suspended test preparation complete; X/Y/A ready")
            elif elapsed >= self.prepare_sec + 5.0:
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
        self.trajectory = load_joint_trajectory(path, RUNNING_TRAJECTORY_RATE_HZ)
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
        self.duration_sec = VIBRATION_DURATION_SEC
        self.start_hz = VIBRATION_START_HZ
        self.end_hz = VIBRATION_END_HZ
        self.amplitude_rad = VIBRATION_AMPLITUDE_RAD

    def on_enter(self, ctx):
        self.stopping = False
        self.session.last_control_at = 0.0
        self.entered_at = time.monotonic()
        self.last_command[:] = JOINT_NOMINAL_POS
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
        self.range_speed_deg_s = range_speed_deg_s
        self.move_sec = move_sec
        self.collision_margin_deg = collision_margin_deg
        self.mechanical_margin_deg = mechanical_margin_deg
        self.hold_sec = hold_sec
        self.tracking_tolerance_rad = math.radians(tracking_tolerance_deg)
        self.start_tolerance_rad = math.radians(start_tolerance_deg)
        safe_ranges = build_safe_ranges(
            collision_margin_deg=collision_margin_deg,
            mechanical_margin_deg=mechanical_margin_deg,
        )
        self.segments = []
        self.zero_position = np.zeros_like(JOINT_NOMINAL_POS)
        previous = self.zero_position.copy()
        for group in WHOLE_BODY_TEST_GROUPS:
            motion_names, waypoints = full_range_waypoints(
                self.zero_position, group, safe_ranges
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
        self.zeroing = False
        self.zero_started_at = 0.0
        self.zero_from = JOINT_NOMINAL_POS.copy()

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
        self.zero_from[:] = measured
        self.zero_started_at = time.monotonic()
        self.zeroing = True
        self.segment_started_at = 0.0
        self.holding = False
        self.last_command[:] = measured
        self.logger.warning(
            "joint range preparation: moving all joints to 0 rad in %.1f s"
            % ZERO_PREPARE_SEC
        )
        self.logger.warning(
            "joint-range MuJoCo collision checks are disabled; using joint limits and feedback checks"
        )

    def on_update(self, ctx, dt):
        measured = self._checked_feedback(ctx)
        if measured is None:
            return
        if self.stopping:
            self._update_stop(ctx, JOINT_NOMINAL_POS)
            return
        if self.zeroing:
            elapsed = time.monotonic() - self.zero_started_at
            position = self.zero_from + minimum_jerk_progress(elapsed / ZERO_PREPARE_SEC) * (
                self.zero_position - self.zero_from
            )
            self._command(ctx, position)
            if elapsed >= ZERO_PREPARE_SEC:
                errors = np.abs(measured - self.zero_position)
                index = int(np.argmax(errors))
                if errors[index] <= self.start_tolerance_rad:
                    self.zeroing = False
                    self.segment_started_at = time.monotonic()
                    self.logger.warning("joint range preparation complete: all joints at 0 rad")
                elif elapsed >= ZERO_PREPARE_SEC + 5.0:
                    self._fault(
                        ctx, "joint range zero pose did not settle: %s error=%.2f deg, limit=%.2f deg"
                        % (JOINT_NAMES[index], math.degrees(errors[index]),
                           math.degrees(self.start_tolerance_rad)),
                    )
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


class SequentialTemperatureGuard:
    def _check_temperature(self, ctx):
        if self.session.temperature_policy is None:
            reason = self.session.temperature_config_error
        else:
            reason = temperature_fault(
                ctx, policy=self.session.temperature_policy,
                timeout_sec=TEMPERATURE_TIMEOUT_SEC,
            )
        if reason is not None:
            self._fault(ctx, reason)
            return False
        return True

    def on_enter(self, ctx):
        policy = self.session.temperature_policy
        sample = getattr(ctx, "actuator_temperatures", None)
        if policy is not None and sample is not None:
            defaulted = [name for name in sample.names if policy.limits_for(name)[3]]
            if defaulted:
                self.logger.warning(
                    "using default %.1f C motor torque limit and %.1f C test stop for %d "
                    "unconfigured joints: %s"
                    % (DEFAULT_TORQUE_LIMIT_C, DEFAULT_TORQUE_LIMIT_C - TEST_STOP_MARGIN_C,
                       len(defaulted), ", ".join(defaulted))
                )
        if self._check_temperature(ctx):
            super().on_enter(ctx)

    def on_update(self, ctx, dt):
        if not self.session.faulted and self._check_temperature(ctx):
            super().on_update(ctx, dt)


class SequentialLimbTestState(SequentialTemperatureGuard, SuspendedLimbTestState):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.return_state = SEQUENCE_VIBRATION

    def on_enter(self, ctx):
        self.return_state = SEQUENCE_VIBRATION
        self.session.start_sequence(
            self.logger,
            joint_command=(
                "joints=%d, segments=%d, range_speed=%.1f deg/s, min_move=%.2f s, "
                "hold=%.2f s, collision_margin=%.1f deg, mechanical_margin=%.1f deg, "
                "tracking_tolerance=%.1f deg, start_tolerance=%.1f deg"
            ) % (
                len(JOINT_NAMES), len(self.segments), self.range_speed_deg_s,
                self.move_sec, self.hold_sec, self.collision_margin_deg,
                self.mechanical_margin_deg, math.degrees(self.tracking_tolerance_rad),
                math.degrees(self.start_tolerance_rad),
            ),
        )
        super().on_enter(ctx)


class SequentialVibrationState(SequentialTemperatureGuard, SuspendedVibrationState):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.return_state = SEQUENCE_RUNNING

    def on_enter(self, ctx):
        self.return_state = SEQUENCE_RUNNING
        self.session.enter_sequence_stage(SEQUENCE_VIBRATION, "STAGE 2/3: vibration")
        super().on_enter(ctx)


def temperature_fault(ctx, *, policy, timeout_sec):
    sample = getattr(ctx, "actuator_temperatures", None)
    if sample is None:
        return "actuator temperature feedback is missing"
    age = time.monotonic() - sample.received_at
    if not math.isfinite(age) or age < 0.0 or age > timeout_sec:
        return "actuator temperature feedback is stale (age=%.3fs)" % age
    names = sample.names
    if len(names) != len(set(names)) or len(sample.motor_c) != len(names):
        return "actuator motor temperature names/values are incomplete"
    available_names = set(names)
    for name in JOINT_NAMES:
        if name not in available_names:
            return "actuator temperature is missing joint %s" % name
    for index, name in enumerate(names):
        model, torque_limit_c, shutdown_c, _ = policy.limits_for(name)
        motor = sample.motor_c[index]
        if not math.isfinite(motor):
            return "invalid motor temperature for %s" % name
        test_stop_c = torque_limit_c - TEST_STOP_MARGIN_C
        if motor >= test_stop_c:
            shutdown_label = "%.1f C" % shutdown_c if shutdown_c is not None else "unconfigured"
            return ("%s (%s) motor temperature %.1f C reached test stop %.1f C "
                    "(%.1f C below torque limit %.1f C; shutdown threshold %s)") % (
                name, model, motor, test_stop_c, TEST_STOP_MARGIN_C,
                torque_limit_c, shutdown_label,
            )
    return None


class SequentialRunningState(SequentialTemperatureGuard, SuspendedRunningState):
    def on_enter(self, ctx):
        self.return_state = TEST_IDLE
        self.session.enter_sequence_stage(SEQUENCE_RUNNING, "STAGE 3/3: running")
        super().on_enter(ctx)
