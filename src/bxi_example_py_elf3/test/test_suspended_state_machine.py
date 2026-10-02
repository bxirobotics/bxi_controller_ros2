"""Offline checks for suspended-test state and timer ownership."""

from __future__ import annotations

from collections import deque
from pathlib import Path
from threading import Event, RLock
from types import SimpleNamespace
import importlib.util
import math
import os
import time

import numpy as np
import pytest
import yaml

from bxi_example_py_elf3.control.elf3 import (
    JOINT_KD,
    JOINT_KP,
    JOINT_NAMES,
    JOINT_NOMINAL_POS,
    JOINT_POSITION_MAX,
    JOINT_POSITION_MIN,
)
from bxi_example_py_elf3.framework.joints import JointLayout
from bxi_example_py_elf3.framework.platform.api import ActuatorTemperatures
from bxi_example_py_elf3.framework.platform.runtime import RobotControlRuntime
from bxi_example_py_elf3.framework.runtime.control_scheduler import (
    ControlCycleResult,
    ControlScheduler,
)
from bxi_example_py_elf3.framework.runtime.state_machine import RemoteEventAdapter


_STATES_PATH = (
    Path(__file__).resolve().parents[1]
    / "mods/com.bxi.suspended_tests/control_states.py"
)
_SPEC = importlib.util.spec_from_file_location("suspended_control_states_test", _STATES_PATH)
assert _SPEC is not None and _SPEC.loader is not None
_STATES = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_STATES)

_GUARD_PATH = _STATES_PATH.with_name("remote_guard.py")
_GUARD_SPEC = importlib.util.spec_from_file_location("suspended_remote_guard_test", _GUARD_PATH)
assert _GUARD_SPEC is not None and _GUARD_SPEC.loader is not None
_GUARD = importlib.util.module_from_spec(_GUARD_SPEC)
_GUARD_SPEC.loader.exec_module(_GUARD)


class _Logger:
    def __init__(self):
        self.warnings = []
        self.errors = []

    def info(self, message):
        pass

    def warning(self, message):
        self.warnings.append(message)

    def error(self, message):
        self.errors.append(message)


class _Context:
    def __init__(self):
        self.robot_layout = JointLayout(JOINT_NAMES)
        self.robot_joints = SimpleNamespace(
            timestamp_ns=1,
            position=JOINT_NOMINAL_POS.copy(),
        )
        self.frame = None
        self.requests = []
        self.actuator_temperatures = None

    def set_motor_target(self, frame):
        self.frame = frame

    def request_state(self, name, *, trigger, force=False):
        self.requests.append((name, trigger, force))


def _bind(state):
    state._bind_logger(_Logger())
    return state


def test_idle_preparation_and_joint_feedback_fault():
    session = _STATES.TestSession()
    idle = _bind(_STATES.SuspendedIdleState("com.bxi.suspended_tests/idle", 1, session))
    ctx = _Context()
    idle.on_prepare(ctx, SimpleNamespace(name="com.bxi.basic_actions/zero_torque"))
    idle.on_enter(ctx)
    idle.entered_at -= 10.1
    idle.on_update(ctx, 0.005)
    assert session.ready
    assert ctx.frame is not None
    assert np.allclose(ctx.frame.qpos, JOINT_NOMINAL_POS)
    np.testing.assert_allclose(ctx.frame.kp, JOINT_KP * 1.1, rtol=1e-5)
    np.testing.assert_allclose(ctx.frame.kd, JOINT_KD * 1.05, rtol=1e-5)

    session.last_feedback_at -= 0.3
    idle.on_update(ctx, 0.005)
    assert session.faulted
    assert ctx.requests[-1][0] == _STATES.ZERO_TORQUE


def test_idle_preparation_time_and_kp_scale():
    session = _STATES.TestSession()
    idle = _bind(
        _STATES.SuspendedIdleState(
            "com.bxi.suspended_tests/idle", 1, session,
            prepare_sec=8.0, prepare_kp_scale=1.1,
        )
    )
    ctx = _Context()
    ctx.robot_joints.position[0] += 0.1
    idle.on_prepare(ctx, SimpleNamespace(name="com.bxi.basic_actions/zero_torque"))
    idle.on_enter(ctx)
    idle.entered_at -= 4.0
    idle.on_update(ctx, 0.005)
    np.testing.assert_allclose(ctx.frame.qpos[0], JOINT_NOMINAL_POS[0] + 0.05, atol=1e-4)
    np.testing.assert_allclose(ctx.frame.kp, JOINT_KP * 1.1 * 0.5, rtol=1e-3)
    np.testing.assert_allclose(ctx.frame.kd, JOINT_KD * 1.05, rtol=1e-5)
    assert not session.ready

    ctx.robot_joints.position[0] = JOINT_NOMINAL_POS[0]
    idle.entered_at -= 4.1
    idle.on_update(ctx, 0.005)
    assert session.ready
    np.testing.assert_allclose(ctx.frame.kp, JOINT_KP * 1.1, rtol=1e-5)

    idle.on_prepare(ctx, SimpleNamespace(name="com.bxi.suspended_tests/running"))
    idle.on_enter(ctx)
    idle.on_update(ctx, 0.005)
    np.testing.assert_allclose(ctx.frame.kp, JOINT_KP * 1.1, rtol=1e-5)
    np.testing.assert_allclose(ctx.frame.kd, JOINT_KD * 1.05, rtol=1e-5)


def test_idle_preparation_rejects_unsafe_parameters():
    for params in (
        {"prepare_sec": 2.0},
        {"prepare_sec": float("nan")},
        {"prepare_kp_scale": 1.3},
        {"prepare_kp_scale": float("inf")},
        {"center_kd_scale": 1.3},
        {"center_kd_scale": float("nan")},
        {"command_limit_slack_deg": -1.0},
        {"command_limit_slack_deg": 10.1},
        {"command_limit_slack_deg": float("inf")},
    ):
        with pytest.raises(ValueError):
            _STATES.SuspendedIdleState("idle", 1, _STATES.TestSession(), **params)


def test_limit_allowance_warns_once_and_keeps_original_range():
    session = _STATES.TestSession()
    state = _bind(_STATES.SuspendedIdleState("idle", 1, session))
    ctx = _Context()
    target = JOINT_NOMINAL_POS.copy()
    target[0] = JOINT_POSITION_MAX[0] - _STATES.JOINT_MARGIN_RAD + np.deg2rad(5.0)

    state._command(ctx, target)
    state._command(ctx, target)

    assert not session.faulted
    np.testing.assert_allclose(ctx.frame.qpos[0], target[0])
    assert len(state.logger.warnings) == 1
    assert "waist_y_joint" in state.logger.warnings[0]
    assert "original=[" in state.logger.warnings[0]
    assert "over=5.00 deg" in state.logger.warnings[0]


def test_reentry_from_zero_torque_accepts_start_pose_within_allowance():
    session = _STATES.TestSession()
    idle = _bind(_STATES.SuspendedIdleState("idle", 1, session))
    ctx = _Context()
    ctx.robot_joints.position[0] = (
        JOINT_POSITION_MAX[0] - _STATES.JOINT_MARGIN_RAD + np.deg2rad(5.0)
    )

    idle.on_prepare(ctx, SimpleNamespace(name="com.bxi.basic_actions/zero_torque"))
    idle.on_enter(ctx)
    idle.on_update(ctx, 0.005)

    assert not session.faulted
    assert ctx.requests == []
    assert ctx.frame is not None
    assert "waist_y_joint" in idle.logger.warnings[-1]


def test_limit_fault_names_every_joint_and_reports_degrees_over_original():
    session = _STATES.TestSession()
    state = _bind(_STATES.SuspendedIdleState("idle", 1, session))
    ctx = _Context()
    target = JOINT_NOMINAL_POS.copy()
    target[0] = JOINT_POSITION_MAX[0] - _STATES.JOINT_MARGIN_RAD + np.deg2rad(11.0)
    target[1] = JOINT_POSITION_MIN[1] + _STATES.JOINT_MARGIN_RAD - np.deg2rad(12.0)
    target[2] = JOINT_POSITION_MAX[2] - _STATES.JOINT_MARGIN_RAD + np.deg2rad(5.0)

    state._command(ctx, target)

    assert session.faulted
    assert ctx.frame is None
    assert ctx.requests == [(_STATES.ZERO_TORQUE, "test_safety_fault", True)]
    message = state.logger.errors[0]
    assert "waist_y_joint" in message and "over=11.00 deg (above max, fault)" in message
    assert "waist_x_joint" in message and "over=12.00 deg (below min, fault)" in message
    assert "waist_z_joint" in message and "over=5.00 deg (above max, within allowance)" in message
    assert message.count("original=[") == 3


def test_active_test_commands_keep_original_limit():
    class ActiveState(_STATES.SuspendedState):
        def on_update(self, ctx, dt):
            pass

    session = _STATES.TestSession()
    state = _bind(ActiveState("running", 2, session))
    ctx = _Context()
    target = JOINT_NOMINAL_POS.copy()
    target[0] = JOINT_POSITION_MAX[0] - _STATES.JOINT_MARGIN_RAD + np.deg2rad(1.0)

    state._command(ctx, target)

    assert session.faulted
    assert ctx.frame is None
    assert "waist_y_joint" in state.logger.errors[0]
    assert "over=1.00 deg" in state.logger.errors[0]


def test_test_mode_has_one_button_exit_and_ignores_basic_mode_buttons():
    config = yaml.safe_load(
        (_STATES_PATH.parent / "mod.yaml").read_text(encoding="utf-8")
    )
    assert config["events"]["test_mode"] == {"slot": "btn_8", "value": 1}
    assert config["events"]["exit_test_mode"] == {"slot": "btn_8", "value": 2}
    assert config["events"]["sequence"] == {"slot": "btn_9", "value": 2}
    for source in (
        "idle", "running", "vibration", "whole_body_joint_test",
        "sequence_joint", "sequence_vibration", "sequence_running",
    ):
        routes = [route for route in config["routes"] if route["from"] == source]
        exits = [route for route in routes if route["to"] == _STATES.ZERO_TORQUE]
        assert exits == [{"from": source, "event": "exit_test_mode", "to": _STATES.ZERO_TORQUE}]
        assert all(route["event"] != "test_mode" for route in routes)

    adapter = RemoteEventAdapter({
        "test_mode": config["events"]["test_mode"],
        "exit_test_mode": config["events"]["exit_test_mode"],
    })
    assert adapter.extract_events(SimpleNamespace(btn_8=1)) == ["test_mode"]
    assert adapter.extract_events(SimpleNamespace(btn_8=2)) == ["exit_test_mode"]


def _temperature_sample(*, names=JOINT_NAMES, motor=None, driver=None, age=0.0):
    return ActuatorTemperatures(
        names=tuple(names),
        motor_c=tuple(motor if motor is not None else [35.0] * len(names)),
        driver_c=tuple(driver if driver is not None else [35.0] * len(names)),
        received_at=time.monotonic() - age,
    )


def test_sequence_advances_only_after_return_to_center():
    session = _STATES.TestSession()
    session.ready = True
    ctx = _Context()
    ctx.actuator_temperatures = _temperature_sample()
    states = (
        _bind(_STATES.SequentialLimbTestState("sequence_joint", 10, session)),
        _bind(_STATES.SequentialVibrationState("sequence_vibration", 11, session)),
    )
    for state, expected_next in zip(
        states, (_STATES.SEQUENCE_VIBRATION, _STATES.SEQUENCE_RUNNING)
    ):
        state.on_enter(ctx)
        if isinstance(state, _STATES.SequentialLimbTestState):
            state.segment_index = len(state.segments)
        else:
            state.entered_at -= 300.1
        state.on_update(ctx, 0.005)
        assert state.stopping
        state.stop_started_at -= 0.51
        ctx.robot_joints.position[0] += math.radians(6.0)
        state.on_update(ctx, 0.005)
        assert ctx.requests == []
        ctx.robot_joints.position[:] = JOINT_NOMINAL_POS
        state.on_update(ctx, 0.005)
        assert ctx.requests[-1][0] == expected_next
        ctx.requests.clear()


def test_sequence_stage_handoff_rejection_faults_without_old_command():
    class RejectingContext(_Context):
        def request_state(self, name, *, trigger, force=False):
            if name == _STATES.SEQUENCE_RUNNING:
                return False
            return super().request_state(name, trigger=trigger, force=force)

    session = _STATES.TestSession()
    session.ready = True
    ctx = RejectingContext()
    state = _bind(_STATES.SequentialVibrationState("sequence_vibration", 11, session))
    state.on_enter(ctx)
    state.on_action(ctx, "stop")
    state.stop_started_at -= 0.51
    state.on_update(ctx, 0.005)
    assert session.faulted
    assert ctx.frame is None
    assert ctx.requests[-1][0] == _STATES.ZERO_TORQUE


def test_sequence_run_stops_on_any_joint_temperature_limit():
    session = _STATES.TestSession()
    session.ready = True
    ctx = _Context()
    ctx.actuator_temperatures = _temperature_sample()
    state = _bind(_STATES.SequentialRunningState("sequence_running", 12, session))
    state.on_enter(ctx)
    state.on_update(ctx, 0.005)
    assert ctx.frame is not None
    motor = list(ctx.actuator_temperatures.motor_c)
    motor[-1] = state.motor_limit_c
    ctx.actuator_temperatures = _temperature_sample(motor=motor)
    ctx.frame = None
    state.on_update(ctx, 0.005)
    assert ctx.frame is None
    assert ctx.requests[-1] == (_STATES.ZERO_TORQUE, "test_safety_fault", True)
    assert JOINT_NAMES[-1] in state.logger.errors[-1]
    assert "60.0 C" in state.logger.errors[-1]


def test_sequence_temperature_check_uses_names_not_message_order():
    ctx = _Context()
    reversed_names = tuple(reversed(JOINT_NAMES))
    motor = [35.0] * len(reversed_names)
    motor[-1] = 61.0
    ctx.actuator_temperatures = _temperature_sample(
        names=reversed_names, motor=motor,
    )
    reason = _STATES.temperature_fault(
        ctx, motor_limit_c=60.0, driver_limit_c=0.0, timeout_sec=0.5,
    )
    assert JOINT_NAMES[0] in reason


def test_sequence_temperature_check_includes_extra_published_joint():
    ctx = _Context()
    names = (*JOINT_NAMES, "head_y_joint")
    ctx.actuator_temperatures = _temperature_sample(
        names=names,
        motor=[35.0] * len(JOINT_NAMES) + [61.0],
    )
    reason = _STATES.temperature_fault(
        ctx, motor_limit_c=60.0, driver_limit_c=0.0, timeout_sec=0.5,
    )
    assert "head_y_joint" in reason


def test_sequence_rejects_missing_temperature_before_joint_motion():
    session = _STATES.TestSession()
    session.ready = True
    ctx = _Context()
    state = _bind(_STATES.SequentialLimbTestState("sequence_joint", 10, session))
    state.on_enter(ctx)
    assert session.faulted
    assert ctx.frame is None
    assert ctx.requests[-1][0] == _STATES.ZERO_TORQUE


@pytest.mark.parametrize("params", [
    {"motor_limit_c": 0.0},
    {"motor_limit_c": float("nan")},
    {"driver_limit_c": -1.0},
    {"temperature_timeout_sec": 0.0},
])
def test_sequence_rejects_invalid_temperature_limits(params):
    with pytest.raises(ValueError):
        _STATES.SequentialRunningState(
            "sequence_running", 12, _STATES.TestSession(), **params,
        )


def test_actuator_callback_captures_named_temperature_snapshot():
    from bxi_example_py_elf3.bxi_example_demo import BxiExample

    updates = []
    node = SimpleNamespace(
        lock_in=RLock(),
        _actuator_temperatures=None,
        _update_joint_state=lambda *args: updates.append(args),
    )
    msg = SimpleNamespace(
        name=[JOINT_NAMES[1], JOINT_NAMES[0]],
        position=[0.0, 0.0],
        velocity=[0.0, 0.0],
        motor_temperature=[41.0, 42.0],
        driver_temperature=[43.0, 44.0],
    )
    BxiExample.actuator_callback(node, msg)
    assert node._actuator_temperatures.names == tuple(msg.name)
    assert node._actuator_temperatures.motor_c == (41.0, 42.0)
    assert node._actuator_temperatures.driver_c == (43.0, 44.0)
    assert updates == [(msg.name, msg.position, msg.velocity)]


@pytest.mark.parametrize("case, reason", [
    ("missing", "missing"),
    ("stale", "stale"),
    ("missing_joint", "missing joint"),
    ("invalid_motor", "invalid motor"),
    ("hot_driver", "driver temperature"),
])
def test_sequence_run_fails_closed_on_bad_temperature_feedback(case, reason):
    session = _STATES.TestSession()
    session.ready = True
    ctx = _Context()
    ctx.actuator_temperatures = {
        "missing": None,
        "stale": _temperature_sample(age=1.0),
        "missing_joint": _temperature_sample(names=JOINT_NAMES[:-1]),
        "invalid_motor": _temperature_sample(
            motor=[float("nan")] + [35.0] * (len(JOINT_NAMES) - 1)
        ),
        "hot_driver": _temperature_sample(
            driver=[70.0] + [35.0] * (len(JOINT_NAMES) - 1)
        ),
    }[case]
    state = _bind(_STATES.SequentialRunningState(
        "sequence_running", 12, session, driver_limit_c=60.0,
    ))
    state.on_enter(ctx)
    assert session.faulted
    assert ctx.requests[-1][0] == _STATES.ZERO_TORQUE
    assert reason in state.logger.errors[-1]


def test_test_exit_does_not_trigger_pd_when_left_shoulder_is_released_first():
    from bxi_example_py_elf3.bxi_example_demo import BxiExample

    adapter = RemoteEventAdapter({
        "com.bxi.suspended_tests/exit_test_mode": {"slot": "btn_8", "value": 2},
        "com.bxi.basic_actions/pd_brake": {"slot": "btn_3", "value": 1},
        "com.bxi.basic_actions/initial_pos": {"slot": "btn_4", "value": 1},
    })
    guard = _GUARD.TestRemoteGuard()
    state = [_STATES.TEST_IDLE]

    def extract(msg, *, sync_only=False):
        events = adapter.extract_events(msg, sync_only=sync_only)
        return guard(msg, events, state[0])

    node = SimpleNamespace(
        runtime=SimpleNamespace(extract_remote_events=extract),
        step=2,
        lock_in=RLock(),
        raw_cmd_vel=np.zeros(3, dtype=np.float32),
        pending_remote_events=deque(),
    )

    def send(btn_8=0, btn_3=0, btn_4=0):
        msg = SimpleNamespace(
            btn_8=btn_8, btn_3=btn_3, btn_4=btn_4,
            vel_des=SimpleNamespace(x=0.0, y=0.0), yawdot_des=0.0,
        )
        BxiExample.joy_callback(node, msg)

    send(btn_4=1)
    assert list(node.pending_remote_events) == []
    send(btn_8=2)
    assert list(node.pending_remote_events) == ["com.bxi.suspended_tests/exit_test_mode"]
    node.pending_remote_events.clear()
    state[0] = _STATES.ZERO_TORQUE
    send(btn_8=3, btn_3=1)
    assert list(node.pending_remote_events) == []
    send(btn_8=3, btn_3=1, btn_4=1)
    assert list(node.pending_remote_events) == []
    send(btn_8=3)
    assert list(node.pending_remote_events) == []
    send()
    send(btn_8=3, btn_3=1)
    assert list(node.pending_remote_events) == ["com.bxi.basic_actions/pd_brake"]


def test_test_exit_waits_for_all_button_slots_to_be_neutral():
    guard = _GUARD.TestRemoteGuard()
    test_state = _STATES.TEST_IDLE
    basic_state = _STATES.ZERO_TORQUE

    def filtered(state, events=(), **buttons):
        message = SimpleNamespace(**buttons)
        return guard(message, events, state)

    assert filtered(test_state, [_GUARD._EXIT], btn_8=2) == [_GUARD._EXIT]
    assert filtered(basic_state, ["com.bxi.basic_actions/pd_brake"], btn_8=2) == []
    assert filtered(basic_state, [], btn_8=3) == []
    assert filtered(basic_state, ["com.bxi.basic_actions/initial_pos"], btn_4=1) == []
    assert filtered(basic_state, ["com.bxi.basic_actions/pd_brake"], btn_3=1) == []
    assert filtered(basic_state) == []
    assert filtered(basic_state, ["com.bxi.basic_actions/pd_brake"], btn_3=1) == [
        "com.bxi.basic_actions/pd_brake"
    ]


def test_test_remote_guard_blocks_basic_buttons_and_covers_fault_exit():
    guard = _GUARD.TestRemoteGuard()
    test_state = _STATES.TEST_IDLE
    basic_state = _STATES.ZERO_TORQUE
    pd = "com.bxi.basic_actions/pd_brake"
    run = "com.bxi.suspended_tests/running"
    sequence = "com.bxi.suspended_tests/sequence"

    assert guard(SimpleNamespace(btn_3=1), [pd, run, sequence], test_state) == [
        run, sequence,
    ]
    guard.observe_state(basic_state)
    assert guard(SimpleNamespace(btn_3=1), [pd], basic_state) == []
    assert guard(SimpleNamespace(), [], basic_state) == []
    assert guard(SimpleNamespace(btn_3=1), [pd], basic_state) == [pd]


def test_test_remote_guard_covers_fault_without_a_test_state_joy_message():
    guard = _GUARD.TestRemoteGuard()
    guard.observe_state(_STATES.TEST_IDLE)
    guard.observe_state(_STATES.ZERO_TORQUE)
    pd = "com.bxi.basic_actions/pd_brake"
    assert guard(SimpleNamespace(btn_3=1), [pd], _STATES.ZERO_TORQUE) == []


def test_full_range_requires_center_before_transition():
    session = _STATES.TestSession()
    idle = _bind(_STATES.SuspendedIdleState("com.bxi.suspended_tests/idle", 1, session))
    limb = _bind(
        _STATES.SuspendedLimbTestState(
            "com.bxi.suspended_tests/whole_body_joint_test", 2, session
        )
    )
    ctx = _Context()
    idle.on_prepare(ctx, SimpleNamespace(name="com.bxi.basic_actions/pd_brake"))
    idle.on_enter(ctx)
    idle.entered_at -= idle.prepare_sec + 0.1
    ctx.robot_joints.position[0] += np.deg2rad(5.5)
    idle.on_update(ctx, 0.005)
    assert not session.ready
    assert not limb.is_available(ctx)
    assert not session.faulted
    assert ctx.requests == []

    ctx.robot_joints.position[0] = JOINT_NOMINAL_POS[0]
    idle.on_update(ctx, 0.005)
    assert session.ready
    assert limb.is_available(ctx)


def test_running_vibration_and_limb_states_emit_single_frame():
    session = _STATES.TestSession()
    session.ready = True
    ctx = _Context()
    states = (
        _bind(_STATES.SuspendedRunningState("com.bxi.suspended_tests/running", 2, session)),
        _bind(_STATES.SuspendedVibrationState("com.bxi.suspended_tests/vibration", 3, session)),
        _bind(_STATES.SuspendedLimbTestState("com.bxi.suspended_tests/whole_body_joint_test", 4, session)),
    )
    for state in states:
        state.on_enter(ctx)
        state.on_update(ctx, 0.005)
        assert ctx.frame is not None
        assert np.all(np.isfinite(ctx.frame.qpos))
        np.testing.assert_allclose(ctx.frame.kp, JOINT_KP, rtol=1e-5)
        np.testing.assert_allclose(ctx.frame.kd, JOINT_KD, rtol=1e-5)
        assert state.on_action(ctx, "stop")
        state.stop_started_at -= 0.51
        state.on_update(ctx, 0.005)
        np.testing.assert_allclose(ctx.frame.kp, JOINT_KP * 1.1, rtol=1e-5)
        np.testing.assert_allclose(ctx.frame.kd, JOINT_KD * 1.05, rtol=1e-5)
        assert ctx.requests[-1][0] == _STATES.TEST_IDLE


def test_test_cycle_gap_latches_zero_torque():
    session = _STATES.TestSession()
    session.ready = True
    ctx = _Context()
    state = _bind(_STATES.SuspendedVibrationState("vibration", 5, session))
    state.on_enter(ctx)
    session.last_control_at -= 0.06
    state.on_update(ctx, 0.005)
    assert session.faulted
    assert ctx.requests[-1][0] == _STATES.ZERO_TORQUE


def test_only_active_scheduler_consumes_events_and_publishes():
    class Framework:
        current_state_name = "com.bxi.basic_actions/zero_torque"

        def update(self, observation, events, period):
            if events:
                self.current_state_name = (
                    _STATES.TEST_IDLE
                    if self.current_state_name == "com.bxi.basic_actions/zero_torque"
                    else "com.bxi.basic_actions/zero_torque"
                )
            return (self.current_state_name, period)

    class Platform:
        def __init__(self):
            self.events = ["toggle"]
            self.published = []

        def startup_step(self, now):
            return True

        def snapshot_control_inputs(self):
            events, self.events = self.events, []
            return object(), events

        def publish_motor_frame(self, frame):
            self.published.append(frame)

    runtime = object.__new__(RobotControlRuntime)
    runtime._framework_lock = RLock()
    runtime.framework = Framework()
    runtime._platform = Platform()
    runtime.scheduler = SimpleNamespace(period_sec=0.02, set_enabled=lambda value: None)
    runtime.test_scheduler = SimpleNamespace(period_sec=0.005, set_enabled=lambda value: None)
    owner_changes = []
    runtime.scheduler.set_enabled = lambda value: owner_changes.append(("basic", value))
    runtime.test_scheduler.set_enabled = lambda value: owner_changes.append(("test", value))

    assert not runtime._run_control_cycle(test_owner=True).active
    assert runtime._platform.events == ["toggle"]
    runtime._run_control_cycle(test_owner=False)
    assert runtime.framework.current_state_name == _STATES.TEST_IDLE
    assert not runtime._platform.published
    assert owner_changes == [("basic", False), ("test", True)]
    runtime._run_control_cycle(test_owner=True)
    assert runtime._platform.published == [(_STATES.TEST_IDLE, 0.005)]

    runtime._platform.events = ["toggle"]
    runtime._run_control_cycle(test_owner=True)
    assert runtime.framework.current_state_name == "com.bxi.basic_actions/zero_torque"
    assert len(runtime._platform.published) == 1
    assert owner_changes[-2:] == [("test", False), ("basic", True)]
    runtime._run_control_cycle(test_owner=False)
    assert runtime._platform.published[-1][1] == 0.02


def test_inactive_scheduler_pauses_until_enabled():
    fired = Event()
    cycles = []

    def cycle():
        cycles.append(time.monotonic())
        fired.set()
        return ControlCycleResult(state="test")

    scheduler = ControlScheduler(
        cycle,
        period_sec=0.005,
        compute_budget_sec=0.003,
        logger=_Logger(),
    )
    scheduler.set_enabled(False)
    scheduler.start()
    try:
        assert not fired.wait(0.02)
        scheduler.set_enabled(True)
        assert fired.wait(0.2)
        scheduler.set_enabled(False)
        time.sleep(0.015)
        paused_count = len(cycles)
        time.sleep(0.02)
        assert len(cycles) == paused_count
    finally:
        scheduler.stop()


def test_full_mod_routes_and_timer_handoff(monkeypatch):
    import rclpy

    from bxi_example_py_elf3.bxi_example_demo import BxiExample, _CPU_AFFINITY_PLAN
    from bxi_example_py_elf3.framework.joints import JointStateBuffer
    from bxi_example_py_elf3.framework.platform import RobotObservation

    monkeypatch.setattr(
        "bxi_example_py_elf3.bxi_example_demo.get_package_share_directory",
        lambda package: str(_STATES_PATH.parents[2]),
    )
    monkeypatch.setenv("ROS_DOMAIN_ID", str(100 + os.getpid() % 100))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    rclpy.init()
    node = None
    try:
        node = BxiExample(cpu_affinity_plan=_CPU_AFFINITY_PLAN)
        assert any(
            type(event_filter).__name__ == "TestRemoteGuard"
            for event_filter in node.runtime.framework.mod_runtime.remote_event_filters
        )
        node.release_suspension = True
        node.topic_prefix = "simulation/"
        automatic_state_requests = []
        original_request_state = node.runtime.request_state

        def record_startup_request(*args, **kwargs):
            automatic_state_requests.append((args, kwargs))
            return original_request_state(*args, **kwargs)

        node.runtime.request_state = record_startup_request

        class ResetFuture:
            ready = False

            def done(self):
                return self.ready

            def result(self):
                return SimpleNamespace(is_success=True)

        class ResetClient:
            def __init__(self):
                self.calls = []

            def wait_for_service(self, timeout_sec):
                return True

            def call_async(self, request):
                future = ResetFuture()
                self.calls.append((request.reset_step, request.release, future))
                return future

        reset_client = ResetClient()
        node.rest_srv = reset_client
        now = time.monotonic()
        assert not node.startup_step(now)
        assert node.step == 0
        assert not node.startup_step(now + 0.1)
        assert node.step == 0
        reset_client.calls[0][2].ready = True
        assert not node.startup_step(now + 0.2)
        assert node.step == 1
        assert not node.startup_step(now + 1.3)
        assert reset_client.calls[1][:2] == (2, True)
        assert node.step == 1
        reset_client.calls[1][2].ready = True
        assert not node.startup_step(now + 1.4)
        assert node.step == 2
        assert automatic_state_requests == [
            (("com.bxi.basic_actions/pd_brake",), {"trigger": "AutoPdbreak"})
        ]
        node.runtime.request_state = original_request_state

        joints = JointStateBuffer(JointLayout(JOINT_NAMES))
        joints.update(
            JOINT_NOMINAL_POS,
            np.zeros(len(JOINT_NAMES)),
            timestamp_ns=1,
        )
        observation = RobotObservation(
            joints=joints.view,
            quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
            omega=np.zeros(3),
            raw_cmd_vel=np.zeros(3, dtype=np.float32),
            linear_acceleration=np.zeros(3),
            actuator_temperatures=_temperature_sample(),
        )

        class Platform:
            def __init__(self):
                self.events = []
                self.published = []

            def startup_step(self, now):
                return True

            def snapshot_control_inputs(self):
                events, self.events = self.events, []
                return observation, events

            def publish_motor_frame(self, frame):
                self.published.append(frame.layout.names)

        platform = Platform()
        runtime = node.runtime
        runtime._platform = platform

        runtime._run_control_cycle(test_owner=False)
        assert runtime.framework.actuator_temperatures is observation.actuator_temperatures
        assert runtime.current_state_name == "com.bxi.basic_actions/pd_brake"
        node.topic_prefix = "hardware/"
        platform.events = ["com.bxi.suspended_tests/test_mode"]
        runtime._run_control_cycle(test_owner=False)
        assert runtime.current_state_name == "com.bxi.basic_actions/pd_brake"
        node.topic_prefix = "simulation/"
        platform.events = ["com.bxi.suspended_tests/test_mode"]
        runtime._run_control_cycle(test_owner=False)
        assert runtime.current_state_name == _STATES.TEST_IDLE
        platform.events = ["com.bxi.suspended_tests/test_mode"]
        runtime._run_control_cycle(test_owner=True)
        assert runtime.current_state_name == _STATES.TEST_IDLE
        platform.events = ["com.bxi.suspended_tests/exit_test_mode"]
        runtime._run_control_cycle(test_owner=True)
        assert runtime.current_state_name == _STATES.ZERO_TORQUE
        runtime._run_control_cycle(test_owner=False)
        platform.events = ["com.bxi.basic_actions/forward_back"]
        runtime._run_control_cycle(test_owner=False)
        assert runtime.current_state_name == _STATES.ZERO_TORQUE

        assert runtime.request_state(
            "com.bxi.basic_actions/normal", trigger="test_setup", force=True
        )
        runtime._run_control_cycle(test_owner=False)
        assert runtime.current_state_name == "com.bxi.basic_actions/normal"
        observation.raw_cmd_vel[:] = (0.8, 0.4, 0.5)
        platform.events = ["com.bxi.basic_actions/forward_back"]
        runtime._run_control_cycle(test_owner=False)
        assert runtime.current_state_name == "com.bxi.basic_actions/forward_back"
        runtime._run_control_cycle(test_owner=False)
        np.testing.assert_allclose(runtime.framework.current_cmd_vel, [0.5, 0.0, 0.0])

        platform.events = ["com.bxi.basic_actions/zero_torque"]
        runtime._run_control_cycle(test_owner=False)
        assert runtime.current_state_name == _STATES.ZERO_TORQUE
        assert runtime.request_state(
            "com.bxi.basic_actions/normal", trigger="test_setup", force=True
        )
        runtime._run_control_cycle(test_owner=False)
        platform.events = ["com.bxi.basic_actions/forward_back"]
        runtime._run_control_cycle(test_owner=False)
        assert runtime.current_state_name == "com.bxi.basic_actions/forward_back"
        platform.events = ["com.bxi.basic_actions/forward_back"]
        runtime._run_control_cycle(test_owner=False)
        assert runtime.current_state_name == "com.bxi.basic_actions/normal"
        platform.events = ["com.bxi.basic_actions/zero_torque"]
        runtime._run_control_cycle(test_owner=False)
        assert runtime.current_state_name == _STATES.ZERO_TORQUE
        observation.raw_cmd_vel.fill(0.0)
        platform.published.clear()

        platform.events = ["com.bxi.suspended_tests/test_mode"]
        runtime._run_control_cycle(test_owner=False)
        assert runtime.current_state_name == _STATES.TEST_IDLE
        assert platform.published == []

        runtime._run_control_cycle(test_owner=True)
        assert len(platform.published) == 1
        idle = runtime.framework.state_machine._states[_STATES.TEST_IDLE]
        assert idle.prepare_sec == 3.0
        assert idle.prepare_kp_scale == 1.1
        assert idle.center_kd_scale == 1.05
        idle.entered_at -= 10.1
        runtime._run_control_cycle(test_owner=True)
        assert idle.session.ready

        observation.actuator_temperatures = _temperature_sample()
        platform.events = ["com.bxi.suspended_tests/sequence"]
        runtime._run_control_cycle(test_owner=True)
        assert runtime.current_state_name == "com.bxi.suspended_tests/sequence_joint"
        platform.events = ["com.bxi.suspended_tests/exit_test_mode"]
        runtime._run_control_cycle(test_owner=True)
        assert runtime.current_state_name == _STATES.ZERO_TORQUE
        platform.events = ["com.bxi.suspended_tests/test_mode"]
        runtime._run_control_cycle(test_owner=False)
        assert runtime.current_state_name == _STATES.TEST_IDLE
        idle.entered_at -= 10.1
        runtime._run_control_cycle(test_owner=True)
        assert idle.session.ready

        platform.events = ["com.bxi.suspended_tests/vibration"]
        runtime._run_control_cycle(test_owner=True)
        assert runtime.current_state_name == "com.bxi.suspended_tests/vibration"

        platform.events = ["com.bxi.suspended_tests/vibration"]
        runtime._run_control_cycle(test_owner=True)
        vibration = runtime.framework.state_machine._states[
            "com.bxi.suspended_tests/vibration"
        ]
        assert vibration.stopping
        vibration.stop_started_at -= 0.51
        runtime._run_control_cycle(test_owner=True)
        assert runtime.current_state_name == _STATES.TEST_IDLE

        platform.events = ["com.bxi.suspended_tests/exit_test_mode"]
        count = len(platform.published)
        runtime._run_control_cycle(test_owner=True)
        assert runtime.current_state_name == _STATES.ZERO_TORQUE
        assert len(platform.published) == count
        runtime._run_control_cycle(test_owner=False)
        assert len(platform.published) == count + 1

        platform.events = ["com.bxi.suspended_tests/test_mode"]
        runtime._run_control_cycle(test_owner=False)
        assert runtime.current_state_name == _STATES.TEST_IDLE
        idle.entered_at -= 10.1
        runtime._run_control_cycle(test_owner=True)
        platform.events = ["com.bxi.suspended_tests/vibration"]
        runtime._run_control_cycle(test_owner=True)
        assert runtime.current_state_name == "com.bxi.suspended_tests/vibration"
        platform.events = ["com.bxi.basic_actions/zero_torque"]
        runtime._run_control_cycle(test_owner=True)
        assert runtime.current_state_name == "com.bxi.suspended_tests/vibration"
        platform.events = ["com.bxi.basic_actions/pd_brake", "com.bxi.suspended_tests/test_mode"]
        runtime._run_control_cycle(test_owner=True)
        assert runtime.current_state_name == "com.bxi.suspended_tests/vibration"
        platform.events = ["com.bxi.suspended_tests/exit_test_mode"]
        count = len(platform.published)
        runtime._run_control_cycle(test_owner=True)
        assert runtime.current_state_name == _STATES.ZERO_TORQUE
        assert len(platform.published) == count

        platform.events = ["com.bxi.suspended_tests/test_mode"]
        runtime._run_control_cycle(test_owner=False)
        assert runtime.current_state_name == _STATES.TEST_IDLE
        assert runtime.request_state(
            "com.bxi.basic_actions/imu_protection",
            trigger="imu_timeout",
            force=True,
        )
        count = len(platform.published)
        runtime._run_control_cycle(test_owner=True)
        assert runtime.current_state_name == "com.bxi.basic_actions/imu_protection"
        assert len(platform.published) == count
        runtime._run_control_cycle(test_owner=False)
        assert len(platform.published) == count + 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
