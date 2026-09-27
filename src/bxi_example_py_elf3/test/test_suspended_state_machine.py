"""Offline checks for suspended-test state and timer ownership."""

from __future__ import annotations

from pathlib import Path
from threading import Event, RLock
from types import SimpleNamespace
import importlib.util
import os
import time

import numpy as np

from bxi_example_py_elf3.control.elf3 import JOINT_NAMES, JOINT_NOMINAL_POS
from bxi_example_py_elf3.framework.joints import JointLayout
from bxi_example_py_elf3.framework.platform.runtime import RobotControlRuntime
from bxi_example_py_elf3.framework.runtime.control_scheduler import (
    ControlCycleResult,
    ControlScheduler,
)


_STATES_PATH = (
    Path(__file__).resolve().parents[1]
    / "mods/com.bxi.suspended_tests/control_states.py"
)
_SPEC = importlib.util.spec_from_file_location("suspended_control_states_test", _STATES_PATH)
assert _SPEC is not None and _SPEC.loader is not None
_STATES = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_STATES)


class _Logger:
    def info(self, message):
        pass

    def warning(self, message):
        pass

    def error(self, message):
        pass


class _Context:
    def __init__(self):
        self.robot_layout = JointLayout(JOINT_NAMES)
        self.robot_joints = SimpleNamespace(
            timestamp_ns=1,
            position=JOINT_NOMINAL_POS.copy(),
        )
        self.frame = None
        self.requests = []

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

    session.last_feedback_at -= 0.3
    idle.on_update(ctx, 0.005)
    assert session.faulted
    assert ctx.requests[-1][0] == _STATES.ZERO_TORQUE


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
    idle.entered_at -= 10.1
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
        assert state.on_action(ctx, "stop")
        state.stop_started_at -= 0.51
        state.on_update(ctx, 0.005)
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

    monkeypatch.setenv("ROS_DOMAIN_ID", str(100 + os.getpid() % 100))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    rclpy.init()
    node = None
    try:
        node = BxiExample(cpu_affinity_plan=_CPU_AFFINITY_PLAN)
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
        assert runtime.current_state_name == "com.bxi.basic_actions/pd_brake"
        assert runtime.request_state(
            _STATES.ZERO_TORQUE, trigger="test_setup", force=True
        )
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
        np.testing.assert_allclose(runtime.framework.current_cmd_vel, [0.3, 0.0, 0.0])

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

        platform.events = ["com.bxi.suspended_tests/test_mode"]
        count = len(platform.published)
        runtime._run_control_cycle(test_owner=True)
        assert runtime.current_state_name == "com.bxi.basic_actions/pd_brake"
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
