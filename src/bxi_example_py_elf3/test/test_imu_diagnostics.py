"""Platform receive/lifecycle regression tests. Never start hardware."""
import importlib
import json
from threading import Event, Lock
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest

from bxi_example_py_elf3.imu_diagnostics import ImuReceiveDiagnostics
from bxi_example_py_elf3.framework.platform.runtime import RobotControlRuntime
from bxi_example_py_elf3.framework.runtime.control_scheduler import ControlScheduler


class Clock:
    value = 1_000_000_000

    def __call__(self):
        return self.value

    def advance(self, ms):
        self.value += int(ms * 1e6)


def message():
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
        orientation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0),
        angular_velocity=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        linear_acceleration=SimpleNamespace(x=0.0, y=0.0, z=9.8),
    )


@pytest.fixture
def demo_class():
    with patch("bxi_example_py_elf3.framework.platform.cpu_affinity.bootstrap_process_scheduling"):
        return importlib.import_module("bxi_example_py_elf3.bxi_example_demo").BxiExample


def diagnostics(clock):
    return ImuReceiveDiagnostics(clock=clock, wall_clock=clock)


def test_waiting_imu_callback_is_distinct_from_no_callback_entry():
    clock = Clock()
    diag = diagnostics(clock)
    diag.begin("imu", message())
    diag.accepted(0)
    diag.end("imu")
    clock.advance(121)
    assert diag.snapshot()["hint"] == "no_imu_entry_delivery_or_executor_unknown"
    diag.begin("imu", message())
    diag.phase("imu", "wait_input_lock")
    clock.advance(80)
    result = diag.snapshot()
    assert result["hint"] == "imu_callback_waiting_input_lock"
    assert result["active_callbacks"]["imu"]["age_ms"] == 80
    diag.accepted(80_000_000)
    diag.end("imu")
    result = diag.snapshot()
    assert result["max_input_lock_wait_ms"] == 80
    assert result["max_entry_gap_ms"] == 121
    assert result["max_callback_ms"] == 80
    assert result["hint"] == "recent_valid_update"


def test_blocking_other_sensor_and_invalid_frames_have_distinct_hints():
    clock = Clock()
    diag = diagnostics(clock)
    diag.begin("joints", object())
    clock.advance(60)
    assert diag.snapshot()["hint"] == "sensor_callback_group_busy"
    diag.end("joints")
    diag.begin("imu", message())
    diag.rejected()
    diag.end("imu")
    assert diag.snapshot()["hint"] == "entered_but_no_valid_update"
    assert diag.snapshot()["rejected"] == 1


def test_history_bounded_and_window_reset_keeps_liveness():
    clock = Clock()
    diag = diagnostics(clock)
    for _ in range(100):
        diag.begin("imu", message())
        diag.accepted(0)
        diag.end("imu")
        clock.advance(60)
    result = diag.snapshot(reset_window=True)
    assert len(result["recent"]) == 16
    assert result["callback_counts"]["imu"] == 100
    assert result["max_entry_gap_ms"] == 60
    assert diag.snapshot()["max_entry_gap_ms"] == 0
    assert diag.snapshot()["last_valid_update_age_ms"] == 60
    assert json.loads(json.dumps(result)) == result


def test_wrapper_finally_clears_active_callback_after_exception():
    clock = Clock()
    diag = diagnostics(clock)
    def fail(_):
        raise ValueError("bad input")
    with pytest.raises(ValueError):
        diag.wrap("imu", fail)(message())
    assert diag.snapshot()["active_callbacks"] == {}
    assert diag.snapshot()["callback_errors"] == 1


def platform():
    return SimpleNamespace(
        _shutting_down=Event(), context=SimpleNamespace(ok=lambda: True),
        imu_required=True, lock_in=Lock(), _imu_last_received_at=1.0,
        _imu_received=True, _imu_protection_latched=False,
        imu_runtime_timeout_sec=1.0, _imu_diagnostics=ImuReceiveDiagnostics(),
        runtime=Mock(current_state_name="com.bxi.basic_actions/normal"),
        get_logger=Mock(return_value=Mock()),
    )


@pytest.mark.parametrize("stop_event,context_ok", [(True, True), (False, False)])
def test_expected_shutdown_does_not_latch_or_log_imu_failure(demo_class, stop_event, context_ok):
    node = platform()
    if stop_event:
        node._shutting_down.set()
    node.context.ok = lambda: context_ok
    demo_class._check_imu_runtime_safety(node)
    assert not node._imu_protection_latched
    node.runtime.request_state.assert_not_called()
    node.get_logger.assert_not_called()


def test_real_runtime_outage_still_latches_with_evidence(demo_class):
    node = platform()
    with patch("bxi_example_py_elf3.bxi_example_demo.time.monotonic", return_value=2.1):
        demo_class._check_imu_runtime_safety(node)
    assert node._imu_protection_latched
    node.runtime.request_state.assert_called_once_with(
        "com.bxi.basic_actions/imu_protection", trigger="imu_timeout", force=True)
    text = node.get_logger().error.call_args.args[0]
    assert "no frame for 1.100s" in text
    assert "imu_receive_diag=" in text


def test_shutdown_callback_only_requests_cancellation(demo_class):
    node = platform()
    demo_class._begin_shutdown(node)
    assert node._shutting_down.is_set()
    node.runtime.request_stop.assert_called_once_with()
    node.runtime.close.assert_not_called()


def test_context_shutdown_suppresses_motor_publication(demo_class):
    node = platform()
    node._shutting_down.set()
    node.act_pub = Mock()
    demo_class.publish_motor_frame(node, object())
    node.act_pub.publish.assert_not_called()


def test_runtime_request_stop_is_nonblocking():
    runtime = object.__new__(RobotControlRuntime)
    runtime._stop_event = Event()
    runtime.scheduler = Mock()
    runtime.request_stop()
    assert runtime._stop_event.is_set()
    runtime.scheduler.request_stop.assert_called_once_with()
    runtime.scheduler.stop.assert_not_called()
    scheduler = ControlScheduler(lambda: None, period_sec=.02,
                                 compute_budget_sec=.005, logger=Mock())
    scheduler.request_stop()
    assert scheduler._stop_event.is_set()


def test_no_motor_frame_is_published_after_midcycle_shutdown():
    runtime = object.__new__(RobotControlRuntime)
    runtime._stop_event = Event()
    runtime._framework_lock = Lock()
    runtime._platform = Mock()
    runtime._platform.startup_step.return_value = True
    runtime._platform.snapshot_control_inputs.return_value = (object(), ())
    runtime.scheduler = SimpleNamespace(period_sec=.02)
    runtime.framework = Mock(current_state_name="normal", desired_control_period=.02)
    def update(*_):
        runtime._stop_event.set()
        return object()
    runtime.framework.update.side_effect = update
    runtime._run_control_cycle()
    runtime._platform.publish_motor_frame.assert_not_called()


def test_imu_wrapper_preserves_validation_and_counts_accepted_rejected(demo_class):
    from bxi_example_py_elf3.imu_freshness import ImuStampObserver
    node = platform()
    node._imu_stamp_observer = ImuStampObserver()
    node._imu_stamp_warning_count = 0
    node._next_imu_stamp_warning = float("inf")
    node._imu_invalid_frame_count = 0
    node._next_imu_invalid_warning = float("inf")
    node._imu_first_received_logged = True
    node.quat_xyzw = np.zeros(4)
    node.quat_wxyz = np.zeros(4)
    node.omega = np.zeros(3)
    node.linear_acceleration = np.zeros(3)
    callback = node._imu_diagnostics.wrap("imu", lambda m: demo_class.imu_callback(node, m))
    callback(message())
    accepted_at = node._imu_last_received_at
    bad = message()
    bad.angular_velocity.x = float("nan")
    callback(bad)
    assert node._imu_last_received_at == accepted_at
    result = node._imu_diagnostics.snapshot()
    assert result["accepted"] == 1 and result["rejected"] == 1
    assert result["callback_counts"]["imu"] == 2
    assert result["active_callbacks"] == {}


def test_callback_diagnostics_remain_readable_while_input_lock_is_blocked(demo_class):
    from threading import Thread
    from bxi_example_py_elf3.imu_freshness import ImuStampObserver
    entered = Event()
    release = Event()

    class BlockingLock:
        def __enter__(self):
            entered.set()
            assert release.wait(2.0)

        def __exit__(self, *_):
            pass

    node = platform()
    node.lock_in = BlockingLock()
    node._imu_stamp_observer = ImuStampObserver()
    node._imu_stamp_warning_count = 0
    node._next_imu_stamp_warning = float("inf")
    node._imu_first_received_logged = True
    node.quat_xyzw = np.zeros(4)
    node.quat_wxyz = np.zeros(4)
    node.omega = np.zeros(3)
    node.linear_acceleration = np.zeros(3)
    errors = []
    def callback():
        try:
            node._imu_diagnostics.wrap("imu", lambda m: demo_class.imu_callback(node, m))(message())
        except BaseException as exc:
            errors.append(exc)
    worker = Thread(target=callback, daemon=True)
    try:
        worker.start()
        assert entered.wait(1.0)
        result = node._imu_diagnostics.snapshot()
        assert result["hint"] == "imu_callback_waiting_input_lock"
        assert result["accepted"] == 0
    finally:
        release.set()
        worker.join(2.0)
    assert not worker.is_alive() and not errors
    assert node._imu_diagnostics.snapshot()["accepted"] == 1
