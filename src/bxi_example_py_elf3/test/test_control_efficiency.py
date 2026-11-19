"""Common runtime scheduling, input isolation and status-wire regressions.

Run with the ROS/BXI environment sourced; tests never construct a ROS node or
start a robot. Model checks use a temporary identity graph, not Mod assets.
"""

from collections import deque
import importlib
import json
from threading import Event, Lock, Thread
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest

from bxi_example_py_elf3.framework.platform.runtime import (
    ControlRuntimeConfig,
    RobotControlRuntime,
)
from bxi_example_py_elf3.framework.platform.status import CachedGraphStatusEncoder
from bxi_example_py_elf3.framework.runtime.control_scheduler import ControlScheduler
from bxi_example_py_elf3.framework.runtime.state_machine import RemoteEventAdapter


@pytest.fixture
def demo_class():
    # Import the actual callbacks, but don't alter the test process's affinity.
    with patch(
        "bxi_example_py_elf3.framework.platform.cpu_affinity.bootstrap_process_scheduling"
    ):
        return importlib.import_module("bxi_example_py_elf3.bxi_example_demo").BxiExample


def _scheduler():
    return ControlScheduler(
        lambda: None, period_sec=0.02, compute_budget_sec=0.005, logger=Mock()
    )


def test_scheduler_sleeps_until_absolute_deadline():
    scheduler = _scheduler()
    scheduler._stop_event = Mock()
    scheduler._stop_event.is_set.return_value = False
    scheduler._stop_event.wait.return_value = False
    # Also exercise an early/spurious wake; the second sleep uses the same target.
    with patch(
        "bxi_example_py_elf3.framework.runtime.control_scheduler.time.monotonic_ns",
        side_effect=[1_000_000_000, 1_015_000_000, 1_020_000_000],
    ):
        assert scheduler._wait_until(1_020_000_000)
    assert [c.args[0] for c in scheduler._stop_event.wait.call_args_list] == [
        0.02, 0.005
    ]


def test_scheduler_sleep_is_interruptible():
    scheduler = _scheduler()
    scheduler._stop_event = Mock()
    scheduler._stop_event.is_set.return_value = False
    scheduler._stop_event.wait.return_value = True
    with patch(
        "bxi_example_py_elf3.framework.runtime.control_scheduler.time.monotonic_ns",
        return_value=0,
    ):
        assert not scheduler._wait_until(10_000_000_000)
    scheduler._stop_event.wait.assert_called_once_with(10.0)


@pytest.mark.parametrize("period_sec,spin_wait_us", [(0.02, 200), (0.005, 100), (0.02, -1)])
def test_existing_scheduler_spin_configuration_remains_compatible(period_sec, spin_wait_us):
    config = ControlRuntimeConfig.from_mapping({
        "period_sec": period_sec, "maintenance_guard_sec": 0.001,
        "spin_wait_us": spin_wait_us,
    })
    scheduler = ControlScheduler(
        lambda: None, period_sec=period_sec, compute_budget_sec=0.002,
        logger=Mock(), spin_wait_us=config.spin_wait_us,
    )
    assert scheduler.spin_wait_ns == (-1 if spin_wait_us < 0 else spin_wait_us * 1000)
    with pytest.raises(ValueError, match="spin_wait_us"):
        ControlRuntimeConfig.from_mapping({"period_sec": period_sec, "maintenance_guard_sec": 0.001, "spin_wait_us": int(period_sec * 1e6)})


def _runtime_for_events():
    runtime = object.__new__(RobotControlRuntime)
    runtime._framework_lock = Lock()
    runtime._remote_event_lock = Lock()
    adapter = RemoteEventAdapter(
        {"activate": {"slot": "btn_1", "value": 1},
         "brake": {"slot": "btn_1", "value": 2}}
    )
    runtime.framework = SimpleNamespace(
        extract_remote_events=lambda value, sync_only=False: adapter.extract_events(
            value, sync_only=sync_only
        )
    )
    return runtime


def _motion(button):
    return SimpleNamespace(
        btn_1=button, vel_des=SimpleNamespace(x=0.2, y=-0.1), yawdot_des=0.3
    )


def test_remote_callback_finishes_while_control_lock_is_held(demo_class):
    runtime = _runtime_for_events()
    platform = SimpleNamespace(
        runtime=runtime, step=2, lock_in=Lock(),
        raw_cmd_vel=np.zeros(3), pending_remote_events=deque(),
    )
    done = Event()
    errors = []

    def callback():
        try:
            demo_class.joy_callback(platform, _motion(1))
        except BaseException as exc:
            errors.append(exc)
        finally:
            done.set()

    worker = Thread(target=callback, daemon=True)
    try:
        with runtime._framework_lock:
            worker.start()
            assert done.wait(1.0), "remote callback waited for control/inference"
    finally:
        worker.join(timeout=1.0)
    assert not errors
    assert list(platform.pending_remote_events) == ["activate"]
    np.testing.assert_allclose(platform.raw_cmd_vel, [0.2, -0.1, 0.3])


def test_remote_edge_order_and_startup_sync_are_preserved():
    runtime = _runtime_for_events()
    assert runtime.extract_remote_events(_motion(1), sync_only=True) == []
    assert runtime.extract_remote_events(_motion(1)) == []
    events = []
    for value in [0, 1, 1, 0, 1, 2, 2, 0]:
        events.extend(runtime.extract_remote_events(_motion(value)))
    assert events == ["activate", "activate", "brake"]


def test_remote_subscriptions_do_not_share_sensor_callback_group(demo_class):
    node = Mock(
        topic_prefix="hardware/", state_machine_info_topic="",
        motor_override_topic="hardware/actuators_cmds_override",
        motor_override_timeout_sec=0.2, motor_override_release_blend_sec=0.2,
        motor_override_allow_in_zero_torque=False,
    )
    demo_class.init_pub_sub(node)
    groups = {
        c.args[1]: c.kwargs["callback_group"]
        for c in node.create_subscription.call_args_list
    }
    assert groups["hardware/imu_data"] is node.io_callback_group
    assert groups["hardware/actuator_states"] is node.io_callback_group
    assert groups["motion_commands"] is node.remote_callback_group
    assert groups["hardware/actuators_cmds_override"] is node.remote_callback_group
    assert node.io_callback_group is not node.remote_callback_group


def test_graph_is_encoded_once_but_present_in_every_message():
    encoder = CachedGraphStatusEncoder()
    graph = {"states": [{"name": '走路"\\\n', "id": 1}], "transitions": []}
    first = {"graph": graph, "current": {"name": "idle"}, "loop_count": 1}
    assert encoder.needs_graph
    assert json.loads(encoder.encode(first)) == first
    assert first["graph"] is graph  # Encoding must not mutate the source snapshot.
    assert not encoder.needs_graph
    dumps = json.dumps
    with patch(
        "bxi_example_py_elf3.framework.platform.status.json.dumps", wraps=dumps
    ) as serialize:
        payload = encoder.encode({"current": {"name": "normal"}, "loop_count": 2})
    serialize.assert_called_once()
    assert "graph" not in serialize.call_args.args[0]
    decoded = json.loads(payload)
    assert decoded["graph"] == graph
    assert decoded["loop_count"] == 2
    assert decoded["current"]["name"] == "normal"


def test_graph_only_empty_and_explicit_replacement():
    encoder = CachedGraphStatusEncoder()
    with pytest.raises(ValueError, match="include a graph"):
        encoder.encode({"loop_count": 0})
    assert json.loads(encoder.encode({"graph": {}})) == {"graph": {}}
    assert json.loads(encoder.encode({})) == {"graph": {}}
    new_graph = {"states": [{"name": "new"}]}
    encoder.encode({"graph": new_graph})
    assert json.loads(encoder.encode({}))["graph"] == new_graph


def test_status_publisher_requests_graph_only_until_first_success(demo_class):
    graph = {"states": [{"name": "走路"}]}
    node = SimpleNamespace(
        _state_machine_info_encoder=CachedGraphStatusEncoder(), step=2,
        runtime=SimpleNamespace(snapshot=Mock(side_effect=[
            None, {"graph": graph, "loop_count": 1}, {"loop_count": 2},
        ])),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1_234_567_890)
        ),
        state_machine_info_pub=Mock(),
    )
    for _ in range(3):
        demo_class.publish_state_machine_info(node)
    assert [c.kwargs["include_graph"] for c in node.runtime.snapshot.call_args_list] == [
        True, True, False
    ]
    messages = node.state_machine_info_pub.publish.call_args_list
    assert len(messages) == 2
    for index, call in enumerate(messages, 1):
        decoded = json.loads(call.args[0].data)
        assert decoded["graph"] == graph
        assert decoded["loop_count"] == index
        assert decoded["stamp"] == {"sec": 1, "nanosec": 234_567_890}
        assert decoded["step"] == 2


def test_onnx_workers_sleep_and_io_binding_still_works(tmp_path):
    onnx = pytest.importorskip("onnx")
    pytest.importorskip("onnxruntime")
    from bxi_example_py_elf3.framework.inference.backends.onnxruntime import OnnxBackend
    from bxi_example_py_elf3.framework.inference.model import ModelSpec

    helper = onnx.helper
    model = helper.make_model(
        helper.make_graph(
            [helper.make_node("Identity", ["obs"], ["actions"])], "identity",
            [helper.make_tensor_value_info("obs", onnx.TensorProto.FLOAT, [1, 4])],
            [helper.make_tensor_value_info("actions", onnx.TensorProto.FLOAT, [1, 4])],
        ), opset_imports=[helper.make_opsetid("", 13)], ir_version=8,
    )
    path = tmp_path / "identity.onnx"
    onnx.save(model, str(path))
    spec = ModelSpec.onnx(path, providers=("CPUExecutionProvider",))
    backend = OnnxBackend(spec.artifacts[0], spec)
    try:
        options = backend._session.get_session_options()
        for key in ("session.intra_op.allow_spinning", "session.inter_op.allow_spinning"):
            assert options.get_session_config_entry(key) == "0"
        assert options.intra_op_num_threads == 4
        obs = np.arange(4, dtype=np.float32).reshape(1, 4)
        for _ in range(3):
            result = backend.run({"obs": obs})
            np.testing.assert_array_equal(result["actions"], obs)
            obs += 1
        assert backend._io_binding is not None
    finally:
        backend.close()


def _cpu_sample(sampler, path, *, user, idle, iowait=0, guest=0):
    path.write_text(
        f"cpu  {user} 0 0 {idle} {iowait} 0 0 0 {guest} 0\n"
        "cpu0 99999 0 0 99999 0 0 0 0 0 0\n", encoding="ascii",
    )
    sampler.sample()


def test_machine_cpu_uses_deltas_and_weighted_window_mean(tmp_path):
    from bxi_example_py_elf3.framework.platform.cpu_usage import SystemCpuUsage

    path = tmp_path / "stat"
    sampler = SystemCpuUsage(path)
    _cpu_sample(sampler, path, user=1000, idle=1000, iowait=100, guest=500)
    baseline = sampler.snapshot()
    assert baseline.samples == 0
    assert baseline.average is None
    # 20/100 = 20%; guest increments must not be double-counted.
    _cpu_sample(sampler, path, user=1020, idle=1070, iowait=110, guest=520)
    # 120/200 = 60%; weighted average is 140/300, not (20+60)/2.
    _cpu_sample(sampler, path, user=1140, idle=1140, iowait=120, guest=640)
    stats = sampler.snapshot(reset_window=True)
    assert stats.samples == 2
    assert stats.unavailable_samples == 0
    assert stats.minimum == pytest.approx(20.0)
    assert stats.maximum == pytest.approx(60.0)
    assert stats.average == pytest.approx(100.0 * 140 / 300)
    assert sampler.snapshot().average is None
    # Window reset keeps the baseline, and bounds are reset as well.
    _cpu_sample(sampler, path, user=1140, idle=1240, iowait=120, guest=640)
    stats = sampler.snapshot()
    assert stats.samples == 1
    assert stats.minimum == stats.maximum == stats.average == 0.0
    _cpu_sample(sampler, path, user=1240, idle=1240, iowait=120)
    assert sampler.snapshot().maximum == 100.0


@pytest.mark.parametrize("invalid", [
    None, "", "cpu0 1 2 3 4 5 6 7 8\n", "cpu 1 2 3\n",
    "cpu 1 2 x 4 5 6 7 8\n", "cpu -1 2 3 4 5 6 7 8\n", "\xff",
])
def test_machine_cpu_unavailable_is_not_reported_as_zero(tmp_path, invalid):
    from bxi_example_py_elf3.framework.platform.cpu_usage import SystemCpuUsage

    path = tmp_path / "stat"
    sampler = SystemCpuUsage(path)
    _cpu_sample(sampler, path, user=10, idle=10)
    if invalid is None:
        path.unlink()
    else:
        path.write_bytes(invalid.encode("latin1"))
    sampler.sample()  # Missing/malformed file must never stop the controller.
    stats = sampler.snapshot()
    assert stats.samples == 0
    assert stats.unavailable_samples == 1
    assert stats.minimum is stats.maximum is stats.average is None
    # Reappearance establishes a fresh baseline; don't span a blind interval.
    _cpu_sample(sampler, path, user=100, idle=100)
    assert sampler.snapshot().samples == 0
    _cpu_sample(sampler, path, user=110, idle=110)
    assert sampler.snapshot().average == 50.0


def test_machine_cpu_rebaselines_after_counter_reset_or_zero_delta(tmp_path):
    from bxi_example_py_elf3.framework.platform.cpu_usage import SystemCpuUsage

    path = tmp_path / "stat"
    sampler = SystemCpuUsage(path)
    for user, idle, iowait in [(100, 100, 10), (100, 100, 10), (10, 10, 1), (30, 30, 0)]:
        _cpu_sample(sampler, path, user=user, idle=idle, iowait=iowait)
    assert sampler.snapshot().samples == 0
    assert sampler.snapshot().unavailable_samples == 3
    _cpu_sample(sampler, path, user=40, idle=40)
    assert sampler.snapshot().average == 50.0


def _timing_runtime():
    from queue import SimpleQueue
    from bxi_example_py_elf3.framework.platform.cpu_usage import SystemCpuUsage

    runtime = RobotControlRuntime.__new__(RobotControlRuntime)
    runtime.config = ControlRuntimeConfig(compute_budget_sec=0.005)
    runtime._deadline_miss_queue = SimpleQueue()
    runtime._pending_deadline_summary = None
    runtime._last_deadline_warning_at = 0.0
    runtime._log = Mock(return_value=True)
    runtime._cpu_usage = SystemCpuUsage()
    runtime._framework_lock = Lock()
    runtime.framework = SimpleNamespace(current_state_name="com.bxi.basic_actions/normal")
    runtime._last_reported_timing_totals = {False: (0, 0, 0, 0), True: (0, 0, 0, 0)}
    runtime.scheduler = _scheduler()
    runtime.test_scheduler = ControlScheduler(
        lambda: None, period_sec=0.005, compute_budget_sec=0.003, logger=Mock()
    )
    runtime.test_config = ControlRuntimeConfig(
        period_sec=0.005, compute_budget_sec=0.003, maintenance_guard_sec=0.001,
    )
    return runtime


def _miss(*, wake=2.0, finish=0.0, count=1):
    return dict(state="idle", period_ms=20.0, wake_late_ms=wake,
                cycle_ms=3.0, finish_late_ms=finish, count=count)


def test_wake_only_misses_do_not_warn_or_pollute_completion_summary():
    runtime = _timing_runtime()
    for count in range(100):
        runtime._enqueue_deadline_miss(_miss(count=count + 1))
    runtime._drain_deadline_misses()
    runtime._log.assert_not_called()
    assert runtime._pending_deadline_summary is None
    assert runtime._deadline_miss_queue.empty()
    # Tolerance equality is not a completion miss either.
    runtime._enqueue_deadline_miss(_miss(finish=1.0))
    runtime._drain_deadline_misses()
    runtime._log.assert_not_called()
    runtime._enqueue_deadline_miss(_miss(wake=0.5, finish=2.0, count=101))
    with patch("bxi_example_py_elf3.framework.platform.runtime.time.monotonic", return_value=10.0):
        runtime._drain_deadline_misses()
    level, message = runtime._log.call_args.args
    assert level == "warning"
    assert "本次汇总异常周期1次" in message
    assert "调度唤醒延迟超限0次" in message
    assert "周期完成截止超限1次" in message
    assert runtime._pending_deadline_summary is None


def test_real_completion_warning_still_aggregates_and_throttles():
    runtime = _timing_runtime()
    runtime._last_deadline_warning_at = 10.0
    for now in (10.2, 10.8, 11.1):
        runtime._enqueue_deadline_miss(_miss(finish=2.0))
        with patch("bxi_example_py_elf3.framework.platform.runtime.time.monotonic", return_value=now):
            runtime._drain_deadline_misses()
    runtime._log.assert_called_once()
    assert "周期完成截止超限3次" in runtime._log.call_args.args[1]
    assert "调度唤醒延迟超限3次" in runtime._log.call_args.args[1]


def test_cpu_stats_in_periodic_info_and_reset_only_after_success(tmp_path):
    from bxi_example_py_elf3.framework.platform.cpu_usage import SystemCpuUsage

    runtime = _timing_runtime()
    path = tmp_path / "stat"
    runtime._cpu_usage = sampler = SystemCpuUsage(path)
    _cpu_sample(sampler, path, user=10, idle=10)
    _cpu_sample(sampler, path, user=20, idle=40)
    runtime._log.return_value = False
    runtime._report_control_timing()
    assert sampler.snapshot().samples == 1  # Keep data on logging failure.
    runtime._log.return_value = True
    runtime._report_control_timing()
    level, message = runtime._log.call_args.args
    assert level == "info"
    assert "整机CPU占用率：最低25.00%，最高25.00%，平均25.00%" in message
    assert "有效采样1次，无效采样0次" in message
    assert "P99唤醒延迟" in message  # Retain the quiet periodic timing summary.
    assert sampler.snapshot().samples == 0
    _cpu_sample(sampler, path, user=30, idle=50)
    runtime._report_control_timing()
    assert "最低50.00%，最高50.00%，平均50.00%" in runtime._log.call_args.args[1]
    runtime._report_control_timing()
    assert "整机CPU占用率：不可用" in runtime._log.call_args.args[1]


def test_wake_only_misses_remain_in_periodic_timing_statistics():
    from bxi_example_py_elf3.framework.runtime.control_scheduler import ControlCycleResult

    runtime = _timing_runtime()
    miss = runtime.scheduler._record_cycle(
        ControlCycleResult(state="idle"), period_ns=20_000_000,
        wake_late_ns=2_000_000, cycle_ns=3_000_000,
        frame_interval_ns=20_000_000, finish_late_ns=0,
        budget_overrun=False, deadline_missed=True, skipped_periods=0,
    )
    runtime._enqueue_deadline_miss(miss)
    runtime._drain_deadline_misses()
    runtime._log.assert_not_called()
    runtime._report_control_timing()
    level, message = runtime._log.call_args.args
    assert level == "info"
    assert "时序异常1次（占100.00%）" in message
    assert "P99唤醒延迟2.00 ms" in message


def test_cpu_sampling_runs_only_at_maintenance_rate():
    runtime = _timing_runtime()
    runtime._cpu_usage = Mock()
    runtime._stop_event = Mock()
    runtime._stop_event.is_set.return_value = False
    runtime._stop_event.wait.side_effect = [False, True]
    runtime._wait_for_maintenance_window = Mock(return_value=False)
    runtime._report_control_timing = Mock()
    runtime.config = ControlRuntimeConfig(statistics_interval_sec=0.1)
    with patch("bxi_example_py_elf3.framework.platform.runtime.time.monotonic",
               side_effect=[10.0, 10.0, 10.0, 10.2, 10.2, 10.2]):
        runtime._maintenance_loop()
    assert runtime._cpu_usage.sample.call_count == 2  # Baseline + one tick.
    runtime._report_control_timing.assert_called_once_with()


def test_stop_request_cancels_both_scheduler_owners_without_waiting():
    runtime = RobotControlRuntime.__new__(RobotControlRuntime)
    runtime._stop_event = Event()
    runtime.scheduler = Mock()
    runtime.test_scheduler = Mock()
    runtime.request_stop()
    assert runtime._stop_event.is_set()
    runtime.scheduler.request_stop.assert_called_once_with()
    runtime.test_scheduler.request_stop.assert_called_once_with()
    runtime.scheduler.stop.assert_not_called()
    runtime.test_scheduler.stop.assert_not_called()


def test_control_cycle_waiting_for_lock_does_not_update_after_stop():
    runtime = RobotControlRuntime.__new__(RobotControlRuntime)
    runtime._stop_event = Event()
    runtime.framework = Mock()
    runtime._platform = Mock()
    acquired = Lock()
    waiting = Event()
    results = []

    class FrameworkLock:
        def __enter__(self):
            waiting.set()
            acquired.acquire()

        def __exit__(self, *_):
            acquired.release()

    runtime._framework_lock = FrameworkLock()
    worker = Thread(target=lambda: results.append(
        runtime._run_control_cycle(test_owner=False)
    ), daemon=True)
    with acquired:
        worker.start()
        assert waiting.wait(1.0)
        runtime._stop_event.set()
    worker.join(timeout=1.0)
    assert not worker.is_alive()
    assert len(results) == 1 and not results[0].active
    runtime._platform.startup_step.assert_not_called()
    runtime.framework.update.assert_not_called()
    runtime._platform.publish_motor_frame.assert_not_called()


def test_runtime_snapshot_keeps_both_scheduler_statuses():
    runtime = RobotControlRuntime.__new__(RobotControlRuntime)
    runtime._framework_lock = Lock()
    runtime.config = ControlRuntimeConfig()
    runtime.framework = SimpleNamespace(
        current_state_name="com.bxi.basic_actions/normal",
        snapshot=Mock(return_value={"current": {"name": "idle"}}),
    )
    runtime._wait_for_maintenance_window = Mock(return_value=True)
    runtime._last_control_events = ("com.bxi.suspended_tests/test_mode",)
    runtime.scheduler = SimpleNamespace(period_sec=0.02, status_snapshot=lambda: {"period_ms": 20.0})
    runtime.test_scheduler = SimpleNamespace(period_sec=0.005, status_snapshot=lambda: {"period_ms": 5.0})
    snapshot = runtime.snapshot(include_graph=True)
    runtime.framework.snapshot.assert_called_once_with(include_graph=True)
    assert snapshot["control_timing"] == {"period_ms": 20.0}
    assert snapshot["test_control_timing"] == {"period_ms": 5.0}
    assert snapshot["events"] == ["com.bxi.suspended_tests/test_mode"]


def test_remote_filters_and_state_observers_share_one_callback_lock():
    """A Mod's mutable guard must not run concurrently with its observer."""
    from bxi_example_py_elf3.framework.runtime.controller import RobotControlFramework

    entered = Event()
    release = Event()
    callback_errors = []
    active = [False]

    def event_filter(values, events, state):
        active[0] = True
        entered.set()
        assert release.wait(1.0)
        active[0] = False
        return list(events)

    observer = Mock(side_effect=lambda state: callback_errors.append("concurrent") if active[0] else None)
    framework = RobotControlFramework.__new__(RobotControlFramework)
    framework._remote_event_lock = Lock()
    framework.remote_event_adapter = RemoteEventAdapter({"activate": {"slot": "btn_1", "value": 1}})
    framework.remote_event_adapter.extract_events(_motion(0))
    framework.state_machine = SimpleNamespace(current_state_name="idle")
    framework.mod_runtime = SimpleNamespace(remote_event_filters=(event_filter,), state_observers=(observer,))
    def extract():
        try:
            framework.extract_remote_events(_motion(1))
        except BaseException as exc:
            callback_errors.append(exc)
    worker = Thread(target=extract, daemon=True)
    observer_done = Event()
    def notify():
        try:
            framework._notify_state_observers()
        except BaseException as exc:
            callback_errors.append(exc)
        finally:
            observer_done.set()
    notifier = Thread(target=notify, daemon=True)
    try:
        worker.start()
        assert entered.wait(1.0)
        notifier.start()
        assert not observer_done.wait(0.05)
    finally:
        release.set()
        worker.join(timeout=1.0)
        notifier.join(timeout=1.0)
    assert observer_done.is_set()
    assert not callback_errors
    observer.assert_called_once_with("idle")


def test_timing_statistics_keep_separate_totals_for_each_scheduler_owner():
    from bxi_example_py_elf3.framework.runtime.control_scheduler import ControlCycleResult

    runtime = _timing_runtime()
    def record(scheduler, state, period_ns):
        scheduler._record_cycle(
            ControlCycleResult(state=state), period_ns=period_ns,
            wake_late_ns=0, cycle_ns=1_000_000,
            frame_interval_ns=period_ns, finish_late_ns=0,
            budget_overrun=False, deadline_missed=False, skipped_periods=0,
        )
    record(runtime.scheduler, "com.bxi.basic_actions/normal", 20_000_000)
    runtime._report_control_timing()
    assert "执行1个周期" in runtime._log.call_args.args[1]
    assert "目标周期20.00 ms" in runtime._log.call_args.args[1]
    runtime.framework.current_state_name = "com.bxi.suspended_tests/idle"
    record(runtime.test_scheduler, runtime.framework.current_state_name, 5_000_000)
    runtime._report_control_timing()
    assert "执行1个周期" in runtime._log.call_args.args[1]
    assert "目标周期5.00 ms" in runtime._log.call_args.args[1]
    runtime.framework.current_state_name = "com.bxi.basic_actions/normal"
    record(runtime.scheduler, runtime.framework.current_state_name, 20_000_000)
    runtime._report_control_timing()
    assert "执行1个周期" in runtime._log.call_args.args[1]
    assert runtime._last_reported_timing_totals[False][0] == 2
    assert runtime._last_reported_timing_totals[True][0] == 1
