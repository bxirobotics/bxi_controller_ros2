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


def test_busy_wait_configuration_has_been_removed():
    assert not hasattr(ControlRuntimeConfig.from_mapping({}), "spin_wait_us")
    with pytest.raises(ValueError, match="unknown fields"):
        ControlRuntimeConfig.from_mapping({"spin_wait_us": 200})
    assert not hasattr(_scheduler(), "spin_wait_ns")


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


@pytest.mark.parametrize("threads", [None, "1"])
def test_onnx_workers_sleep_and_io_binding_still_works(tmp_path, monkeypatch, threads):
    if threads is None:
        monkeypatch.delenv("BXI_ONNX_INTRA_OP_THREADS", raising=False)
    else:
        monkeypatch.setenv("BXI_ONNX_INTRA_OP_THREADS", threads)
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
        assert options.intra_op_num_threads == int(threads or 4)
        obs = np.arange(4, dtype=np.float32).reshape(1, 4)
        for _ in range(3):
            result = backend.run({"obs": obs})
            np.testing.assert_array_equal(result["actions"], obs)
            obs += 1
        assert backend._io_binding is not None
    finally:
        backend.close()


@pytest.mark.parametrize("value", ["0", "-1", "1.5"])
def test_onnx_threads_reject_invalid_values(monkeypatch, value):
    from bxi_example_py_elf3.framework.inference.backends.onnxruntime import onnx_intra_op_threads

    monkeypatch.setenv("BXI_ONNX_INTRA_OP_THREADS", value)
    with pytest.raises(ValueError, match="BXI_ONNX_INTRA_OP_THREADS"):
        onnx_intra_op_threads()


@pytest.mark.parametrize("size", [29, 31])
def test_simulation_reset_request_matches_robot_layout(size):
    from bxi_example_py_elf3.elf3.model import ELF3_BODY_JOINTS, ELF3_RESET_JOINTS
    from bxi_example_py_elf3.elf3.ros_messages import simulation_reset_request

    layout = ELF3_BODY_JOINTS if size == 29 else ELF3_RESET_JOINTS
    request = simulation_reset_request(layout)
    assert request.header.frame_id == "elf3"
    assert request.base_pose.position.z == request.base_pose.orientation.w == 1.0
    assert tuple(request.joint_state.name) == layout.names
    assert len(request.joint_state.name) == size
    if size == 31:
        assert layout.names == ELF3_BODY_JOINTS.names + ("head_z_joint", "head_y_joint")
    for values in (request.joint_state.position, request.joint_state.velocity,
                   request.joint_state.effort):
        assert list(values) == [0.0] * size


@pytest.mark.parametrize("size", [2, 31])
@pytest.mark.parametrize("contiguous", [True, False], ids=["contiguous", "strided"])
def test_motor_frame_wire_format(demo_class, size, contiguous):
    from communication.msg import ActuatorCmds
    from rclpy.serialization import deserialize_message, serialize_message
    from bxi_example_py_elf3.elf3.model import ELF3_RESET_JOINTS
    from bxi_example_py_elf3.framework.joints import JointLayout
    from bxi_example_py_elf3.framework.mod_api.frame import MotorFrame

    layout = JointLayout.create(["b", "a"]) if size == 2 else ELF3_RESET_JOINTS
    data = np.arange(size * 5, dtype=np.float32)
    fields = data.reshape(5, size) if contiguous else data.reshape(size, 5).T
    frame = MotorFrame(layout, *fields)
    node = Mock()
    node._shutting_down.is_set.return_value = False
    node.context.ok.return_value = True
    node._motor_override.apply.return_value = frame
    expected = ActuatorCmds()
    expected.header.frame_id = "elf3"
    expected.header.stamp.sec = 123
    expected.header.stamp.nanosec = 456
    node.get_clock.return_value.now.return_value.to_msg.return_value = expected.header.stamp
    expected.actuators_name = list(layout.names)
    for name, values in (("pos", frame.qpos), ("kp", frame.kp), ("kd", frame.kd),
                         ("vel", frame.vel), ("torque", frame.torque)):
        setattr(expected, name, values.tolist())
    demo_class.publish_motor_frame(node, frame)
    data.fill(-1)
    actual, = node.act_pub.publish.call_args.args
    assert deserialize_message(serialize_message(actual), ActuatorCmds) == expected


class WritableArrayView:
    def __init__(self, array):
        self.array = array

    def __array__(self, dtype=None):
        return np.asarray(self.array, dtype=dtype)


@pytest.mark.parametrize("wrap", [lambda a: a.T, lambda a: a.view(np.matrix), WritableArrayView],
                         ids=["transposed", "matrix", "array_protocol"])
def test_history_updates_output_across_wrap(wrap):
    from bxi_example_py_elf3.framework.inference.history import HistoryBuffer

    history = HistoryBuffer(3, 2)
    samples = np.arange(12, dtype=np.float32).reshape(6, 2)
    storage = np.empty((2, 3), dtype=np.float32)
    output = wrap(storage)
    view = np.asarray(output)
    assert np.shares_memory(view, storage)
    for index, item in enumerate(samples):
        before = history.storage.copy()
        history.preview_append_into(item, output)
        expected = np.zeros((3, 2), dtype=np.float32)
        recent = samples[max(0, index - 2):index + 1]
        expected[-len(recent):] = recent
        np.testing.assert_array_equal(view.ravel(), expected.ravel())
        np.testing.assert_array_equal(history.storage, before)
        history.append(item)
        storage.fill(-1)
        history.write_into(output)
        np.testing.assert_array_equal(view.ravel(), expected.ravel())


def test_history_rejects_output_that_would_only_write_to_a_copy():
    from bxi_example_py_elf3.framework.inference.history import HistoryBuffer

    with pytest.raises(TypeError):
        HistoryBuffer(2, 1).write_into([np.float32(0), np.float32(0)])


def test_model_inspection_preserves_direct_graph_inputs(tmp_path):
    onnx = pytest.importorskip("onnx")
    from bxi_example_py_elf3.framework.inference.model import ModelSpec

    helper = onnx.helper
    def tensor(name):
        return helper.make_tensor_value_info(name, onnx.TensorProto.FLOAT, [1])

    graph = helper.make_graph(
        [helper.make_node("Identity", ["obs"], ["actions"])], "passthrough",
        [tensor("obs"), tensor("reference"), tensor("unused")],
        [tensor("actions"), tensor("reference")],
    )
    path = tmp_path / "passthrough.onnx"
    onnx.save(helper.make_model(graph), str(path))
    with patch.object(onnx, "load_model", wraps=onnx.load_model) as load:
        spec = ModelSpec.portable_onnx(path, input_names=(), output_names=())
    assert spec.artifacts[0].runtime_input_names == ("obs", "reference")
    load.assert_called_once()


def test_resource_shutdown_drains_accepted_loads(tmp_path):
    from bxi_example_py_elf3.framework.mod_api.resource import ResourceKey
    from bxi_example_py_elf3.framework.runtime.resource_manager import ResourceManager

    manager = ResourceManager()
    started, release = Event(), Event()
    first, second = ResourceKey("test/first"), ResourceKey("test/second")
    closed = []

    def slow_factory(_):
        started.set()
        assert release.wait(2)
        return SimpleNamespace(close=lambda: closed.append(first))

    manager.register(first, owner="test", root=tmp_path, factory=slow_factory, policy="startup")
    manager.register(second, owner="test", root=tmp_path, policy="on_demand",
                     factory=lambda _: SimpleNamespace(close=lambda: closed.append(second)))
    startup = Thread(target=manager.load_startup)
    startup.start()
    try:
        assert started.wait(2)
        with pytest.raises(RuntimeError, match="after startup"):
            manager.register(ResourceKey("test/late"), owner="test", root=tmp_path,
                             factory=slow_factory, policy="startup")
        manager.request(second)
        worker = Thread(target=manager.close)
        worker.start()
    finally:
        release.set()
        startup.join(2)
        manager.close()
    worker.join(2)
    assert not startup.is_alive() and not worker.is_alive()
    assert closed == [second, first]
    with pytest.raises(RuntimeError, match="closed"):
        manager.request(first)


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
    runtime._last_reported_total_cycles = 0
    runtime._last_reported_total_budget_overruns = 0
    runtime._last_reported_total_deadline_misses = 0
    runtime._last_reported_total_skipped_periods = 0
    runtime.scheduler = _scheduler()
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
