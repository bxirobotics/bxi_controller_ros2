from bxi_example_py_elf3.framework.platform.cpu_affinity import (
    bootstrap_process_scheduling,
    CpuAffinityPlan,
)

_CPU_AFFINITY_PLAN = bootstrap_process_scheduling()

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, qos_profile_sensor_data
import communication.msg as bxiMsg
import communication.srv as bxiSrv
import nav_msgs.msg
import sensor_msgs.msg
from threading import Event, Lock
import numpy as np

import time
import json
import math
import os
from collections import deque
from pathlib import Path
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from bxi_example_py_elf3.framework.runtime.state_machine import load_state_machine_config
from bxi_example_py_elf3.framework.joints import (
    JointStateBuffer,
    NamedJointCommandOverride,
)
from bxi_example_py_elf3.framework.mod_api import MotorFrame
from bxi_example_py_elf3.imu_freshness import ImuStampObserver
from bxi_example_py_elf3.imu_diagnostics import ImuReceiveDiagnostics
from bxi_example_py_elf3.framework.platform.status import CachedGraphStatusEncoder
from bxi_example_py_elf3.framework.platform import (
    NamedJointStateSource,
    RobotControlRuntime,
    RobotObservation,
)

from bxi_example_py_elf3.elf3.model import ELF3_COMMAND_DEFAULTS, ELF3_RESET_JOINTS, ROBOT_NAME as robot_name
from bxi_example_py_elf3.elf3.ros_messages import (
    motor_command, robot_reset_request, simulation_reset_request,
)

dof_num = ELF3_RESET_JOINTS.dof_num
joint_name = ELF3_RESET_JOINTS.names


class BxiExample(Node):
    def __init__(self, *, cpu_affinity_plan: CpuAffinityPlan):
        super().__init__("bxi_example_py")

        self._shutting_down = Event()
        # Context shutdown callbacks must never join workers or log.
        self.context.on_shutdown(self._begin_shutdown)
        self._imu_diagnostics = ImuReceiveDiagnostics()

        # 加载运行参数
        self.load_files()

        self._motor_override = NamedJointCommandOverride(
            timeout_sec=self.motor_override_timeout_sec,
            release_blend_sec=self.motor_override_release_blend_sec,
        )
        self._motor_override_last_names: tuple[str, ...] = ()
        self._motor_override_last_error = ""
        self._motor_override_waiting_for_joints_warned = False

        # 订阅发布ros主题
        self.init_pub_sub()

        # 机器人状态变量(robot states)
        # Immutable callback sample, ordered as orientation (xyzw), gyro, acceleration.
        self._imu_sample = ((0.0,) * 4, (0.0,) * 3, (0.0,) * 3)
        self._imu_received = False
        self._imu_last_received_at = None
        self._imu_protection_latched = False
        self._imu_invalid_frame_count = 0
        self._next_imu_invalid_warning = 0.0
        self._imu_stamp_observer = ImuStampObserver()
        self._next_imu_stamp_warning = 0.0
        self._imu_stamp_warning_count = 0
        self._imu_startup_started_at = None
        self._imu_startup_timeout_logged = False
        self._next_imu_startup_warning = 0.0
        self.raw_cmd_vel = np.zeros(3, dtype=np.float32)
        self.pending_remote_events = deque()
        self._joint_source = NamedJointStateSource(dtype=np.float64)
        self._joint_received = False
        self._bad_joint_state_warned = False
        self._joint_snapshot: JointStateBuffer | None = None
        self._quat_xyzw_snapshot = np.zeros(4, dtype=np.float64)
        self._quat_wxyz_snapshot = np.zeros(4, dtype=np.float64)
        self._omega_snapshot = np.zeros(3, dtype=np.float64)
        self._linear_acceleration_snapshot = np.zeros(3, dtype=np.float64)
        self._cmd_snapshot = np.zeros(3, dtype=np.float32)
        self._observation: RobotObservation | None = None

        # 控制循环初始化
        self.step = 0
        self._second_reset_at = 0.0
        self.runtime = RobotControlRuntime(
            self.state_machine_config,
            built_in_mod_root=self.package_share / "mods",
            command_defaults=ELF3_COMMAND_DEFAULTS,
            ros_node=self,
            platform=self,
            cpu_affinity_plan=cpu_affinity_plan,
            fatal_callback=self._on_control_fatal,
        )
        self._state_machine_info_encoder = CachedGraphStatusEncoder()
        self.state_machine_info_timer = None
        self.imu_diagnostics_timer = self.create_timer(
            30.0, self._report_imu_diagnostics,
            callback_group=self.status_callback_group,
        )
        if self.state_machine_info_hz > 0.0:
            self.state_machine_info_timer = self.create_timer(
                1.0 / self.state_machine_info_hz,
                self.publish_state_machine_info,
                callback_group=self.status_callback_group,
            )

    def load_files(self):
        self.declare_parameter("/topic_prefix", "default_value")
        self.topic_prefix = (
            self.get_parameter("/topic_prefix").get_parameter_value().string_value
        )

        self.package_share = Path(get_package_share_directory("bxi_example_py_elf3"))
        self.declare_parameter(
            "/state_machine_config",
            os.path.join(
                self.package_share,
                "config",
                "elf3_state_machine.yaml",
            ),
        )
        state_machine_config_path = self.get_parameter("/state_machine_config").value
        self.state_machine_config = load_state_machine_config(state_machine_config_path)

        self.declare_parameter("/state_machine_info_topic", "")
        self.state_machine_info_topic = (
            self.get_parameter("/state_machine_info_topic")
            .get_parameter_value()
            .string_value
        )

        self.declare_parameter("/state_machine_info_hz", 10.0)
        self.state_machine_info_hz = float(
            self.get_parameter("/state_machine_info_hz").value
        )

        self.imu_required = self.topic_prefix.startswith("hardware/")
        self.declare_parameter("/imu_startup_timeout_sec", 15.0)
        self.imu_startup_timeout_sec = float(
            self.get_parameter("/imu_startup_timeout_sec").value
        )
        self.declare_parameter("/imu_runtime_timeout_sec", 1.0)
        self.imu_runtime_timeout_sec = float(
            self.get_parameter("/imu_runtime_timeout_sec").value
        )
        if self.imu_runtime_timeout_sec <= 0.0:
            raise ValueError("/imu_runtime_timeout_sec must be positive")
        self.declare_parameter("/imu_protection_allow_release", False)
        self.imu_protection_allow_release = bool(
            self.get_parameter("/imu_protection_allow_release").value
        )

        self.motor_override_topic = self.topic_prefix + "actuators_cmds_override"
        self.motor_override_timeout_sec = 0.2
        self.motor_override_release_blend_sec = 0.2
        self.motor_override_allow_in_zero_torque = False

    def _begin_shutdown(self):
        self._shutting_down.set()
        runtime = getattr(self, "runtime", None)
        if runtime is not None:
            runtime.request_stop()

    def _on_control_fatal(self, _message: str):
        self._begin_shutdown()
        rclpy.try_shutdown()

    def destroy_node(self):
        self._begin_shutdown()
        runtime = getattr(self, "runtime", None)
        if runtime is not None:
            try:
                runtime.close()
            except Exception as exc:
                self.get_logger().warning(f"control runtime cleanup failed: {exc}")
        return super().destroy_node()

    # ---------------------------------------------------------------------------- #
    #                                    ROS话题部分                                   #
    # ---------------------------------------------------------------------------- #
    def init_pub_sub(self):
        # 订阅和发布主题
        self.io_callback_group = MutuallyExclusiveCallbackGroup()
        # Keep remote messages ordered, independently of sensor callbacks.
        self.remote_callback_group = MutuallyExclusiveCallbackGroup()
        self.status_callback_group = MutuallyExclusiveCallbackGroup()
        qos = QoSProfile(
            depth=1,
            durability=qos_profile_sensor_data.durability,
            reliability=qos_profile_sensor_data.reliability,
        )

        self.act_pub = self.create_publisher(
            bxiMsg.ActuatorCmds, self.topic_prefix + "actuators_cmds", qos
        )  # CHANGE
        state_machine_info_topic = self.state_machine_info_topic or (
            self.topic_prefix + "state_machine_info"
        )
        self.state_machine_info_pub = self.create_publisher(
            String, state_machine_info_topic, 10
        )

        self.odom_sub = self.create_subscription(
            nav_msgs.msg.Odometry,
            self.topic_prefix + "odom",
            self._imu_diagnostics.wrap("odom", self.odom_callback),
            qos,
            callback_group=self.io_callback_group,
        )
        self.actuator_sub = self.create_subscription(
            bxiMsg.ActuatorStates,
            self.topic_prefix + "actuator_states",
            self._imu_diagnostics.wrap("joints", self.actuator_callback),
            qos,
            callback_group=self.io_callback_group,
        )
        self.imu_sub = self.create_subscription(
            sensor_msgs.msg.Imu,
            self.topic_prefix + "imu_data",
            self._imu_diagnostics.wrap("imu", self.imu_callback),
            qos,
            callback_group=self.io_callback_group,
        )
        self.touch_sub = self.create_subscription(
            bxiMsg.TouchSensor,
            self.topic_prefix + "touch_sensor",
            self._imu_diagnostics.wrap("touch", self.touch_callback),
            qos,
            callback_group=self.io_callback_group,
        )
        self.joy_sub = self.create_subscription(
            bxiMsg.MotionCommands,
            "motion_commands",
            self._imu_diagnostics.wrap("remote", self.joy_callback),
            qos,
            callback_group=self.remote_callback_group,
        )
        self.motor_override_sub = self.create_subscription(
            bxiMsg.ActuatorCmds,
            self.motor_override_topic,
            self._imu_diagnostics.wrap("override", self.motor_override_callback),
            qos,
            callback_group=self.remote_callback_group,
        )

        self.rest_srv = self.create_client(
            bxiSrv.RobotReset, self.topic_prefix + "robot_reset"
        )
        self.sim_rest_srv = self.create_client(
            bxiSrv.SimulationReset, self.topic_prefix + "sim_reset"
        )

        self.lock_in = Lock()
        self.lock_ou = self.lock_in  # Lock()

        self.get_logger().info(
            "motor override input: "
            f"topic={self.motor_override_topic}, "
            f"timeout={self.motor_override_timeout_sec:.3f}s, "
            f"release_blend={self.motor_override_release_blend_sec:.3f}s, "
            "allow_in_zero_torque="
            f"{self.motor_override_allow_in_zero_torque}"
        )

    # --------------------------- Runtime 平台适配接口 --------------------------- #

    def startup_step(self, now: float) -> bool:
        """Perform the ELF3-specific two-stage reset before control starts."""
        if self._shutting_down.is_set() or not self.context.ok():
            return False
        if self.imu_required and not self._imu_received:
            if self._imu_startup_started_at is None:
                self._imu_startup_started_at = now
            elif (
                not self._imu_startup_timeout_logged
                and now - self._imu_startup_started_at >= self.imu_startup_timeout_sec
            ):
                self.get_logger().error(
                    f"IMU startup timeout: no data received on "
                    f"{self.topic_prefix + 'imu_data'} after "
                    f"{self.imu_startup_timeout_sec:.1f}s"
                )
                self._imu_startup_timeout_logged = True
            if now >= self._next_imu_startup_warning:
                self.get_logger().warning(
                    f"waiting for IMU data on {self.topic_prefix + 'imu_data'} "
                    "before control startup"
                )
                self._next_imu_startup_warning = now + 2.0
            return False
        if self.step == 0:
            if self.robot_reset(1, False):
                self.get_logger().info("robot reset step 1 requested")
                self._second_reset_at = now + 1.0
                self.step = 1
            return False
        if self.step == 1:
            if now < self._second_reset_at:
                return False
            if not self.robot_reset(2, True):
                return False
            self.get_logger().info("robot reset step 2 requested")
            self.step = 2
            if self.topic_prefix.find("simulation") != -1:
                self.runtime.request_state(
                    "com.bxi.basic_actions/pd_brake", trigger="AutoPdbreak"
                )
                self.runtime.request_state(
                    "com.bxi.basic_actions/normal", trigger="AutoRelease"
                )
            return False
        with self.lock_in:
            return self._joint_received

    def snapshot_control_inputs(self):
        """Copy the latest ROS inputs into one coherent framework observation."""
        self._check_imu_runtime_safety()
        with self.lock_in:
            latest_joints = self._joint_source.view
            if self._joint_snapshot is None:
                self._joint_snapshot = JointStateBuffer(
                    latest_joints.layout,
                    dtype=np.float64,
                )
                self._observation = RobotObservation(
                    joints=self._joint_snapshot.view,
                    quat_xyzw=self._quat_xyzw_snapshot,
                    quat_wxyz=self._quat_wxyz_snapshot,
                    omega=self._omega_snapshot,
                    raw_cmd_vel=self._cmd_snapshot,
                    linear_acceleration=self._linear_acceleration_snapshot,
                )
            self._joint_snapshot.update(
                latest_joints.position,
                latest_joints.velocity,
                timestamp_ns=latest_joints.timestamp_ns,
            )
            quat, gyro, acceleration = self._imu_sample
            np.copyto(self._cmd_snapshot, self.raw_cmd_vel)
            events = (
                ()
                if self._imu_protection_latched
                else tuple(self.pending_remote_events)
            )
            self.pending_remote_events.clear()
        self._quat_xyzw_snapshot[:] = quat
        self._quat_wxyz_snapshot[:] = quat[3], quat[0], quat[1], quat[2]
        self._omega_snapshot[:] = gyro
        self._linear_acceleration_snapshot[:] = acceleration
        assert self._observation is not None
        return self._observation, events

    def _check_imu_runtime_safety(self):
        if self._shutting_down.is_set() or not self.context.ok():
            return
        if not self.imu_required:
            return
        now = time.monotonic()
        with self.lock_in:
            last_received_at = self._imu_last_received_at
            imu_received = self._imu_received
        if not imu_received or last_received_at is None:
            return
        if not self._imu_protection_latched and (
            now - last_received_at >= self.imu_runtime_timeout_sec
        ):
            if self._shutting_down.is_set() or not self.context.ok():
                return
            self._imu_protection_latched = True
            diagnosis = json.dumps(self._imu_diagnostics.snapshot(), separators=(",", ":"))
            self.get_logger().error(
                f"IMU data timeout: no frame for {now - last_received_at:.3f}s; "
                "entering locked IMU protection state; "
                f"imu_receive_diag={diagnosis}"
            )
        if (
            not self._imu_protection_latched
            or self._shutting_down.is_set()
            or not self.context.ok()
        ):
            return
        protection_state = "com.bxi.basic_actions/imu_protection"
        if self.runtime.current_state_name != protection_state:
            accepted = self.runtime.request_state(
                protection_state,
                trigger="imu_timeout",
                force=True,
            )
            if not accepted:
                self.get_logger().error(
                    "failed to request locked IMU protection state"
                )

    def release_imu_protection(
        self, target_state: str = "com.bxi.basic_actions/zero_torque"
    ) -> bool:
        """Reserved recovery interface; disabled until explicitly enabled."""
        if not self.imu_protection_allow_release:
            self.get_logger().warning(
                "IMU protection release requested but the recovery interface is disabled"
            )
            return False
        with self.lock_in:
            fresh = (
                self._imu_last_received_at is not None
                and time.monotonic() - self._imu_last_received_at
                < self.imu_runtime_timeout_sec
            )
        if not fresh:
            self.get_logger().warning(
                "IMU protection release rejected because IMU data is still stale"
            )
            return False
        self._imu_protection_latched = False
        return self.runtime.request_state(
            target_state,
            trigger="imu_recovery",
            force=True,
        )

    def publish_motor_frame(self, frame: MotorFrame):
        """Convert a framework motor frame into the ELF3 ROS command message."""
        if self._shutting_down.is_set() or not self.context.ok():
            return
        state_name = self.runtime.framework.current_state_name
        override_permitted = self.motor_override_allow_in_zero_torque or (
            state_name != "com.bxi.basic_actions/zero_torque"
        )
        frame = self._motor_override.apply(
            frame,
            now=time.monotonic(),
            permitted=override_permitted,
        )

        msg = motor_command(frame, self.get_clock().now().to_msg())
        if not self._shutting_down.is_set() and self.context.ok():
            self.act_pub.publish(msg)

    def _report_imu_diagnostics(self):
        if (not self.imu_required or self._shutting_down.is_set()
                or not self.context.ok()):
            return
        self.get_logger().info(
            "IMU receive diagnostic (header age combines transport/executor delay, "
            "not device age): " + json.dumps(
                self._imu_diagnostics.snapshot(reset_window=True), separators=(",", ":")
            )
        )

    def publish_state_machine_info(self):
        encoder = self._state_machine_info_encoder
        info = self.runtime.snapshot(include_graph=encoder.needs_graph)
        if info is None:
            return
        now = self.get_clock().now()
        info.update(
            {
                "stamp": {
                    "sec": int(now.nanoseconds // 1000000000),
                    "nanosec": int(now.nanoseconds % 1000000000),
                },
                "step": int(self.step),
            }
        )

        msg = String()
        msg.data = encoder.encode(info)
        self.state_machine_info_pub.publish(msg)

    def robot_reset(self, reset_step, release):
        req = robot_reset_request(reset_step, release)

        while not self.rest_srv.wait_for_service(timeout_sec=0.2):
            if self._shutting_down.is_set() or not rclpy.ok():
                return False
            self.get_logger().info("robot reset service not available; waiting")

        self.rest_srv.call_async(req)
        return True

    def sim_robot_reset(self):
        req = simulation_reset_request(ELF3_RESET_JOINTS)

        while not self.sim_rest_srv.wait_for_service(timeout_sec=1.0):
            print("service not available, waiting again...")

        self.sim_rest_srv.call_async(req)

    def joint_callback(self, msg):
        self._update_joint_state(msg.name, msg.position, msg.velocity)

    def actuator_callback(self, msg):
        self._update_joint_state(msg.name, msg.position, msg.velocity)

    def _update_joint_state(self, names, position, velocity):
        with self.lock_in:
            try:
                latest = self._joint_source.update(
                    names,
                    position,
                    velocity,
                    timestamp_ns=self.get_clock().now().nanoseconds,
                )
            except (TypeError, ValueError) as exc:
                if not self._bad_joint_state_warned:
                    self.get_logger().error(f"invalid named joint state: {exc}")
                    self._bad_joint_state_warned = True
                return

            if not self._joint_received:
                self.get_logger().info(
                    "ELF3 state layout initialized from message names: "
                    f"{latest.layout.dof_num} joints"
                )
            self._joint_received = True
            self._bad_joint_state_warned = False

    def joy_callback(self, msg):
        events = self.runtime.extract_remote_events(msg, sync_only=self.step < 2)
        with self.lock_in:
            self.raw_cmd_vel[:] = (
                msg.vel_des.x,
                msg.vel_des.y,
                msg.yawdot_des,
            )
            self.pending_remote_events.extend(events)

        if self.step < 2:
            return

    def motor_override_callback(self, msg):
        """Validate and atomically replace the final named-joint override."""
        names = tuple(msg.actuators_name)
        if not names:
            if msg.pos or msg.kp or msg.kd or msg.vel or msg.torque:
                self._warn_motor_override(
                    "empty actuators_name disables override only when all command "
                    "arrays are also empty"
                )
                return
            self._motor_override.clear()
            if self._motor_override_last_names:
                self.get_logger().info("motor override release requested")
            self._motor_override_last_names = ()
            self._motor_override_last_error = ""
            return

        with self.lock_in:
            if not self._joint_received:
                if not self._motor_override_waiting_for_joints_warned:
                    self.get_logger().warning(
                        "motor override ignored before the robot joint layout "
                        "is initialized"
                    )
                    self._motor_override_waiting_for_joints_warned = True
                return
            robot_layout = self._joint_source.view.layout

        try:
            submitted_names = self._motor_override.submit(
                robot_layout,
                names,
                msg.pos,
                msg.kp,
                msg.kd,
                vel=msg.vel if msg.vel else None,
                torque=msg.torque if msg.torque else None,
                received_at=time.monotonic(),
            )
        except (TypeError, ValueError) as exc:
            self._warn_motor_override(str(exc))
            return

        self._motor_override_last_error = ""
        self._motor_override_waiting_for_joints_warned = False
        if submitted_names != self._motor_override_last_names:
            self.get_logger().info(
                "motor override active for joints: "
                f"{submitted_names}"
            )
            self._motor_override_last_names = submitted_names

    def _warn_motor_override(self, message: str):
        if message == self._motor_override_last_error:
            return
        self._motor_override_last_error = message
        self.get_logger().warning(f"invalid motor override: {message}")

    def imu_callback(self, msg):
        self._imu_diagnostics.phase("imu", "validate")
        if self.imu_required:
            stamp_status, stamp_age_ms = self._imu_stamp_observer.observe(msg.header.stamp)
            if stamp_status != "fresh":
                self._imu_stamp_warning_count += 1
                now = time.monotonic()
                if now >= self._next_imu_stamp_warning:
                    self._next_imu_stamp_warning = now + 5.0
                    self.get_logger().warning(
                        "IMU ROS stamp diagnostic: status=%s transport_age=%s ms "
                        "count=%d (observe only; device sample age unknown)"
                        % (
                            stamp_status,
                            "n/a" if stamp_age_ms is None else f"{stamp_age_ms:.1f}",
                            self._imu_stamp_warning_count,
                        )
                    )
        quat = msg.orientation
        avel = msg.angular_velocity
        acceleration = msg.linear_acceleration
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        gyro = (avel.x, avel.y, avel.z)
        accel = (acceleration.x, acceleration.y, acceleration.z)
        quaternion_norm = math.sqrt(w * w + x * x + y * y + z * z)
        if not (
            0.9 <= quaternion_norm <= 1.1
            and all(map(math.isfinite, gyro))
            and all(map(math.isfinite, accel))
        ):
            self._imu_diagnostics.rejected()
            now = time.monotonic()
            with self.lock_in:
                self._imu_invalid_frame_count += 1
                invalid_count = self._imu_invalid_frame_count
                should_log = now >= self._next_imu_invalid_warning
                if should_log:
                    self._next_imu_invalid_warning = now + 5.0
            if should_log:
                self.get_logger().warning(
                    "dropping invalid IMU frame: "
                    f"quaternion_norm={quaternion_norm:.6f}, "
                    f"count={invalid_count}"
                )
            return

        self._imu_diagnostics.phase("imu", "wait_input_lock")
        lock_started_ns = time.monotonic_ns()
        with self.lock_in:
            lock_wait_ns = time.monotonic_ns() - lock_started_ns
            self._imu_sample = ((x, y, z, w), gyro, accel)
            first_frame = not self._imu_received
            self._imu_received = True
            self._imu_last_received_at = time.monotonic()
        self._imu_diagnostics.phase("imu", "updated")
        self._imu_diagnostics.accepted(lock_wait_ns)
        if first_frame:
            self.get_logger().info(f"received first IMU frame on {self.topic_prefix}imu_data")

    def touch_callback(self, _msg):
        pass

    def odom_callback(self, _msg):  # 全局里程计（上帝视角，仅限仿真使用）
        pass


def main(args=None):
    time.sleep(5)

    rclpy.init(args=args)
    node = BxiExample(cpu_affinity_plan=_CPU_AFFINITY_PLAN)

    executor = MultiThreadedExecutor(num_threads=3)
    try:
        executor.add_node(node)
        node.runtime.attach_executor(executor)
        if not node._shutting_down.is_set() and node.context.ok():
            node.runtime.start()
            executor.spin()
    finally:
        try:
            node.destroy_node()
        finally:
            executor.shutdown()

    # rclpy's signal handler may already have shut down the context before
    # executor.spin() returns. A second shutdown raises RuntimeError.
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
