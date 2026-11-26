"""Translate ELF3 controller data to the communication package's wire types."""

from communication.msg import ActuatorCmds
from communication.srv import RobotReset, SimulationReset
from numpy.typing import NDArray
from numpy import float32

from bxi_example_py_elf3.framework.joints import JointLayout
from bxi_example_py_elf3.framework.mod_api import MotorFrame
from .model import ROBOT_NAME


def _field_bytes(values: NDArray[float32]) -> memoryview | bytes:
    # frombytes needs a contiguous, byte-sized buffer. Pack only strided inputs.
    if values.flags.c_contiguous:
        return memoryview(values).cast("B")
    return values.tobytes()


def motor_command(frame: MotorFrame, stamp) -> ActuatorCmds:
    msg = ActuatorCmds(actuators_name=frame.layout.names)
    msg.header.frame_id = ROBOT_NAME
    msg.header.stamp = stamp
    msg.pos.frombytes(_field_bytes(frame.qpos))
    msg.vel.frombytes(_field_bytes(frame.vel))
    msg.torque.frombytes(_field_bytes(frame.torque))
    msg.kp.frombytes(_field_bytes(frame.kp))
    msg.kd.frombytes(_field_bytes(frame.kd))
    return msg


def robot_reset_request(step: int, release: bool) -> RobotReset.Request:
    req = RobotReset.Request()
    req.header.frame_id = ROBOT_NAME
    req.reset_step = step
    req.release = release
    return req


def simulation_reset_request(layout: JointLayout) -> SimulationReset.Request:
    req = SimulationReset.Request()
    req.header.frame_id = ROBOT_NAME
    req.base_pose.position.z = 1.0
    req.base_pose.orientation.w = 1.0
    req.joint_state.name = layout.names
    zeros = [0.0] * layout.dof_num
    req.joint_state.position = zeros
    req.joint_state.velocity = zeros
    req.joint_state.effort = zeros
    return req
