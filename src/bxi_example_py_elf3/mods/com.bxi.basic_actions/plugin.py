from bxi_example_py_elf3.policies import (
    DanceMotionPolicyGravityIsaaclabV3,
    HumanoidGaitPolicyLiteIsaaclab,
)
from bxi_example_py_elf3.framework.mod_api import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
)

from .imu_protection_state import ImuProtectionState
from .forward_back_state import ForwardBackState
from .initial_pos_state import InitialPosState
from .normal_state import NormalState
from .pd_brake_state import PdBrakeState
from .recover_state import RecoverState
from .zero_torque_state import ZeroTorqueState

NORMAL_POLICY = ResourceKey[HumanoidGaitPolicyLiteIsaaclab]("com.bxi.basic_actions/normal_policy")
RECOVER_POLICY = ResourceKey[DanceMotionPolicyGravityIsaaclabV3]("com.bxi.basic_actions/recover_policy")
RECOVER_FACE_POLICY = ResourceKey[DanceMotionPolicyGravityIsaaclabV3]("com.bxi.basic_actions/recover_face_policy")


def _load_normal_policy(context: ResourceLoadContext) -> HumanoidGaitPolicyLiteIsaaclab:
    return HumanoidGaitPolicyLiteIsaaclab(str(context.asset("assets/amp_terrain.onnx")))


def _load_recover_policy(context: ResourceLoadContext) -> DanceMotionPolicyGravityIsaaclabV3:
    return DanceMotionPolicyGravityIsaaclabV3(
        str(context.asset("assets/getup_back.npz")),
        str(context.asset("assets/getup_back.onnx")),
        start_frame=0,
    )


def _load_recover_face_policy(context: ResourceLoadContext) -> DanceMotionPolicyGravityIsaaclabV3:
    return DanceMotionPolicyGravityIsaaclabV3(
        str(context.asset("assets/getup_face.npz")),
        str(context.asset("assets/getup_face.onnx")),
        start_frame=0,
    )


def create_mod(context: ModLoadContext) -> ModDefinition:
    context.register_resource(NORMAL_POLICY, _load_normal_policy)
    context.register_resource(RECOVER_POLICY, _load_recover_policy, policy="on_demand")
    context.register_resource(RECOVER_FACE_POLICY, _load_recover_face_policy, policy="on_demand")
    normal_policy = context.resource(NORMAL_POLICY)
    recover_policy = context.resource(RECOVER_POLICY)
    recover_face_policy = context.resource(RECOVER_FACE_POLICY)
    return ModDefinition(
        state_factories={
            "normal": lambda state: NormalState(state.name, state.state_id, normal_policy),
            "forward_back": lambda state: ForwardBackState(
                state.name, state.state_id, normal_policy,
                speed=state.float_param("speed", 0.2),
                segment_sec=state.float_param("segment_sec", 2.0),
            ),
            "zero_torque": lambda state: ZeroTorqueState(state.name, state.state_id),
            "imu_protection": lambda state: ImuProtectionState(
                state.name, state.state_id,
                kp=state.float_param("kp", 20.0), kd=state.float_param("kd", 1.0),
            ),
            "pd_brake": lambda state: PdBrakeState(state.name, state.state_id, normal_policy),
            "initial_pos": lambda state: InitialPosState(state.name, state.state_id),
            "recover": lambda state: RecoverState(
                state.name, state.state_id, recover_policy, recover_face_policy
            ),
        }
    )
