"""Full state-machine entrypoint for suspended tests."""

from bxi_example_py_elf3.framework.mod_api import ModDefinition, ModLoadContext

from .control_states import (
    SuspendedIdleState,
    SuspendedLimbTestState,
    SuspendedRunningState,
    SuspendedVibrationState,
    TestSession,
)


def create_mod(context: ModLoadContext) -> ModDefinition:
    session = TestSession()
    return ModDefinition(
        state_factories={
            "idle": lambda state: SuspendedIdleState(
                state.name, state.state_id, session,
                prepare_sec=state.float_param("prepare_sec", 3.0),
                prepare_kp_scale=state.float_param("prepare_kp_scale", 1.1),
                center_kd_scale=state.float_param("center_kd_scale", 1.05),
                command_limit_slack_deg=state.float_param("command_limit_slack_deg", 10.0),
            ),
            "running": lambda state: SuspendedRunningState(
                state.name, state.state_id, session
            ),
            "vibration": lambda state: SuspendedVibrationState(
                state.name, state.state_id, session
            ),
            "whole_body_joint_test": lambda state: SuspendedLimbTestState(
                state.name,
                state.state_id,
                session,
                range_speed_deg_s=state.float_param("range_speed_deg_s", 180.0),
                move_sec=state.float_param("move_sec", 1.5),
                hold_sec=state.float_param("hold_sec", 0.2),
                collision_margin_deg=state.float_param("collision_margin_deg", 10.0),
                mechanical_margin_deg=state.float_param("mechanical_margin_deg", 2.0),
                tracking_tolerance_deg=state.float_param("tracking_tolerance_deg", 2.0),
                start_tolerance_deg=state.float_param("start_tolerance_deg", 5.0),
            ),
        }
    )
