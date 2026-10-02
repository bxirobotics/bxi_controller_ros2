"""Full state-machine entrypoint for suspended tests."""

import yaml

from bxi_example_py_elf3.framework.mod_api import ModDefinition, ModLoadContext

from .control_states import (
    SuspendedIdleState,
    SuspendedLimbTestState,
    SuspendedRunningState,
    SuspendedVibrationState,
    SequentialLimbTestState,
    SequentialVibrationState,
    SequentialRunningState,
    TestSession,
    SEQUENCE_STATES,
    load_temperature_policy,
)
from .remote_guard import TestRemoteGuard


def create_mod(context: ModLoadContext) -> ModDefinition:
    session = TestSession()
    try:
        session.temperature_policy = load_temperature_policy(
            context.mod_root / "config"
        )
        session.temperature_config_error = None
    except (OSError, ValueError, yaml.YAMLError) as exc:
        session.temperature_config_error = "motor temperature configuration invalid: %s" % exc
    def on_sequence_exit(current_state):
        if session.sequence_active and current_state in SEQUENCE_STATES:
            session.sequence_exit_requested = True

    remote_guard = TestRemoteGuard(on_sequence_exit=on_sequence_exit)

    def observe_state(current_state):
        remote_guard.observe_state(current_state)
        session.observe_sequence_state(current_state)

    def limb_test(state, state_type):
        return state_type(
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
        )

    return ModDefinition(
        remote_event_filter=remote_guard,
        state_observer=observe_state,
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
            "whole_body_joint_test": lambda state: limb_test(
                state, SuspendedLimbTestState,
            ),
            "sequence_joint": lambda state: limb_test(
                state, SequentialLimbTestState,
            ),
            "sequence_vibration": lambda state: SequentialVibrationState(
                state.name, state.state_id, session,
            ),
            "sequence_running": lambda state: SequentialRunningState(
                state.name, state.state_id, session,
            ),
        }
    )
