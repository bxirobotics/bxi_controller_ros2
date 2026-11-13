from unittest.mock import Mock

from bxi_example_py_elf3.framework.mod_api.state import StateBehavior
from bxi_example_py_elf3.framework.runtime.state_machine import RobotStateMachine


def test_transition_logs_have_aligned_borders_and_keep_context():
    source = StateBehavior("com.bxi.basic_actions/zero_torque", 0)
    target = StateBehavior("com.bxi.basic_actions/pd_brake", 1)
    logger = Mock()
    machine = RobotStateMachine(
        Mock(),
        {
            "initial_state": source.name,
            "default_transition": "instant",
            "transition_profiles": {"instant": {"type": "instant"}},
        },
        {source.name: source, target.name: target},
        logger=logger,
    )
    logger.reset_mock()

    assert machine.request_transition(target.name, trigger=target.name)
    assert machine.current_state_name == target.name
    messages = [call.args[0] for call in logger.info.call_args_list]
    assert len(messages) == 2
    assert messages[0].splitlines()[2] == "state transition started"
    assert messages[1].splitlines()[2] == "state transition complete"
    for message in messages:
        lines = message.splitlines()
        assert lines[0] == ""  # Keep the ROS log prefix above the whole block.
        assert len(lines) == 6
        assert lines[1] == lines[-1] == "=" * max(map(len, lines[2:-1]))
        assert lines[3] == f"{source.name} ──────▶ {target.name}"
        assert lines[4] == f"plan=instant, trigger={target.name}"
