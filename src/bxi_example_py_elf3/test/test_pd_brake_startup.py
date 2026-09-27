"""Checks for the initial PD brake gain and pose ramp."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from bxi_example_py_elf3.framework.joints import JointLayout


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
STATE_PATH = PACKAGE_ROOT / "mods/com.bxi.basic_actions/pd_brake_state.py"


def _state_class():
    spec = importlib.util.spec_from_file_location("test_pd_brake_state", STATE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.PdBrakeState


def _context():
    layout = JointLayout(("left", "right"))
    target = SimpleNamespace(
        layout=layout,
        position=np.array([0.0, 0.0], dtype=np.float32),
        kp=np.array([100.0, 80.0], dtype=np.float32),
        kd=np.array([10.0, 8.0], dtype=np.float32),
    )
    frames = []
    ctx = SimpleNamespace(
        loop_count=0,
        robot_layout=layout,
        robot_joints=SimpleNamespace(position=np.array([1.0, -1.0], dtype=np.float32)),
        resolve_motor_frame=lambda source, output: output.update(
            source.qpos, source.kp, source.kd
        ),
        set_motor_target=lambda frame: frames.append(
            (frame.qpos.copy(), frame.kp.copy(), frame.kd.copy())
        ),
    )
    resource = SimpleNamespace(get=lambda: SimpleNamespace(default_target=target))
    return ctx, resource, frames


def test_initial_pd_ramps_pose_and_both_gains_only_once():
    ctx, resource, frames = _context()
    state = _state_class()("pd_brake", 1, resource)
    state._bind_logger(SimpleNamespace(info=lambda message: None))

    state.on_enter(ctx)
    state.on_update(ctx, 0.0)
    np.testing.assert_allclose(frames[-1][0], [1.0, -1.0])
    np.testing.assert_allclose(frames[-1][1], [20.0, 16.0])
    np.testing.assert_allclose(frames[-1][2], [2.0, 1.6])

    state.on_update(ctx, 1.5)
    np.testing.assert_allclose(frames[-1][0], [0.5, -0.5])
    np.testing.assert_allclose(frames[-1][1], [60.0, 48.0])
    np.testing.assert_allclose(frames[-1][2], [6.0, 4.8])

    state.on_update(ctx, 1.5)
    np.testing.assert_allclose(frames[-1][0], [0.0, 0.0])
    np.testing.assert_allclose(frames[-1][1], [100.0, 80.0])
    np.testing.assert_allclose(frames[-1][2], [10.0, 8.0])

    ctx.loop_count = 10
    state.on_enter(ctx)
    state.on_update(ctx, 0.02)
    np.testing.assert_allclose(frames[-1][1], [100.0, 80.0])


def test_later_pd_entry_does_not_start_ramp():
    ctx, resource, frames = _context()
    ctx.loop_count = 10
    state = _state_class()("pd_brake", 1, resource)
    state.on_enter(ctx)
    state.on_update(ctx, 0.02)
    np.testing.assert_allclose(frames[-1][0], [0.0, 0.0])
    np.testing.assert_allclose(frames[-1][1], [100.0, 80.0])


@pytest.mark.parametrize("ramp_sec,gain_from", [(0.0, 0.2), (float("nan"), 0.2), (3.0, 1.2)])
def test_invalid_startup_ramp_parameters_are_rejected(ramp_sec, gain_from):
    _, resource, _ = _context()
    with pytest.raises(ValueError):
        _state_class()(
            "pd_brake", 1, resource,
            startup_ramp_sec=ramp_sec,
            startup_gain_from=gain_from,
        )


def test_startup_configuration_selects_pd_with_ramp():
    config = yaml.safe_load((PACKAGE_ROOT / "config/elf3_state_machine.yaml").read_text())
    mod = yaml.safe_load((PACKAGE_ROOT / "mods/com.bxi.basic_actions/mod.yaml").read_text())
    assert config["initial_state"] == "com.bxi.basic_actions/pd_brake"
    assert mod["states"]["pd_brake"]["params"] == {
        "startup_ramp_sec": 3.0,
        "startup_gain_from": 0.2,
    }
