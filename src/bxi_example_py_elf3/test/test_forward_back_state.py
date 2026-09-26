"""Offline checks for the normal-only forward/backward mode."""

import importlib
from pathlib import Path
import sys
import time
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest
import yaml


MOD_ROOT = Path(__file__).resolve().parents[1] / "mods/com.bxi.basic_actions"


class _Logger:
    def info(self, message):
        pass


@pytest.fixture
def states(monkeypatch):
    package = ModuleType("_test_basic_actions")
    package.__path__ = [str(MOD_ROOT)]
    monkeypatch.setitem(sys.modules, package.__name__, package)
    normal_module = importlib.import_module(f"{package.__name__}.normal_state")
    forward_back_module = importlib.import_module(
        f"{package.__name__}.forward_back_state"
    )
    yield normal_module.NormalState, forward_back_module.ForwardBackState
    monkeypatch.delitem(sys.modules, normal_module.__name__, raising=False)
    monkeypatch.delitem(sys.modules, forward_back_module.__name__, raising=False)


def test_forward_back_ignores_joystick_and_alternates_every_two_seconds(states):
    _, ForwardBackState = states
    policy = object()
    state = ForwardBackState("forward_back", 1, policy)
    state._bind_logger(_Logger())
    ctx = SimpleNamespace(
        current_raw_cmd_vel=np.array([0.9, 0.4, 0.5], dtype=np.float32),
        current_cmd_vel=np.zeros(3, dtype=np.float32),
    )
    state.on_bind(ctx)
    state.on_enter(ctx)

    for elapsed, expected_x in ((0.0, 0.2), (1.9, 0.2), (2.1, -0.2), (4.1, 0.2)):
        state._entered_at = time.monotonic() - elapsed
        np.testing.assert_allclose(state.get_cmd_vel(ctx), [expected_x, 0.0, 0.0])
        np.testing.assert_allclose(ctx.current_cmd_vel, [expected_x, 0.0, 0.0])

    assert not state.on_action(ctx, "toggle_navigation_control")
    np.testing.assert_allclose(state.get_cmd_vel(ctx)[1:], [0.0, 0.0])


def test_shared_normal_policy_is_not_reset_on_mode_switch(states):
    NormalState, ForwardBackState = states
    policy = object()
    normal = NormalState("normal", 1, policy)
    forward_back = ForwardBackState("forward_back", 2, policy)
    ctx = SimpleNamespace(
        current_raw_cmd_vel=np.zeros(3, dtype=np.float32),
        current_cmd_vel=np.zeros(3, dtype=np.float32),
    )
    forward_back.on_prepare(ctx, normal)
    normal.on_prepare(ctx, forward_back)


def test_mode_is_reachable_only_from_normal_and_buttons_keep_safety_routes():
    config = yaml.safe_load((MOD_ROOT / "mod.yaml").read_text())
    entry_routes = [
        route for route in config["routes"] if route["to"] == "forward_back"
    ]
    assert entry_routes == [
        {"from": "normal", "event": "forward_back", "to": "forward_back", "transition": "soft_switch"}
    ]
    exits = {
        (route["event"], route["to"])
        for route in config["routes"]
        if route["from"] == "forward_back"
    }
    assert exits == {
        ("forward_back", "normal"),
        ("zero_torque", "zero_torque"),
        ("pd_brake", "pd_brake"),
        ("initial_pos", "initial_pos"),
    }
