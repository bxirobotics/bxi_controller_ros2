"""Offline checks for the imported lie-down state and its resources."""

import importlib
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest
import yaml


MOD_ROOT = Path(__file__).resolve().parents[1] / "mods/com.bxi.basic_actions"


def test_lie_down_is_normal_only_and_has_emergency_exit():
    config = yaml.safe_load((MOD_ROOT / "mod.yaml").read_text())
    assert config["events"]["lie_down"] == {"slot": "btn_10", "value": 8}
    assert [route["from"] for route in config["routes"] if route["to"] == "lie_down"] == ["normal"]
    assert any(
        route["from"] == "lie_down"
        and route["event"] == "zero_torque"
        and route["to"] == "zero_torque"
        for route in config["routes"]
    )


def test_lie_down_assets_have_expected_motion_frames():
    assets = MOD_ROOT / "assets"
    assert (assets / "lie_down.onnx").is_file()
    with np.load(assets / "lie_down.npz") as motion:
        assert motion["joint_pos"].shape == (569, 29)


def test_lie_down_finishes_in_pd_brake(monkeypatch):
    package = ModuleType("_test_lie_down_basic_actions")
    package.__path__ = [str(MOD_ROOT)]
    monkeypatch.setitem(sys.modules, package.__name__, package)
    module = importlib.import_module(f"{package.__name__}.lie_down_state")
    try:
        policy = SimpleNamespace(finished=lambda trim: trim == 225)
        state = module.LieDownState("lie_down", 1, SimpleNamespace(get=lambda: policy))
        state.sample_running_frame = lambda ctx, dt, advance: "frame"
        requests = []
        frames = []
        ctx = SimpleNamespace(
            set_motor_target=frames.append,
            request_state=lambda target, **kwargs: requests.append((target, kwargs)),
        )
        state.on_update(ctx, 0.02)
        assert frames == ["frame"]
        assert requests[0][0] == "com.bxi.basic_actions/pd_brake"
        assert requests[0][1]["trigger"] == "lie_down_finished"
    finally:
        monkeypatch.delitem(sys.modules, module.__name__, raising=False)
