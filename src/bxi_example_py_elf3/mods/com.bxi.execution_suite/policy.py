"""Reuse the real model decoder. Never apply SONIC or guessed action scales."""
from pathlib import Path
import time

import numpy as np

from .observation import register_policy, policy_output
from bxi_example_py_elf3.policies import (
    DanceMotionPolicyMjlab, DanceMotionPolicyGravityMjlab,
    DanceMotionPolicyGravityIsaaclab, DanceMotionPolicyGravityIsaaclabV2,
    DanceMotionPolicyGravityIsaaclabV3,
)
from bxi_example_py_elf3.policies.joints import ELF3_POLICY_JOINTS, ELF3_ISAAC_PARAMETERS

VARIANTS = {
    "mjlab": DanceMotionPolicyMjlab,
    "gravity_mjlab": DanceMotionPolicyGravityMjlab,
    "gravity_isaaclab": DanceMotionPolicyGravityIsaaclab,
    "gravity_isaaclab_v2": DanceMotionPolicyGravityIsaaclabV2,
    "gravity_isaaclab_v3": DanceMotionPolicyGravityIsaaclabV3,
}


def inspect_assets(motion_path, model_path, variant="auto"):
    """Validate resources at loading time, never in a control callback."""
    import onnx

    motion_path, model_path = Path(motion_path), Path(model_path)
    if not motion_path.is_file() or not model_path.is_file():
        raise FileNotFoundError(f"dance resources missing: {motion_path}, {model_path}")
    model = onnx.load(str(model_path), load_external_data=False)
    metadata = {item.key: item.value for item in model.metadata_props}
    names = tuple(n.strip() for n in metadata.get("joint_names", "").split(","))
    if len(names) != 29 or len(set(names)) != 29 or set(names) != set(ELF3_POLICY_JOINTS.names):
        raise ValueError("dance model must declare the 29 named ELF3 body joints")
    inputs = {item.name: item for item in model.graph.input}
    if not {"obs", "time_step"} <= inputs.keys():
        raise ValueError("dance policy requires ONNX obs and time_step inputs")
    obs_width = inputs["obs"].type.tensor_type.shape.dim[-1].dim_value
    if variant == "auto":
        if obs_width == 154 and names == ELF3_POLICY_JOINTS.names:
            variant = "mjlab"
        elif obs_width == 157:
            variant = ("gravity_isaaclab" if names == ELF3_ISAAC_PARAMETERS.layout.names
                       else "gravity_mjlab")
        else:
            raise ValueError("cannot infer history/residual decoder; set BXI_DANCE_VARIANT explicitly")
    if variant not in VARIANTS:
        raise ValueError(f"unknown dance variant: {variant}")
    expected = VARIANTS[variant].joint_contract.action.names
    if names != expected:
        raise ValueError("dance variant joint order disagrees with model metadata")
    named_motion = False
    with np.load(motion_path, allow_pickle=False) as motion:
        for key in ("joint_pos", "joint_vel", "body_pos_w", "body_quat_w"):
            if key not in motion:
                raise ValueError(f"dance motion missing {key}")
        q, dq = motion["joint_pos"], motion["joint_vel"]
        if q.ndim != 2 or q.shape[1] != 29 or len(q) < 2 or dq.shape != q.shape:
            raise ValueError("dance motion must contain matching N x 29 position and velocity")
        if not np.isfinite(q).all() or not np.isfinite(dq).all():
            raise ValueError("dance motion contains nonfinite angles/velocities")
        for key, width in (("body_pos_w", 3), ("body_quat_w", 4)):
            a = motion[key]
            if a.ndim != 3 or a.shape[0] != len(q) or a.shape[1] < 1 or a.shape[2] != width or not np.isfinite(a).all():
                raise ValueError(f"invalid dance motion {key}")
        if "joint_names" in motion:
            if tuple(motion["joint_names"].astype(str).tolist()) != names:
                raise ValueError("motion joint names disagree with model")
            named_motion = True
        frames = len(q)
        if "fps" in motion:
            fps = np.asarray(motion["fps"]).reshape(-1)
            if len(fps) != 1 or not np.isfinite(fps[0]) or float(fps[0]) != 50.0:
                raise ValueError("dance policy requires a 50 Hz reference")
    return variant, dict(frames=frames, joint_names=list(names), motion_names_observed=named_motion,
                         motion_order_source="motion_metadata" if named_motion else "existing_policy_contract",
                         reference_rate_hz=50, variant=variant)


class ObservedDancePolicy:
    """Record the exact output of an existing decoder before framework handling."""
    def __init__(self, policy, model_path, motion_path, asset_info):
        self.inner = policy
        self.asset_info = asset_info
        register_policy(self, model_path, adapter="dance",
                policy_class=type(policy).__name__, source_files=[motion_path], details=asset_info)

    @property
    def output(self):
        return self.inner.output

    def reset(self, frame):
        self.inner.reset(frame)

    def step(self, frame, dt, *, advance=True):
        # The exit blend samples the outgoing policy after its final frame.
        # Reuse the last decoded target; a new time_step=N could be out of
        # range in the model graph. This is a hold, not another inference.
        if self.inner.finished():
            return self.inner.output
        cursor = float(self.inner._frame)
        started = time.monotonic()
        output = self.inner.step(frame, dt, advance=advance)
        policy_output(self, started, advance, reference_frame=cursor)
        return output

    def finished(self, trim=0):
        return self.inner.finished(trim)

    def close(self):
        self.inner.close()


def load_dance_policy(motion_path, model_path, *, variant="auto", runtime=None, backend="onnxruntime",
                      start_frame=0, fixed_pos=False):
    variant, info = inspect_assets(motion_path, model_path, variant)
    if isinstance(start_frame, bool) or not isinstance(start_frame, (int, np.integer)) or not 0 <= start_frame < info['frames']:
        raise ValueError("start_frame must be an integer within the motion")
    if not isinstance(fixed_pos, bool):
        raise ValueError("fixed_pos must be boolean")
    policy = VARIANTS[variant](str(motion_path), str(model_path), runtime=runtime, backend=backend,
                              start_frame=int(start_frame), fixed_pos=fixed_pos)
    info.update(start_frame=int(start_frame), fixed_pos=fixed_pos,
                playback_seconds=(info['frames'] - start_frame) / 50.0)
    info['backend'] = policy._backend.backend_name
    info['action_scale'] = policy._parameters.action_scale.tolist()
    info['default_q'] = policy._parameters.default_position.tolist()
    info['decode_rule'] = ('reference_plus_scaled_action' if getattr(policy, '_reference_residual', False)
                           else 'default_q_plus_scaled_action')
    return ObservedDancePolicy(policy, model_path, motion_path, info)
