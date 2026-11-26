"""Public robot platform integration API."""

from typing import TYPE_CHECKING

from .config import ControlRuntimeConfig
from .cpu_affinity import CpuAffinityPlan, CpuAffinityRole, CpuAffinitySpec

if TYPE_CHECKING:
    from .api import ControlPlatformAdapter, RobotObservation
    from .joint_io import (
        FixedOrderJointCommandEncoder,
        FixedOrderJointStateSource,
        NamedJointCommandEncoder,
        NamedJointStateSource,
    )
    from .runtime import RobotControlRuntime


def __getattr__(name: str):
    if name in {"ControlPlatformAdapter", "RobotObservation"}:
        from .api import ControlPlatformAdapter, RobotObservation

        return {
            "ControlPlatformAdapter": ControlPlatformAdapter,
            "RobotObservation": RobotObservation,
        }[name]
    if name in {
        "FixedOrderJointCommandEncoder",
        "FixedOrderJointStateSource",
        "NamedJointCommandEncoder",
        "NamedJointStateSource",
    }:
        from .joint_io import (
            FixedOrderJointCommandEncoder,
            FixedOrderJointStateSource,
            NamedJointCommandEncoder,
            NamedJointStateSource,
        )

        return {
            "FixedOrderJointCommandEncoder": FixedOrderJointCommandEncoder,
            "FixedOrderJointStateSource": FixedOrderJointStateSource,
            "NamedJointCommandEncoder": NamedJointCommandEncoder,
            "NamedJointStateSource": NamedJointStateSource,
        }[name]
    if name == "RobotControlRuntime":
        from .runtime import RobotControlRuntime

        return RobotControlRuntime
    raise AttributeError(name)

__all__ = [
    "ControlPlatformAdapter",
    "ControlRuntimeConfig",
    "CpuAffinityPlan",
    "CpuAffinityRole",
    "CpuAffinitySpec",
    "FixedOrderJointCommandEncoder",
    "FixedOrderJointStateSource",
    "NamedJointCommandEncoder",
    "NamedJointStateSource",
    "RobotControlRuntime",
    "RobotObservation",
]
