"""Reusable named and fixed-order platform joint I/O helpers."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from bxi_example_py_elf3.framework.joints import (
    CompiledJointMap,
    ExactJointTargetAssembler,
    JointLayout,
    JointStateBuffer,
    JointStateView,
    JointTargetView,
)


class NamedJointStateSource:
    """Normalize name-carrying messages into one stable state layout."""

    def __init__(
        self,
        canonical_layout: JointLayout | None = None,
        *,
        dtype: np.dtype | type = np.float64,
    ) -> None:
        self._dtype = np.dtype(dtype)
        self._buffer = (
            JointStateBuffer(canonical_layout, dtype=self._dtype)
            if canonical_layout is not None
            else None
        )
        self._message_map: CompiledJointMap | None = None

    @property
    def ready(self) -> bool:
        return self._buffer is not None and self._message_map is not None

    @property
    def layout(self) -> JointLayout:
        if self._buffer is None:
            raise RuntimeError("named joint source has not received its first message")
        return self._buffer.layout

    @property
    def view(self) -> JointStateView:
        if not self.ready or self._buffer is None:
            raise RuntimeError("named joint source is not ready")
        return self._buffer.view

    def update(
        self,
        names: Sequence[str],
        position: object,
        velocity: object,
        *,
        timestamp_ns: int = 0,
    ) -> JointStateView:
        names = tuple(names)
        mapping = self._message_map
        buffer = self._buffer
        if mapping is None or names != mapping.source.names:
            incoming = JointLayout.create(names, label="incoming joint state")
            if buffer is None:
                buffer = JointStateBuffer(incoming, dtype=self._dtype)
            mapping = CompiledJointMap.compile(incoming, buffer.layout)
        assert buffer is not None
        view = buffer.update(position, velocity, timestamp_ns=timestamp_ns, mapping=mapping)
        self._buffer = buffer
        self._message_map = mapping
        return view


class FixedOrderJointStateSource(JointStateBuffer):
    """State source for transports whose array order is declared once."""


class NamedJointCommandEncoder:
    """Return targets unchanged for a transport that carries joint names."""

    def __init__(self, *, supports_partial: bool = False) -> None:
        self.supports_partial = bool(supports_partial)

    def encode(
        self,
        target: JointTargetView,
        *,
        required_layout: JointLayout | None = None,
    ) -> JointTargetView:
        if required_layout is not None:
            target_names = set(target.layout.names)
            required_names = set(required_layout.names)
            extra = target_names - required_names
            if extra:
                raise ValueError(
                    f"named command contains joints outside required layout: "
                    f"{tuple(sorted(extra))}"
                )
            if not self.supports_partial and target_names != required_names:
                raise ValueError(
                    "named command transport requires a complete target; "
                    "enable supports_partial explicitly for subset commands"
                )
        return target


class FixedOrderJointCommandEncoder:
    """Reorder complete named targets into a hardware fixed layout."""

    def __init__(self, hardware_layout: JointLayout) -> None:
        self.hardware_layout = hardware_layout
        self._assembler = ExactJointTargetAssembler(hardware_layout)

    @property
    def target(self) -> JointTargetView:
        return self._assembler.target

    def encode(self, target: JointTargetView) -> JointTargetView:
        return self._assembler.compose(target)


__all__ = [
    "FixedOrderJointCommandEncoder",
    "FixedOrderJointStateSource",
    "NamedJointCommandEncoder",
    "NamedJointStateSource",
]
