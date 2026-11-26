"""Allocation-free temporal buffers used by inference input builders."""

from __future__ import annotations

from typing import Sequence

import numpy as np
from numpy.typing import NDArray


class HistoryBuffer:
    """Fixed-size, oldest-to-newest history backed by a ring buffer.

    ``append`` never allocates.  ``write_into`` linearizes the ring into a
    caller-owned, writable buffer in chronological order.
    """

    def __init__(
        self,
        length: int,
        item_shape: int | Sequence[int],
        *,
        dtype: np.dtype | type = np.float32,
    ) -> None:
        if int(length) <= 0:
            raise ValueError("history length must be greater than zero")
        if isinstance(item_shape, int):
            shape = (int(item_shape),)
        else:
            shape = tuple(int(dim) for dim in item_shape)
        if not shape or any(dim <= 0 for dim in shape):
            raise ValueError("history item shape must contain positive dimensions")

        self.length = int(length)
        self.item_shape = shape
        self.dtype = np.dtype(dtype)
        self._data = np.empty((self.length, *shape), dtype=self.dtype)
        self._next_index = 0
        self._initialized = False
        self.clear()

    @property
    def storage(self) -> NDArray[np.generic]:
        """Return physical ring storage; its row order is not chronological."""

        return self._data

    @property
    def size(self) -> int:
        return int(self._data.size)

    @property
    def initialized(self) -> bool:
        return self._initialized

    def clear(self) -> None:
        self._data.fill(0)
        self._next_index = 0
        self._initialized = False

    def fill(self, value: object) -> None:
        array = np.asarray(value)
        self._validate_item(array)
        self._data[...] = array
        self._next_index = 0
        self._initialized = True

    def append(self, value: object) -> None:
        array = np.asarray(value)
        self._validate_item(array)
        np.copyto(self._data[self._next_index], array, casting="same_kind")
        self._next_index += 1
        if self._next_index == self.length:
            self._next_index = 0
        self._initialized = True

    def write_into(self, output: object) -> None:
        self._write(output, None)

    def preview_append_into(self, value: object, output: object) -> None:
        """Linearize the history as if ``value`` were appended, without mutation."""
        array = np.asarray(value)
        self._validate_item(array)
        self._write(output, array)

    def _write(self, output: object, pending: NDArray[np.generic] | None) -> None:
        target = np.asarray(output)
        # An array provider can return its own storage; a plain sequence converts to a copy.
        if target is not output and target.flags.owndata and not hasattr(output, "__array__"):
            raise TypeError("history output must expose writable array storage")
        if target.size != self._data.size:
            raise ValueError(
                f"history output has {target.size} elements, expected {self._data.size}"
            )
        if target.dtype != self.dtype:
            raise TypeError(
                f"history output dtype is {target.dtype}, expected {self.dtype}"
            )
        # reshape can silently copy a transposed view. Write to its flat iterator instead.
        flat = target.reshape(-1) if target.flags.c_contiguous else target.flat
        split = self._next_index
        start = split if pending is None else split + 1
        tail = self._data[start:].reshape(-1)
        flat[:tail.size] = tail
        if split:
            head = self._data[:split].reshape(-1)
            flat[tail.size:tail.size + head.size] = head
        if pending is not None:
            flat[-pending.size:] = pending.reshape(-1)

    def _validate_item(self, value: NDArray[np.generic]) -> None:
        if value.shape != self.item_shape:
            raise ValueError(
                f"history item shape is {value.shape}, expected {self.item_shape}"
            )
        if value.dtype != self.dtype:
            raise TypeError(
                f"history item dtype is {value.dtype}, expected {self.dtype}"
            )


__all__ = ["HistoryBuffer"]
