"""Cache static graph JSON while preserving self-contained status messages."""

from __future__ import annotations

from collections.abc import Mapping
import json


class CachedGraphStatusEncoder:
    """Encode changing telemetry without walking the static graph every tick.

    The graph is fixed for a framework instance; create a new encoder when that
    instance is replaced. Passing an explicit graph also replaces the cache.
    Every encoded message still contains the complete graph for late subscribers.
    Calls must be serialized by the publisher's callback group.
    """

    def __init__(self) -> None:
        self._graph_json: str | None = None

    @property
    def needs_graph(self) -> bool:
        return self._graph_json is None

    def encode(self, info: Mapping[str, object]) -> str:
        dynamic = dict(info)
        if "graph" in dynamic:
            self._graph_json = json.dumps(
                dynamic.pop("graph"), ensure_ascii=False, separators=(",", ":")
            )
        if self._graph_json is None:
            raise ValueError("first state-machine status must include a graph")
        payload = json.dumps(dynamic, ensure_ascii=False, separators=(",", ":"))
        separator = "," if dynamic else ""
        return payload[:-1] + separator + '"graph":' + self._graph_json + "}"
