"""Keep button release transients from dispatching another model or legacy action.

The same protocol is used by keyboard and gamepad. Inspect raw slots even
when an earlier Mod filter removed an unavailable request. Escape to normal
or zero torque always remains available; this filter never creates events.
"""

import time

_NORMAL = "com.bxi.basic_actions/normal"
_FORWARD_BACK = "com.bxi.basic_actions/forward_back"
_ZERO = "com.bxi.basic_actions/zero_torque"
_EXIT_TEST = "com.bxi.suspended_tests/exit_test_mode"
_ESCAPES = frozenset({
    _NORMAL,
    _ZERO,
    _EXIT_TEST,
})
_SUITE = "com.bxi.execution_suite/"
_MODELS = frozenset(_SUITE + stage for stage in ("dance", "sonic06", "sonic11"))
_MODEL_OUTPUTS = frozenset((31, 33, 34))
_WALKING = frozenset({_NORMAL, _FORWARD_BACK})
_WALKING_TEST_EVENTS = _MODELS | _ESCAPES | {
    _FORWARD_BACK, "com.bxi.basic_actions/pd_brake",
}
_RELEASE_SECONDS = 0.15


class SuiteRemoteGuard:
    def __init__(self, *, isolate=False, clock=time.monotonic):
        self._awaiting_release = False
        self._isolate = isolate
        self._clock = clock
        self._neutral_since = None

    def __call__(self, message, events, current_state):
        neutral = all(
            int(getattr(message, f"btn_{slot}", 0)) == 0
            for slot in range(1, 11)
        )
        if self._awaiting_release:
            now = self._clock()
            if self._neutral_since is not None and now - self._neutral_since >= _RELEASE_SECONDS:
                self._awaiting_release = False
            if neutral and self._awaiting_release and self._neutral_since is None:
                self._neutral_since = now
        if neutral:
            return list(events)
        self._neutral_since = None

        suite_request = int(getattr(message, "btn_10", 0)) in _MODEL_OUTPUTS
        in_model = current_state in _MODELS
        from_walking = current_state in _WALKING
        if self._isolate and from_walking:
            events = [event for event in events if event in _WALKING_TEST_EVENTS]
        if self._awaiting_release or in_model or (suite_request and from_walking):
            escapes = [event for event in events if event in _ESCAPES]
            if escapes:
                self._awaiting_release = True
                # Zero torque preempts any pending or active model immediately.
                for stop in (_ZERO, _EXIT_TEST):
                    if stop in escapes:
                        return [stop]
                return escapes
            if self._awaiting_release or in_model:
                return []
            self._awaiting_release = True
            return [event for event in events if event in _MODELS]
        return list(events)
