"""Remote-button interlock owned by the suspended-test Mod."""

from collections.abc import Sequence


_PREFIX = "com.bxi.suspended_tests/"
_EXIT = _PREFIX + "exit_test_mode"
_TEST_EVENTS = frozenset({
    _PREFIX + "running",
    _PREFIX + "vibration",
    _PREFIX + "whole_body_joint_test",
    _PREFIX + "sequence",
})


class TestRemoteGuard:
    def __init__(self, on_sequence_exit=None) -> None:
        self._was_in_test = False
        self._awaiting_release = False
        self._on_sequence_exit = on_sequence_exit

    def observe_state(self, current_state: str) -> None:
        in_test = current_state.startswith(_PREFIX)
        if self._was_in_test and not in_test:
            self._awaiting_release = True
        self._was_in_test = in_test

    def __call__(
        self, message: object, events: Sequence[str], current_state: str
    ) -> list[str]:
        self.observe_state(current_state)
        in_test = current_state.startswith(_PREFIX)

        exit_pressed = in_test and _EXIT in events
        if exit_pressed:
            self._awaiting_release = True
            if self._on_sequence_exit is not None:
                self._on_sequence_exit(current_state)

        if self._awaiting_release:
            neutral = all(
                int(getattr(message, f"btn_{slot}", 0)) == 0
                for slot in range(1, 11)
            )
            if neutral:
                self._awaiting_release = False
            else:
                return [_EXIT] if exit_pressed else []

        if in_test:
            return [event for event in events if event in _TEST_EVENTS or event == _EXIT]
        return list(events)
