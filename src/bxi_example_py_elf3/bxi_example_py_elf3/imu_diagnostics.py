"""Bounded, observation-only input callback diagnostics (no ROS dependency).

Header age combines delivery and executor delay; it is NOT device sample age.
Use the independent C++ guard's receive windows to narrow that ambiguity.
"""

from collections import deque
from functools import wraps
from threading import Lock
import time


class ImuReceiveDiagnostics:
    def __init__(self, *, clock=time.monotonic_ns, wall_clock=time.time_ns,
                 gap_sec=0.05, history_size=16):
        self._clock = clock
        self._wall_clock = wall_clock
        self._gap_ns = int(gap_sec * 1e9)
        self._lock = Lock()
        self._active = {}
        self._recent = deque(maxlen=history_size)
        self._counts = {}
        self._accepted = 0
        self._rejected = 0
        self._errors = 0
        self._last_entry = None
        self._last_exit = None
        self._last_accept = None
        self._header_age_ms = None
        self._max_gap_ns = 0
        self._max_duration_ns = 0
        self._max_lock_wait_ns = 0
        self._window_started = self._clock()

    def wrap(self, name, callback):
        @wraps(callback)
        def traced(message):
            self.begin(name, message)
            try:
                return callback(message)
            except BaseException:
                with self._lock:
                    self._errors += 1
                raise
            finally:
                self.end(name)
        return traced

    def begin(self, name, message):
        now = self._clock()
        age = None
        if name == "imu":
            stamp = message.header.stamp
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            if stamp_ns > 0:
                age = (self._wall_clock() - stamp_ns) / 1e6
        with self._lock:
            self._counts[name] = self._counts.get(name, 0) + 1
            self._active[name] = (now, "callback")
            if name == "imu":
                if self._last_entry is not None:
                    gap = now - self._last_entry
                    self._max_gap_ns = max(self._max_gap_ns, gap)
                    if gap >= self._gap_ns:
                        self._recent.append((now, "imu_entry_gap", gap))
                self._last_entry = now
                self._header_age_ms = age

    def phase(self, name, phase):
        with self._lock:
            active = self._active.get(name)
            if active is not None:
                self._active[name] = (active[0], phase)

    def accepted(self, lock_wait_ns):
        now = self._clock()
        with self._lock:
            self._accepted += 1
            self._last_accept = now
            self._max_lock_wait_ns = max(self._max_lock_wait_ns, lock_wait_ns)
            if lock_wait_ns >= self._gap_ns:
                self._recent.append((now, "imu_input_lock_wait", lock_wait_ns))

    def rejected(self):
        with self._lock:
            self._rejected += 1

    def end(self, name):
        now = self._clock()
        with self._lock:
            active = self._active.pop(name, None)
            if active is None:
                return
            duration = now - active[0]
            if name == "imu":
                self._last_exit = now
                self._max_duration_ns = max(self._max_duration_ns, duration)
            if duration >= self._gap_ns:
                self._recent.append((now, name + "_callback_duration", duration))

    def snapshot(self, *, reset_window=False):
        now = self._clock()
        with self._lock:
            def age(value):
                return None if value is None else round(max(0, now - value) / 1e6, 3)

            active = {
                name: {"age_ms": age(start), "phase": phase}
                for name, (start, phase) in self._active.items()
            }
            imu = active.get("imu")
            if imu is not None:
                hint = ("imu_callback_waiting_input_lock"
                        if imu["phase"] == "wait_input_lock"
                        else "imu_callback_in_progress")
            elif any(name in active and active[name]["age_ms"] >= self._gap_ns / 1e6
                     for name in ("joints", "odom", "touch")):
                hint = "sensor_callback_group_busy"
            elif self._last_accept is not None and now - self._last_accept < self._gap_ns:
                hint = "recent_valid_update"
            elif self._last_entry is not None and (
                self._last_accept is None or self._last_entry > self._last_accept
            ):
                hint = "entered_but_no_valid_update"
            else:
                hint = "no_imu_entry_delivery_or_executor_unknown"
            result = {
                "window_ms": age(self._window_started),
                "callback_counts": dict(self._counts),
                "accepted": self._accepted, "rejected": self._rejected,
                "callback_errors": self._errors,
                "last_entry_age_ms": age(self._last_entry),
                "last_exit_age_ms": age(self._last_exit),
                "last_valid_update_age_ms": age(self._last_accept),
                "last_header_age_ms": self._header_age_ms,
                "max_entry_gap_ms": round(self._max_gap_ns / 1e6, 3),
                "max_callback_ms": round(self._max_duration_ns / 1e6, 3),
                "max_input_lock_wait_ms": round(self._max_lock_wait_ns / 1e6, 3),
                "active_callbacks": active, "hint": hint,
                "recent": [{"age_ms": age(t), "event": event,
                            "duration_ms": round(duration / 1e6, 3)}
                           for t, event, duration in self._recent],
            }
            if reset_window:
                self._window_started = now
                self._max_gap_ns = self._max_duration_ns = self._max_lock_wait_ns = 0
            return result
