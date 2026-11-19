"""Transport-age diagnostics for IMU ROS messages (not device sample age)."""

import time


class ImuStampObserver:
    def __init__(self, stale_sec=0.1, future_sec=0.1):
        self.stale_ns = int(stale_sec * 1e9)
        self.future_ns = int(future_sec * 1e9)
        self.last_stamp_ns = None

    def observe(self, stamp, receive_ns=None):
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp_ns <= 0:
            return "missing", None
        if receive_ns is None:
            receive_ns = time.time_ns()
        age_ms = (receive_ns - stamp_ns) / 1e6
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            status = "repeated" if stamp_ns == self.last_stamp_ns else "reversed"
        elif receive_ns - stamp_ns > self.stale_ns:
            status = "stale"
        elif stamp_ns - receive_ns > self.future_ns:
            status = "future"
        else:
            status = "fresh"
        self.last_stamp_ns = stamp_ns
        return status, age_ms
