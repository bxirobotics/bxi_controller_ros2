from types import SimpleNamespace

from bxi_example_py_elf3.imu_freshness import ImuStampObserver


def stamp(ns):
    return SimpleNamespace(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def test_transport_stamp_diagnostics():
    observer = ImuStampObserver(stale_sec=0.1, future_sec=0.1)
    assert observer.observe(stamp(0), 2_000_000_000) == ("missing", None)
    assert observer.observe(stamp(2_000_000_000), 2_001_000_000)[0] == "fresh"
    assert observer.observe(stamp(2_000_000_000), 2_002_000_000)[0] == "repeated"
    assert observer.observe(stamp(1_999_000_000), 2_003_000_000)[0] == "reversed"
    assert observer.observe(stamp(2_010_000_000), 2_200_000_000)[0] == "stale"
    assert observer.observe(stamp(2_500_000_000), 2_300_000_000)[0] == "future"
