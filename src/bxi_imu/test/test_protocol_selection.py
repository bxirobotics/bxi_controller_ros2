#!/usr/bin/env python3
# Copyright 2026 BXI Robotics

"""Exercise the real IMU node and decoders using a serial pseudoterminal."""

import binascii
import errno
import os
from pathlib import Path
import pty
import signal
import struct
import subprocess
import sys
import tempfile
import time
import unittest


EXECUTABLE = Path(sys.argv.pop(1)).resolve()


def yesense_frame():
    payload = b"\x10\x0c" + struct.pack("<iii", 0, 0, 9800000)
    payload += b"\x20\x0c" + struct.pack("<iii", 0, 0, 0)
    payload += b"\x41\x10" + struct.pack("<iiii", 1000000, 0, 0, 0)
    frame = b"\x59\x53\x01\x00" + bytes([len(payload)]) + payload
    ck1 = ck2 = 0
    for byte in frame[2:]:
        ck1 = (ck1 + byte) & 255
        ck2 = (ck2 + ck1) & 255
    return frame + bytes([ck1, ck2])


def hipnuc_frame():
    payload = struct.pack("<BHbfI16f", 0x91, 0, 25, 0, 1,
                          0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0)
    header = b"\x5a\xa5" + struct.pack("<H", len(payload))
    crc = binascii.crc_hqx(header + payload, 0)
    return header + struct.pack("<H", crc) + payload


class ProtocolSelectionTest(unittest.TestCase):
    def run_node(self, frame, driver="auto", explicit_port=False):
        master, slave = pty.openpty()
        port = os.ttyname(slave)
        os.close(slave)
        os.set_blocking(master, False)
        candidates = (
            f"['hipnuc|{port}|921600|identity|500|2.5', "
            f"'yesense|{port}|921600|-y,x,z|200|2.5']"
        )
        environment = os.environ.copy()
        environment.update(ROS_DOMAIN_ID="219", ROS_LOCALHOST_ONLY="1")
        command = [str(EXECUTABLE), "--ros-args",
                   "-p", f"driver:={driver}",
                   "-p", f"port:={port if explicit_port else 'auto'}",
                   "-p", f"imu_candidates:={candidates}",
                   "-p", "imu_record_enabled:=false"]
        try:
            with tempfile.TemporaryFile(mode="w+") as log:
                process = subprocess.Popen(command, env=environment, stdout=log,
                                           stderr=subprocess.STDOUT)
                try:
                    deadline = time.monotonic() + 8
                    while time.monotonic() < deadline:
                        if frame:
                            try:
                                os.write(master, frame)
                            except OSError as error:
                                if error.errno not in (errno.EIO, errno.EAGAIN):
                                    raise
                        time.sleep(0.01)
                        log.seek(0)
                        output = log.read()
                        if "ACTIVE IMU" in output or process.poll() is not None:
                            break
                    else:
                        self.fail(f"IMU node did not finish protocol selection:\n{output}")
                    started = "ACTIVE IMU" in output
                finally:
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                    log.seek(0)
                    output = log.read()
                return started, process.returncode, output
        finally:
            os.close(master)

    def test_yesense_skips_openable_hipnuc_on_same_port(self):
        started, code, output = self.run_node(yesense_frame())
        self.assertTrue(started, output)
        self.assertEqual(code, 0, output)
        self.assertIn("IMU probe failed: driver=hipnuc", output)
        self.assertIn("selected IMU candidate driver=yesense", output)

    def test_hipnuc_matches_its_protocol(self):
        started, code, output = self.run_node(hipnuc_frame())
        self.assertTrue(started, output)
        self.assertEqual(code, 0, output)
        self.assertIn("selected IMU candidate driver=hipnuc", output)

    def test_invalid_checksums_do_not_select_either_driver(self):
        for frame in (yesense_frame(), hipnuc_frame()):
            with self.subTest(header=frame[:2]):
                damaged = frame[:-1] + bytes([frame[-1] ^ 0xff])
                started, code, output = self.run_node(damaged)
                self.assertFalse(started, output)
                self.assertEqual(code, 1, output)

    def test_open_port_without_data_fails_startup(self):
        started, code, output = self.run_node(b"")
        self.assertFalse(started, output)
        self.assertEqual(code, 1, output)
        self.assertIn("IMU probe failed: driver=hipnuc", output)
        self.assertIn("IMU probe failed: driver=yesense", output)

    def test_explicit_driver_does_not_fall_back_to_other_protocol(self):
        started, code, output = self.run_node(hipnuc_frame(), driver="yesense")
        self.assertFalse(started, output)
        self.assertEqual(code, 1, output)
        self.assertNotIn("opened hipnuc", output)

    def test_explicit_driver_and_port_also_require_valid_protocol(self):
        started, code, output = self.run_node(
            yesense_frame(), driver="hipnuc", explicit_port=True)
        self.assertFalse(started, output)
        self.assertEqual(code, 1, output)

    def test_explicit_matching_protocol_starts(self):
        started, code, output = self.run_node(
            yesense_frame(), driver="yesense", explicit_port=True)
        self.assertTrue(started, output)
        self.assertEqual(code, 0, output)


if __name__ == "__main__":
    unittest.main()
