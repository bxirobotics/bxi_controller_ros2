#!/usr/bin/env python3
import os
import runpy
import subprocess
import tempfile
import unittest
from pathlib import Path

import yaml

SCRIPT = Path(__file__).with_name("generate_robot_config.sh")


class RobotConfigTests(unittest.TestCase):
    def generate(self, output, *args, bin_dir=None):
        env = dict(os.environ, BXI_ROBOT_CONFIG_FILE=str(output))
        if bin_dir is not None:
            env["PATH"] = f"{bin_dir}:/usr/bin:/bin"
        result = subprocess.run(["bash", str(SCRIPT), *args], env=env, capture_output=True, text=True)
        self.assertIn("[进入脚本] generate_robot_config.sh\n", result.stderr)
        self.assertIn(f"[离开脚本] generate_robot_config.sh (退出码={result.returncode})\n", result.stderr)
        return result

    def test_default_zero_and_one(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            output = folder / "robot_config.yaml"
            (folder / "hostname").write_text("#!/bin/sh\necho elf3-117\n")
            (folder / "hostname").chmod(0o755)
            for args, head in [((), False), (("0",), False), (("1",), True)]:
                result = self.generate(output, *args, bin_dir=folder)
                self.assertEqual(result.returncode, 0, result.stderr)
                config = yaml.safe_load(output.read_text())
                self.assertEqual(config["modules"]["head"], {"label": "Head", "enabled": head})
                self.assertEqual(config["modules"]["depth_cameras"]["items"][0]["model"], "realsense_d435i")
                self.assertEqual(config["modules"]["depth_cameras"]["items"][0]["id"], "realsense_d435i")
                self.assertEqual(config["modules"]["lidars"]["quantity"], 0)
                self.assertEqual(output.stat().st_mode & 0o777, 0o644)
                reader_path = SCRIPT.parent.parent / "src/bxi_example_py_elf3/bxi_example_py_elf3/framework/mod_api/hardware_launch.py"
                if reader_path.is_file():
                    reader = runpy.run_path(str(reader_path))
                    self.assertEqual(reader["_resolve_enable_head"]("auto", config), head)

    def test_invalid_argument_preserves_old_file(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            output = folder / "robot_config.yaml"
            (folder / "hostname").write_text("#!/bin/sh\necho elf3-117\n")
            (folder / "hostname").chmod(0o755)
            output.write_text("original config\n")
            for args in [("2",), ("abc",), ("-1",), ("",), ("1", "0")]:
                self.assertNotEqual(self.generate(output, *args, bin_dir=folder).returncode, 0)
                self.assertEqual(output.read_text(), "original config\n")

    def test_hostname_and_domain(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            output = folder / "robot_config.yaml"
            command = folder / "hostname"
            for host, number in [("elf3-117", 117), ("elf3-117.local", 117), ("elf3-008", 8), ("elf3-202", 202)]:
                command.write_text(f"#!/bin/sh\necho {host}\n")
                command.chmod(0o755)
                result = self.generate(output, bin_dir=folder)
                self.assertEqual(result.returncode, 0, result.stderr)
                config = yaml.safe_load(output.read_text())
                self.assertEqual(config["ros_domain_id"], number + 30)
                self.assertEqual(config["hostname"], host)
            original = output.read_bytes()
            for host in ("elf3-203", "elf3-18446744073709551616", "no-number"):
                command.write_text(f"#!/bin/sh\necho {host}\n")
                self.assertNotEqual(self.generate(output, bin_dir=folder).returncode, 0)
                self.assertEqual(output.read_bytes(), original)

    def test_failed_replace_preserves_existing_config(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            output = folder / "robot_config.yaml"
            output.write_text("original config\n")
            (folder / "mv").write_text("#!/bin/sh\nexit 1\n")
            (folder / "mv").chmod(0o755)
            (folder / "hostname").write_text("#!/bin/sh\necho elf3-117\n")
            (folder / "hostname").chmod(0o755)
            result = self.generate(output, bin_dir=folder)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(output.read_text(), "original config\n")
            self.assertEqual(list(folder.glob(".robot-config.*")), [])


if __name__ == "__main__":
    unittest.main()
