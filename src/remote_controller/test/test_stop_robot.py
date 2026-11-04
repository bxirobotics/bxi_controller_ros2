"""Exercise ordered shutdown with fake pgrep/pkill. Never signal real processes."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'stop_robot.sh'


class OrderedStopTest(unittest.TestCase):
    def run_helper(self, mode):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            fake = '''#!%s
import json, os, sys
from pathlib import Path
root = Path(os.environ['FAKE_ROOT'])
name = Path(sys.argv[0]).name
with (root/'calls').open('a') as f:
    f.write(json.dumps([name] + sys.argv[1:]) + '\\n')
if name == 'pgrep':
    control = 'xi_example_py_elf3_demo' in sys.argv[-1]
    if control and os.environ['FAKE_MODE'] == 'stuck':
        sys.exit(0)
    if control and os.environ['FAKE_MODE'] == 'inspect_error':
        sys.exit(2)
    if control and os.environ['FAKE_MODE'] == 'slow':
        count_file = root/'count'
        n = int(count_file.read_text()) if count_file.exists() else 0
        count_file.write_text(str(n+1))
        sys.exit(0 if n < 2 else 1)
    sys.exit(1)
sys.exit(0)
''' % sys.executable
            for name in ['pgrep', 'pkill', 'sleep']:
                p = root/name
                p.write_text(fake)
                p.chmod(0o755)
            env = dict(os.environ, PATH=str(root)+os.pathsep+os.environ['PATH'],
                       FAKE_ROOT=str(root), FAKE_MODE=mode)
            result = subprocess.run(['bash', str(SCRIPT)], env=env,
                                    capture_output=True, text=True, timeout=15)
            calls = [json.loads(x) for x in (root/'calls').read_text().splitlines()]
            return result, calls

    def test_control_exits_before_imu_is_signalled(self):
        for mode in ['clean', 'slow']:
            with self.subTest(mode=mode):
                result, calls = self.run_helper(mode)
                self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
                kills = [x for x in calls if x[0] == 'pkill']
                self.assertIn('xi_example_py_elf3_demo', kills[0][-1])
                self.assertNotIn('imu_node', kills[0][-1])
                self.assertIn('imu_node', kills[1][-1])
                imu_index = calls.index(kills[1])
                self.assertTrue(any(x[0] == 'pgrep' for x in calls[:imu_index]))

    def test_stuck_control_keeps_imu_alive(self):
        result, calls = self.run_helper('stuck')
        self.assertEqual(result.returncode, 1)
        self.assertIn('keeping IMU alive', result.stdout)
        self.assertFalse(any(x[0] == 'pkill' and 'imu_node' in x[-1] for x in calls))
        control_signals = [x[1] for x in calls if x[0] == 'pkill'
                           and 'xi_example_py_elf3_demo' in x[-1]]
        self.assertEqual(control_signals, ['-SIGINT', '-SIGTERM', '-SIGKILL'])

    def test_inspection_error_is_not_misread_as_success(self):
        result, calls = self.run_helper('inspect_error')
        self.assertEqual(result.returncode, 1)
        self.assertFalse(any(x[0] == 'pkill' and 'imu_node' in x[-1] for x in calls))


if __name__ == '__main__':
    unittest.main()
