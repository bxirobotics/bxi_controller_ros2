"""Run with python3 -m unittest discover -s <mod>/tests."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'evaluation'))
from elf3_eval.cycles import link_commands
from elf3_eval.diagnostics import FrameworkDiagnostics


def marker(cycle, stamp):
    return dict(kind='mod_cycle', session='s', cycle=cycle, period_s=.02,
                ros_stamp_ns=stamp, model_id='m', model_hash='h', model_session='s',
                model_seq=cycle, phase='active', state='com.bxi.execution_suite/dance')


def command(stamp, **kwargs):
    return dict(kind='command', source_stamp_ns=stamp, sole_controller_publisher=True, **kwargs)


class CycleTests(unittest.TestCase):
    def test_only_one_real_command_in_consecutive_cycle(self):
        c = command(1001)
        link_commands([marker(5, 1000), c, marker(6, 1000 + 20_000_000)])
        self.assertEqual((c['model_seq'], c['association']), (5, 'mod_cycle_window'))

    def test_unobserved_or_ambiguous_windows(self):
        for markers in ([], [marker(1, 1000)], [marker(1, 1000), marker(3, 1100)],
                        [marker(1, 1000), marker(1, 1001), marker(2, 1100)],
                        [marker(1, 1000), marker(2, 100_000_000)],
                        [marker(1, 1000), marker(2, 999)],
                        [marker(1, 1000), dict(marker(2, 1100), session='another')]):
            with self.subTest(markers=markers):
                c = command(1005)
                link_commands([*markers, c])
                self.assertIsNone(c['model_seq'])

    def test_conflicting_missing_and_invalid_commands(self):
        for cmds in ([command(1001), command(1002)], [dict(command(1001), sole_controller_publisher=False)],
                     [command(1001, time_valid=False)], [command(1100)]):
            link_commands([marker(1, 1000), *cmds, marker(2, 1100)])
            self.assertTrue(all(c['model_seq'] is None for c in cmds))

    def test_timing_counts_attributed_only_within_action(self):
        d = FrameworkDiagnostics()
        stage = 'com.bxi.execution_suite/sonic11'
        def sample(count, state=stage):
            return d.update({'control_timing': dict(state=state, total_deadline_misses=count)},
                            dict(t=1., received_monotonic=1., state=state))
        self.assertFalse(sample(10))
        self.assertEqual(sample(12)[0]['details']['total_deadline_misses'], 2)
        self.assertFalse(sample(13, 'com.bxi.basic_actions/normal'))
