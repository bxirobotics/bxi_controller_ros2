"""Passive per-action capture and a separate, low-priority report worker.

No controller state requests or command publishers. Only the recorder's bounded
writer thread touches files; analysis runs in a separate process.
"""
from collections import Counter, deque
import gzip
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

from .contract import clean, dump

STATES = frozenset('com.bxi.execution_suite/' + s for s in ('dance', 'sonic06', 'sonic11'))
TERMINAL = frozenset(('playback_complete', 'failed', 'cancelled', 'interrupted'))


def atomic_json(path, value):
    temporary = path.with_suffix('.tmp')
    dump(temporary, value)
    temporary.replace(path)


def log(message):
    # The launch may have exited while the detached worker finishes a report.
    # A closed stdout pipe must not change a saved result into a failure.
    try:print(message, flush=True)
    except (BrokenPipeError, OSError):pass


class ActionRuns:
    """Receives normalized events on the disk thread, never on control callbacks."""
    def __init__(self, root, *, preroll_s=1., tail_s=1., capacity=2048):
        self.root = Path(root)
        self.preroll_s, self.tail_s = preroll_s, tail_s
        self.buffer = deque(maxlen=capacity)
        self.current = None
        self.last_closed_state = None
        self.statistics = lambda: dict(queue_lost={}, callback_errors={}, writer_errors=[])
        self.baseline = self.statistics()
        self.runs = []

    @staticmethod
    def timestamp(event):
        return float(event.get('received_monotonic', event.get('t', time.monotonic())))

    def _start(self, event, now, observed_start):
        state = event['state']
        path = self.root / ('run_' + time.strftime('%Y%m%d_%H%M%S') + '_' +
                            state.rsplit('/', 1)[-1] + '_' + uuid.uuid4().hex[:8])
        path.mkdir()
        metadata = json.loads((self.root / 'metadata.json').read_text())
        metadata.update(automatic_recording=True, test_state=state,
                        start_event_observed=observed_start, preroll_limit_s=self.preroll_s,
                        preroll_capacity=self.buffer.maxlen, postroll_s=self.tail_s)
        dump(path / 'metadata.json', metadata)
        self.current = dict(path=path, state=state, session=event.get('session'),
                            started=now, ended=None, terminal=None, counts=Counter(),
                            analysis_start=event.get('t', now), analysis_end=None,
                            left_at=None,
                            seen_start=observed_start,
                            start_key=(event.get('session'), event.get('t')) if observed_start else None,
                            stream=gzip.open(path / 'events.jsonl.gz', 'xb', compresslevel=1))
        for previous in self.buffer:
            if now - self.timestamp(previous) <= self.preroll_s:
                self._append(previous)
        self.buffer.clear()
        log(f'ELF3_EVAL recording {state} {path}')

    def _append(self, event):
        row = self.current
        row['stream'].write((json.dumps(clean(event), separators=(',', ':'), allow_nan=False)+'\n').encode())
        row['counts'][event.get('kind', 'unknown')] += 1

    def _interrupt(self, now, reason):
        row = self.current
        event = dict(schema=1, kind='test_event', state=row['state'], session=row['session'],
                     status='interrupted', reason=reason, severity='warning', category='recording',
                     t=now, received_monotonic=now, source='automatic_recorder')
        self._append(event)
        row.update(terminal='interrupted', ended=now, analysis_end=now)

    def write(self, event):
        now = self.timestamp(event)
        self.tick(now)
        kind, state, status = event.get('kind'), event.get('state'), event.get('status')
        specific = state in STATES
        started = specific and kind == 'test_event' and status == 'started'
        terminal = specific and kind == 'test_event' and status in TERMINAL
        fallback = specific and kind in ('mode', 'command_link')
        if not specific and kind in ('mode', 'command_link') and event.get('phase') != 'blend':
            self.last_closed_state = None
        if self.current is None and state == self.last_closed_state and not started:
            fallback = terminal = False
        current = self.current
        duplicate_start = bool(current and started and current['state'] == state and
                               current['start_key'] == (event.get('session'), event.get('t')))
        if current and (started or fallback) and (
                state != current['state'] or
                (started and current['seen_start'] and not duplicate_start)):
            if current['terminal'] is None:self._interrupt(now, 'another_test_started')
            self._finish(now)
        if self.current is None and (started or terminal or fallback):
            self._start(event, now, started)
        if self.current:
            row = self.current
            self._append(event)
            if state == row['state'] and kind in ('mode', 'command_link'):
                row['left_at'] = None
            if started and not duplicate_start:
                row['seen_start'] = True
                row['start_key'] = (event.get('session'), event.get('t'))
                row['session'] = event.get('session')
                row['analysis_start'] = event.get('t', now)
                # A ROS mode event can precede the same action's tap event.
                metadata = json.loads((row['path'] / 'metadata.json').read_text())
                metadata['start_event_observed'] = True
                dump(row['path'] / 'metadata.json', metadata)
            if terminal and state == row['state']:
                # Keep the first terminal status; later accuracy_* events are
                # recorded during postroll without changing playback status.
                if row['terminal'] is None:row.update(terminal=status, ended=now, analysis_end=event.get('t', now))
            elif (kind == 'mode' and state and state not in STATES and
                  event.get('phase') != 'blend' and row['terminal'] is None):
                # ROS and tap delivery are independent. One late normal mode
                # message after the start event must not terminate a new test.
                if row['left_at'] is None:row['left_at'] = now
        else:
            self.buffer.append(event)
            while self.buffer and now - self.timestamp(self.buffer[0]) > self.preroll_s:
                self.buffer.popleft()

    def tick(self, now):
        if (self.current and self.current['terminal'] is None and
                self.current['left_at'] is not None and now-self.current['left_at'] >= .3):
            self._interrupt(now, 'left_test_without_terminal_event')
        if self.current and self.current['ended'] is not None:
            if now - self.current['ended'] >= self.tail_s:self._finish(now)

    def _finish(self, now):
        row = self.current
        if row is None:return
        row['stream'].close()
        stats = self.statistics()
        capture = dict(hardware_commands_sent=0, automatic_recording=True,
                       elapsed_s=max(0., now-row['started']), stop_reason=row['terminal'],
                       accepted=dict(row['counts']), written=sum(row['counts'].values()),
                       writer_errors=stats['writer_errors'])
        for name in ('queue_lost', 'callback_errors'):
            capture[name] = dict(Counter(stats[name]) - Counter(self.baseline[name]))
        # A missing start/terminal must never be mistaken for a completed test.
        capture['start_event_observed'] = row['seen_start']
        capture['analysis_start_monotonic_s'] = row['analysis_start']
        capture['analysis_end_monotonic_s'] = max(row['analysis_start'] + 1e-9, row['analysis_end'])
        atomic_json(row['path'] / 'capture_result.json', capture)
        self.baseline = stats
        self.runs.append(str(row['path']))
        self.last_closed_state = row['state']
        self.current = None

    def close(self):
        if self.current:
            now = time.monotonic()
            if self.current['terminal'] is None:self._interrupt(now, 'recorder_shutdown')
            self._finish(now)


def report_run(path, profile):
    """A report failure leaves all original data and an explicit retry status."""
    from .storage import import_recording
    from .analysis import analyze, Options
    path = Path(path)
    result_path = path / 'report_status.json'
    atomic_json(result_path, dict(status='running', pid=os.getpid()))
    try:
        # A fresh derived directory also makes retry after an interrupted import safe.
        derived = path / ('analysis_' + uuid.uuid4().hex[:8])
        derived.mkdir()
        import_recording(path, derived / 'input.sqlite')
        capture = json.loads((path / 'capture_result.json').read_text())
        metadata = json.loads((path / 'metadata.json').read_text())
        result = analyze(derived / 'input.sqlite', derived / 'report',
                         Options(acceptance_profile=profile, test_state=metadata['test_state'],
                                 start=capture['analysis_start_monotonic_s'],
                                 end=capture['analysis_end_monotonic_s']))
        report = derived / 'report/report.html'
        atomic_json(result_path, dict(status='ready', report=str(report),
                                     acceptance=result.get('acceptance', {}).get('status'),
                                     execution=result.get('acceptance', {}).get('execution')))
        grade = result.get('acceptance', {})
        failed_joints = sorted({name for segment in grade.get('segments', [])
                                for name, row in segment['joints'].items() if row.get('violations')})
        execution = grade.get('execution', {}).get('status')
        log(f"MODEL_TEST {metadata['test_state']} playback={execution} precision={grade.get('status')}"
            f" failed_joints={','.join(failed_joints) or '-'} report={report}")
        return True
    except Exception as exc:
        atomic_json(result_path, dict(status='failed', error=repr(exc)))
        log(f'ELF3_EVAL report_failed {path}: {exc}')
        return False


def worker(root, parent, profile):
    # Inherits the framework's existing _elf.log stdout, not a separate log file.
    # Analysis is sequential and low priority; it never runs on the control thread.
    os.nice(15)
    root = Path(root)
    while True:
        for capture in sorted(root.glob('run_*/capture_result.json')):
            if not (capture.parent / 'report_status.json').exists():report_run(capture.parent, profile)
        try:os.kill(parent, 0); alive = True
        except ProcessLookupError:alive = False
        if (root / 'session_closed.json').exists() or not alive:
            # Scan once more after closure, so the final action cannot be missed.
            for capture in sorted(root.glob('run_*/capture_result.json')):
                if not (capture.parent / 'report_status.json').exists():report_run(capture.parent, profile)
            return
        time.sleep(.2)


def auto_record(output, *, acceptance_profile='elf3-dynamic-screening-v1', **kwargs):
    from .record import record
    root = Path(output).expanduser() / ('session_' + time.strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:8])
    root.mkdir(parents=True)
    sink = ActionRuns(root)
    # Spawn before recording; the worker waits for finalized capture manifests.
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
    process = subprocess.Popen([sys.executable, '-m', 'elf3_eval.auto_record',
        str(root), str(os.getpid()), acceptance_profile], env=env,
        stdin=subprocess.DEVNULL, start_new_session=True)
    try:
        result = record(str(root), _sink=sink, **kwargs)
        result['action_runs'] = sink.runs
        return result
    finally:
        atomic_json(root / 'session_closed.json', dict(action_runs=sink.runs))
        try:process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            log(f'ELF3_EVAL final report processing continues pid={process.pid} {root}')


if __name__ == '__main__':
    worker(Path(sys.argv[1]), int(sys.argv[2]), sys.argv[3])
