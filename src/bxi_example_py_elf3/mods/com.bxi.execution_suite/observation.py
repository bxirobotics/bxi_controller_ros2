"""Observe only inference performed by these states and the public end-of-cycle callback."""
import hashlib
import json
import os
from pathlib import Path
import threading
import time
import uuid
import weakref
from contextlib import contextmanager

from .evaluation.elf3_eval.tap import Tap
from .evaluation.elf3_eval.contract import angle_event, clean

_session = uuid.uuid4().hex
address = f'elf3_model_test_{os.getpid()}_{_session}'
_tap = Tap(address)
_local = threading.local()
_policies = weakref.WeakKeyDictionary()


@contextmanager
def sampling(ctx, running):
    _local.ctx = ctx
    _local.running = running
    try:
        yield
    finally:
        _local.ctx = None


def end_cycle(ctx, state):
    # The official controller returns its MotorFrame and publishes after this observer.
    # The next tick closes this tick's ROS-clock interval.
    latest = getattr(_local, 'latest', None)
    here = state in ('com.bxi.execution_suite/dance', 'com.bxi.execution_suite/sonic06', 'com.bxi.execution_suite/sonic11')
    last = getattr(_local, 'in_action', False)
    _local.in_action = here
    if not (here or last or latest):
        return
    _local.latest = None
    try:
        stamp = ctx.ros_node.get_clock().now().nanoseconds
        now = time.monotonic()
        cycle = ctx.loop_count
        event = dict(schema=1, kind='mod_cycle', t=now, received_monotonic=now,
                     ros_stamp_ns=stamp, cycle=cycle, period_s=float(ctx.dt), session=_session,
                     state=state, model_seq=None, phase='unknown')
        if here and latest and latest['cycle'] == cycle:
            event.update(model_seq=latest['seq'], model_session=_session,
                         model_id=latest['model_id'], model_hash=latest['model_hash'], phase=latest['phase'])
        _tap.capture(lambda: event)
    except Exception:
        _tap.errors += 1


def register_policy(policy, model, *, adapter, policy_class, source_files=(), details=None):
    """Register a decoder's physical output. Hashes are computed off the control path."""
    try:
        paths = [Path(model)] if isinstance(model, (str, Path)) else [Path(a.path) for a in model.artifacts]
        artifacts = {}
        sources = {}
        for collection, files in ((artifacts, paths), (sources, map(Path, source_files))):
            for path in files:
                h = hashlib.sha256()
                with path.open('rb') as stream:
                    for chunk in iter(lambda: stream.read(1024*1024), b''):
                        h.update(chunk)
                collection[str(path)] = h.hexdigest()
        if not artifacts:raise ValueError('no model artifact')
        # Content identity does not depend on installation path.
        hashes = sorted(artifacts.values())
        digest = hashes[0] if len(hashes)==1 else hashlib.sha256(json.dumps(hashes).encode()).hexdigest()
        _policies[policy] = dict(model_id=f'{adapter}:{policy_class}:{paths[0].stem}',
            model_hash=digest, artifacts=artifacts, sources=sources, seq=0,
            adapter=adapter, details=details or {},
            hash_scope='single_artifact' if len(hashes)==1 else 'artifact_contents')
    except Exception:_tap.errors+=1

def policy_output(policy, started, advance, *, reference_frame=None):
    """Observe the exact named physical target returned by the host decoder."""
    try:
        ctx = getattr(_local, "ctx", None)
        if ctx is None:return
        m = _policies.get(policy)
        if m is None:return
        m['seq'] += 1
        end = time.monotonic()
        meta = dict(model_id=m['model_id'], model_hash=m['model_hash'],
                    session=_session, seq=m['seq'])
        phase = 'blend' if (not _local.running or getattr(policy.inner, 'source_blend_active', False)) else 'active' if advance else 'hold'
        _local.latest = dict(meta, phase=phase, cycle=ctx.loop_count)
        def factory():
            target = policy.output.joints
            return angle_event('model', end, target.position, names=target.layout.names,
                **meta, phase=phase, position_target=True, inference_start=started,
                inference_end=end, adapter=m['adapter'],
                stage='post_host_decoder_pre_framework', reference_frame=reference_frame,
                model_artifacts=m['artifacts'], source_artifacts=m['sources'],
                hash_scope=m['hash_scope'], decoder_details=m['details'])
        _tap.capture(factory)
    except Exception:_tap.errors+=1

def test_event(state, status, reason='', *, severity=None, code=None, category=None, details=None):
    try:
        severity=severity or ('error' if status=='failed' else 'warning' if status in ('cancelled','interrupted') else 'info')
        if severity not in ('info','warning','error'):raise ValueError('invalid test event severity')
        now = time.monotonic()
        _tap.capture(lambda:dict(schema=1,kind='test_event',t=now,received_monotonic=now,
            session=_session,state=state,status=status,reason=reason,severity=severity,
            code=code,category=category,details=clean(details or {}),
            accuracy_acceptance='not_assessed_by_playback_controller'))
    except Exception:_tap.errors+=1
