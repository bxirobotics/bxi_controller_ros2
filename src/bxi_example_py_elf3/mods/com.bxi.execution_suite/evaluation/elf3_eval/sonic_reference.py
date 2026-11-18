"""Optional offline SONIC g1_kin reconstruction; never a control target.

The unpacking and FSQ checks are reused from tracking_eval/evaluation/
kinematic_reference.py. Runtime assets remain an explicitly supplied bundle.
"""
from collections import Counter
import hashlib
import json
from pathlib import Path
import re

import numpy as np

from .contract import BODY

PACKING = 'raw[..., :58].reshape(batch,580): positions[:290], velocities[290:]; rotation=raw[...,58:64]'


def unpack(raw):
    """Training concatenated ALL q frames then ALL dq frames before reshape."""
    raw = np.asarray(raw)
    if raw.ndim != 3 or raw.shape[1:] != (10, 64):
        raise ValueError('unexpected g1_kin shape; expected (batch,10,64)')
    packed = raw[:, :, :58].reshape(len(raw), 580)
    return packed[:, :290].reshape(-1, 10, 29), packed[:, 290:].reshape(-1, 10, 29), raw[:, :, 58:64]


def valid_token(value):
    try:
        token = np.asarray(value, dtype=np.float32)
    except (TypeError, ValueError):
        return None
    if (token.shape != (64,) or not np.isfinite(token).all()
            or not ((token >= -1.00001) & (token <= .93751)).all()
            or not (np.abs(token * 16 - np.rint(token * 16)) < .0001).all()):
        return None
    return token


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


class SonicReference:
    def __init__(self, bundle):
        root = Path(bundle)
        manifest_path = root / 'kinematic_manifest.json'
        self.manifest = m = json.loads(manifest_path.read_text())
        required = dict(schema=1, reference_kind='learned_kinematic_reconstruction',
                        units='rad', current_slot=0, robot_window_frames=10, packing=PACKING,
                        reference_angle_transform='absolute joint coordinates; no action scaling or default-pose addition')
        for key, expected in required.items():
            if m.get(key) != expected:
                raise ValueError('unsupported SONIC reference manifest field: ' + key)
        names = m.get('joint_names', [])
        if len(names) != 29 or set(names) != set(BODY):
            raise ValueError('SONIC reference must describe the same 29 named body joints')
        self.order = [names.index(name) for name in BODY]
        hashes = m.get('compatible_policy_sha256', [])
        if (not isinstance(hashes, list) or not hashes
                or any(not isinstance(h, str) or not re.fullmatch('[0-9a-f]{64}', h) for h in hashes)):
            raise ValueError('SONIC reference needs explicit compatible policy SHA256 values')
        self.compatible = set(hashes)
        if not isinstance(m.get('robot_window_dt_s'), (int, float)) or not np.isfinite(m['robot_window_dt_s']) or m['robot_window_dt_s'] <= 0:
            raise ValueError('invalid SONIC reference window interval')
        model = root / 'g1_kin_step28800.onnx'
        if sha256(model) != m.get('kinematic_onnx_sha256'):
            raise ValueError('SONIC reference ONNX SHA256 does not match its manifest')
        self.model = model
        self.session = None
        self.records = []
        self.counts = Counter()
        self.errors = []
        self.metadata = dict(enabled=True, reference_kind=m['reference_kind'],
            is_independent_ground_truth=False, joint_names=list(BODY), unit='rad',
            manifest_sha256=sha256(manifest_path),
            kinematic_onnx_sha256=m['kinematic_onnx_sha256'],
            checkpoint_sha256=m.get('checkpoint_sha256'), compatible_policy_sha256=hashes,
            current_slot=0, robot_window_frames=10, robot_window_dt_s=m['robot_window_dt_s'],
            alignment='Same inference as q_model after command association; current slot 0 held causally. No future slot, time shift, or crossing segment boundaries.',
            eligibility='Matching SONIC policy hash, valid post-FSQ token, active live_reference, no source blend; normal feedback/command/model validity rules also apply.',
            missing_token_help='If using a pre-0.2 SQLite index, re-import the original JSONL[.gz]; old indexes did not retain motion_token. Command/state-only recordings cannot reconstruct a reference.')

    def _open_session(self):
        try:
            import onnxruntime as ort
        except ImportError as exc:
            raise ImportError("SONIC reference needs the optional dependency: pip install '.[sonic-reference]'") from exc
        options = ort.SessionOptions()
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        return ort.InferenceSession(str(self.model), sess_options=options, providers=['CPUExecutionProvider'])

    def attach(self, models):
        """Attach one independently reconstructed reference to each inference."""
        n = len(models)
        raw = np.full((n, 10, 64), np.nan, np.float32)
        tokens = np.zeros((n, 64), np.float32)
        reasons = []
        for i, event in enumerate(models):
            token = valid_token(event.get('motion_token'))
            if event.get('adapter') != 'sonic-stk1': reason = 'not_sonic_trace'
            elif event.get('model_hash') not in self.compatible: reason = 'incompatible_policy_hash'
            elif event.get('motion_token') is None: reason = 'missing_motion_token_reimport_required'
            elif token is None: reason = 'invalid_fsq_token'
            elif not event.get('time_valid', True): reason = 'invalid_model_time'
            elif event.get('phase') != 'active' or event.get('source_blend_active'):
                reason = 'hold_or_blend_reference_excluded'
            elif event.get('sonic_source') != 'live' or event.get('sonic_policy_status') != 'live_reference':
                reason = 'not_live_reference'
            else:
                reason = 'available'
                tokens[i] = token
            reasons.append(reason)
        indices = [i for i, reason in enumerate(reasons) if reason == 'available']
        if indices and self.session is None:
            self.session = self._open_session()
        for start in range(0, len(indices), 256):
            ix = indices[start:start + 256]
            try:
                decoded = np.asarray(self.session.run(None, {'motion_token': tokens[ix].reshape(-1, 2, 32)})[0])
                if decoded.shape != (len(ix), 10, 64):
                    raise ValueError('unexpected decoder batch shape')
                raw[ix] = decoded
            except Exception as exc:
                message = type(exc).__name__ + ': ' + str(exc)
                if message not in self.errors: self.errors.append(message)
                for i in ix: reasons[i] = 'decoder_failed'
        q, dq, rotation = unpack(raw)
        q, dq = q[:, :, self.order], dq[:, :, self.order]
        for i, event in enumerate(models):
            if reasons[i] == 'available' and not np.isfinite(raw[i]).all():
                reasons[i] = 'nonfinite_decoder_output'
            if reasons[i] != 'available':
                q[i] = np.nan; dq[i] = np.nan; rotation[i] = np.nan
            event['_q_ref'] = q[i, 0]
            event['_reference_reason'] = reasons[i]
            self.records.append(dict(t=event['t'], session=event.get('session', ''),
                seq=event.get('seq', -1), model_id=event.get('model_id', ''),
                model_hash=event.get('model_hash') or '', epoch=event.get('epoch', 0),
                reason=reasons[i], q=q[i], dq=dq[i], rotation=rotation[i]))
        counts = Counter(reasons)
        self.counts.update(counts)
        return dict(inference_events=n, inference_status_counts=dict(counts))

    def save(self, output):
        """Keep full windows separately from the aligned current-frame curve."""
        rows = self.records
        arrays = {k: np.asarray([r[k] for r in rows], dtype=dtype) for k, dtype in
                  [('t', float), ('session', str), ('seq', np.int64), ('model_id', str),
                   ('model_hash', str), ('epoch', np.int64), ('reason', str)]}
        for key, name, width in [('q', 'reference_window_rad', 29),
                                 ('dq', 'reference_velocity_window_rad_s', 29),
                                 ('rotation', 'relative_anchor_rotation6d', 6)]:
            arrays[name] = np.stack([r[key] for r in rows]) if rows else np.empty((0, 10, width), np.float32)
        arrays.update(joint_names=np.asarray(BODY), current_slot=np.asarray(0),
                      robot_window_dt_s=np.asarray(self.manifest['robot_window_dt_s']))
        np.savez_compressed(output, **arrays)
