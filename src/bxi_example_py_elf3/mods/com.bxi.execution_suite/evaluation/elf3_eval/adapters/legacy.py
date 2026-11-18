"""Adapt existing recordings without importing any SONIC runtime package."""
import base64
import numpy as np
from .stk1 import decode
from ..contract import angle_event

def convert(e, *, source_age_max=1.0):
    k, v = e['kind'], e['value']
    rx = float(e['received_monotonic'])
    common = dict(received_monotonic=rx, ros_now_ns=e.get('ros_now_ns'),
                  received_wall_ns=e.get('received_wall_ns'))
    if k in ('state', 'command'):
        stamp = int(v['stamp_ns']); now = int(e['ros_now_ns'])
        age = (now-stamp)*1e-9
        valid = stamp > 0 and 0 <= age <= source_age_max
        out = angle_event('actual' if k == 'state' else k, rx-age if valid else rx,
                          v['position'], names=v['names'])
        if 'sole_controller_publisher' in e:
            out['sole_controller_publisher'] = e['sole_controller_publisher']
        out.update(common, source_stamp_ns=stamp, source_clock='ros', time_valid=valid,
                   source_age_s=age, kp=v.get('kp'), phase='unknown', enabled=None)
        if k=='state' and any(field in v for field in ('motor_temperature','driver_temperature')):
            out['temperature_names']=list(v['names'])
            for field in ('motor_temperature','driver_temperature'):
                if field in v:out[field]=v[field]
        return out
    if k == 'trace':
        m, a = decode(base64.b64decode(v['packet_b64'], validate=True))
        expected = np.asarray(m['default_q']) + a['clipped_action']*np.asarray(m['action_scale'])
        if not np.allclose(expected, a['decoded_target'], atol=2e-6, rtol=2e-6):
            raise ValueError('SONIC decoded target disagrees with recorded scale/default/action')
        phase = ('blend' if m['source_blend_active'] else
                 'hold' if m['policy_status'] == 'stale_hold' else 'active')
        out = angle_event('model', m['completed_monotonic'], a['decoded_target'],
                          names=m['joint_names'], model_id=m['model_id'],
                          model_hash=m['model_sha256'], session=m['session'], seq=m['sequence'],
                          phase=phase, position_target=True,
                          inference_start=m['started_monotonic'], inference_end=m['completed_monotonic'],
                          adapter='sonic-stk1', stage='post_action_clip_scale_offset_pre_source_blend',
                          policy_target=a['policy_target'].tolist(),
                          motion_token=a['motion_token'].tolist(),
                          sonic_source=m.get('source'), sonic_policy_status=m.get('policy_status'),
                          source_blend_active=m.get('source_blend_active'),
                          reference_frame_index=m.get('reference_frame_index'),
                          reference_sequence=m.get('reference_sequence'),
                          reference_received_monotonic=m.get('reference_received_monotonic'),
                          tap_sent=m.get('tap_sent',0),
                          tap_dropped=m.get('tap_dropped', 0), tap_errors=m.get('tap_errors', 0))
        out.update(common)
        return out
    if k == 'state_machine':
        return dict(schema=1, kind='mode', t=rx, state=v.get('current', {}).get('name', ''),
                    phase='blend' if v.get('transition') else 'unknown', **common)
    return None
