"""Read-only STK1 decoder extracted from the existing SONIC recorder."""

import json, struct
import numpy as np

MAGIC = b'STK1'

ARRAYS = ('model_input', 'raw_action', 'clipped_action', 'decoded_target',
          'policy_target', 'observed_q', 'observed_dq', 'motion_token', 'root_quat')

SIZES = (1770, 29, 29, 29, 29, 29, 29, 64, 40)

def decode(packet):
    if packet[:4] != MAGIC or len(packet) < 8:
        raise ValueError('unknown trace packet')
    length, = struct.unpack('<I', packet[4:8])
    if length > 16384 or len(packet) != 8+length+4*sum(SIZES):
        raise ValueError('trace packet size mismatch')
    meta = json.loads(packet[8:8+length])
    if meta.get('schema') != 1:
        raise ValueError('unknown trace schema')
    offset = 8+length
    result = {}
    for name, size in zip(ARRAYS, SIZES):
        array = np.frombuffer(packet, dtype='<f4', count=size, offset=offset).copy()
        if not np.isfinite(array).all():
            raise ValueError('nonfinite '+name)
        result[name] = array
        offset += 4*size
    return meta, result
