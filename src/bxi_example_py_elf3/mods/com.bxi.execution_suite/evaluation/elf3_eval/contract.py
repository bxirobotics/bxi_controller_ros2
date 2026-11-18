"""Named physical angles; raw action tensors never silently become radians."""
import json
from pathlib import Path
import numpy as np
from .math_core import named

BODY = tuple(json.loads(Path(__file__).with_name('body29.json').read_text())['joint_names'])
GROUPS = {'body29': list(range(29)), 'waist': list(range(3)),
          'left_leg': list(range(3, 9)), 'right_leg': list(range(9, 15)),
          'left_arm': list(range(15, 22)), 'right_arm': list(range(22, 29))}

def vector(values, names):
    result = named(values, names, BODY)
    result[~np.isfinite(result)] = np.nan
    return result

def clean(value):
    if isinstance(value, dict): return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)): return [clean(v) for v in value]
    if isinstance(value, np.ndarray): return clean(value.tolist())
    if isinstance(value, np.generic): return clean(value.item())
    if isinstance(value, float) and not np.isfinite(value): return None
    return value

def dump(path, value):
    Path(path).write_text(json.dumps(clean(value), ensure_ascii=False, indent=2, allow_nan=False)+'\n')

def angle_event(kind, t, q, *, names=BODY, **meta):
    return dict(schema=1, kind=kind, t=float(t), received_monotonic=float(t),
                unit='rad', names=list(names), q=clean(q), **meta)
