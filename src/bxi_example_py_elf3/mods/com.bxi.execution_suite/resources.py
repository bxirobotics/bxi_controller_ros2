"""Resolve model assets independently of the ROS workspace location."""
import os
from pathlib import Path


def model_root(kind):
    override = os.environ.get(f'BXI_{kind.upper()}_ROOT')
    if override:
        return Path(override).expanduser()
    root = Path(__file__).resolve().parent
    candidates = (root / 'assets' / kind, root.parent / ('com.bxi.' + kind),
                  Path.home() / 'elf3_execution_test' / 'current' / kind)
    marker = 'policy.py' if kind == 'sonic' else 'assets/cxkv5_15k.onnx'
    for path in candidates:
        if (path / marker).is_file():
            return path
    raise FileNotFoundError(f'Set BXI_{kind.upper()}_ROOT to the directory containing {marker}')
