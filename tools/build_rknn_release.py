#!/usr/bin/env python3
"""Build and validate the built-in RK3588 policy artifacts without an NPU."""

import argparse
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
MODS = ROOT / "src/bxi_example_py_elf3/mods"
sys.path.insert(0, str(MODS.parent))

from bxi_example_py_elf3.framework.inference.model import ModelSpec  # noqa: E402
from bxi_example_py_elf3.framework.inference.backends.rknn_builder import (  # noqa: E402
    prepare_rknn_artifact,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="validate without converting")
    parser.add_argument("--force-rebuild", action="store_true", help="rebuild existing RKNN files")
    parser.add_argument("--mods", type=Path, default=MODS, help="source or installed Mod directory")
    args = parser.parse_args()
    models = sorted(args.mods.rglob("*.onnx"))
    if not models:
        raise SystemExit(f"No ONNX models found in {args.mods}")

    if args.check:
        os.environ["BXI_RKNN_CONVERT_ON_LOAD"] = "0"
    elif args.force_rebuild:
        os.environ["BXI_RKNN_CONVERT_ON_LOAD"] = json.dumps(
            {"target": "rk3588", "force_rebuild": True}, separators=(",", ":")
        )
    else:
        os.environ["BXI_RKNN_CONVERT_ON_LOAD"] = "rk3588"
    for model in models:
        # Built-in policies use actions on the NPU; trajectory outputs stay
        # in the existing NPZ/ONNX sidecar path used by those policies.
        artifact = ModelSpec.portable_onnx(
            model, input_names=(), output_names=("actions",), rknn_target="rk3588"
        ).artifacts[0]
        result = prepare_rknn_artifact(artifact)
        manifest = Path(str(artifact.resolved_path) + ".build.json")
        if not result.ready or not manifest.is_file():
            raise SystemExit(f"{model.name}: {result.reason}; build contract required")
        print(f"OK {model.relative_to(args.mods)}: {result.reason}", flush=True)
    print(f"Validated {len(models)} RK3588 models and build contracts")


if __name__ == "__main__":
    main()
