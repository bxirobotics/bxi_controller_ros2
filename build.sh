#!/usr/bin/env bash
set -Ee -o pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
cd "$SCRIPT_DIR"

RKNN_TOOLKIT_VERSION="2.3.2"
PYTHON_EXECUTABLE="${PYTHON_EXECUTABLE:-python3}"

is_rk3588_arm() {
    [[ "$(uname -m)" == "aarch64" ]] || return 1
    local path compatible
    for path in \
        /proc/device-tree/compatible \
        /sys/firmware/devicetree/base/compatible \
        /proc/device-tree/model \
        /sys/firmware/devicetree/base/model; do
        [[ -r "$path" ]] || continue
        compatible="$(tr '\0' '\n' < "$path" 2>/dev/null || true)"
        if grep -qi 'rk3588' <<<"$compatible"; then
            return 0
        fi
    done
    grep -qi 'rk3588' /proc/cpuinfo 2>/dev/null
}

rknn_toolkit_ready() {
    "$PYTHON_EXECUTABLE" - <<'PY'
import importlib.metadata
import platform
import sys

if platform.machine() != "aarch64" or sys.version_info[:2] != (3, 10):
    raise SystemExit(1)
try:
    version = importlib.metadata.version("rknn-toolkit2")
except importlib.metadata.PackageNotFoundError:
    raise SystemExit(1)
if version != "2.3.2":
    raise SystemExit(1)
try:
    from rknn.api import RKNN  # noqa: F401
except Exception:
    raise SystemExit(1)
PY
}

ensure_rknn_models() {
    is_rk3588_arm || return 0

    local model_root="$SCRIPT_DIR/src/bxi_example_py_elf3/mods"
    local model missing=0
    local models=()
    while IFS= read -r -d '' model; do
        models+=("$model")
        if [[ ! -s "${model%.onnx}.rknn" || \
            ! -s "${model%.onnx}.rknn.build.json" ]]; then
            missing=1
        fi
    done < <(find "$model_root" -type f -name '*.onnx' -print0 | sort -z)

    ((${#models[@]} > 0)) || return 0
    ((missing == 0)) && {
        echo "RKNN artifacts are current (${#models[@]} models)."
        return 0
    }

    echo "RK3588 detected; checking RKNN Toolkit ${RKNN_TOOLKIT_VERSION}."
    if ! rknn_toolkit_ready; then
        echo "RKNN Toolkit ${RKNN_TOOLKIT_VERSION} is missing; installing it."
        "$PYTHON_EXECUTABLE" -m pip install --upgrade \
            "rknn-toolkit2==${RKNN_TOOLKIT_VERSION}"
    fi
    rknn_toolkit_ready || {
        echo "error: RKNN Toolkit ${RKNN_TOOLKIT_VERSION} is unavailable for Python 3.10/aarch64" >&2
        return 1
    }

    # A missing artifact triggers one consistent rebuild of the model set.
    "$PYTHON_EXECUTABLE" tools/build_rknn_release.py \
        --mods "$model_root" --force-rebuild
}

if [ -f /opt/bxi/bxi_ros2_pkg/setup.bash ]; then
    source /opt/bxi/bxi_ros2_pkg/setup.bash
fi
ensure_rknn_models
BUILD_TYPE=Release
colcon build \
        --merge-install \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On"\
        -Wall -Wextra -Wpedantic
