"""Model-test diagnostics and reuse of the original suspended-test thermal policy."""
import importlib.util
from pathlib import Path
import sys


def diagnostic(reason):
    """Classify existing protections without inventing new control thresholds."""
    text = reason.lower()
    rules = (
        (("temperature configuration",), "THERMAL_CONFIGURATION", "temperature"),
        (("reached test stop",), "MOTOR_OVER_TEMPERATURE", "temperature"),
        (("temperature feedback is missing",), "THERMAL_FEEDBACK_MISSING", "temperature"),
        (("temperature feedback is stale",), "THERMAL_FEEDBACK_STALE", "temperature"),
        (("invalid motor temperature",), "MOTOR_TEMPERATURE_INVALID", "temperature"),
        (("temperature names/values", "temperature is missing joint"), "TEMPERATURE_LAYOUT_INVALID", "temperature"),
        (("temperature",), "THERMAL_FEEDBACK_INVALID", "temperature"),
        (("feedback stale", "feedback is stale"), "JOINT_FEEDBACK_STALE", "feedback"),
        (("feedback",), "JOINT_FEEDBACK_INVALID", "feedback"),
        (("control cycle gap",), "CONTROL_CYCLE_GAP", "timing"),
        (("orientation",), "ORIENTATION_UNSAFE", "orientation"),
        (("nonfinite", "non-finite", "invalid sonic output", "invalid model output"), "MODEL_OUTPUT_INVALID", "model"),
        (("negative", "gains"), "MODEL_GAINS_INVALID", "model"),
        (("body29", "joint count", "joint names"), "JOINT_LAYOUT_INVALID", "model"),
        (("software joint limits",), "TARGET_LIMIT_REJECTED", "command"),
        (("normal return", "normal rejected"), "NORMAL_RETURN_REJECTED", "transition"),
    )
    for terms, code, category in rules:
        if any(term in text for term in terms):
            return code, category
    return "MODEL_EXECUTION_FAILED", "model"


class ModelTemperatureGuard:
    """Load configuration off the control path; check the exact original rules."""
    def __init__(self, config_dir):
        path = Path(config_dir).parent / "control_states.py"
        name = "_elf3_model_test_thermal_policy"
        if name not in sys.modules:
            spec = importlib.util.spec_from_file_location(name, path)
            module = importlib.util.module_from_spec(spec)
            sys.modules[name] = module
            try:
                spec.loader.exec_module(module)
            except Exception:
                sys.modules.pop(name, None)
                raise
        self.module = sys.modules[name]
        self.policy = None
        self.configuration_error = None
        try:
            self.policy = self.module.load_temperature_policy(Path(config_dir))
        except (OSError, ValueError, self.module.yaml.YAMLError) as exc:
            self.configuration_error = "motor temperature configuration invalid: " + str(exc)

    def check(self, ctx):
        prefix = getattr(getattr(ctx, "ros_node", None), "topic_prefix", "")
        if prefix == "simulation/":
            return None
        if prefix != "hardware/":
            return {"reason": "actuator temperature feedback source is not hardware or simulation",
                    "code": "THERMAL_SOURCE_UNKNOWN", "category": "temperature", "details": {}}
        reason = self.configuration_error
        if reason is None:
            reason = self.module.temperature_fault(
                ctx, policy=self.policy, timeout_sec=self.module.TEMPERATURE_TIMEOUT_SEC)
        if reason is None:
            return None
        code, category = diagnostic(reason)
        sample = getattr(ctx, "actuator_temperatures", None)
        details = {"maximum_feedback_age_s": self.module.TEMPERATURE_TIMEOUT_SEC,
                   "stop_margin_below_torque_limit_c": self.module.TEST_STOP_MARGIN_C}
        if sample is not None:
            details["feedback_age_s"] = self.module.time.monotonic() - sample.received_at
            if self.policy is not None:
                for name, value in zip(sample.names, sample.motor_c):
                    model, limit, shutdown, defaulted = self.policy.limits_for(name)
                    if isinstance(value, (int, float)) and value >= limit - self.module.TEST_STOP_MARGIN_C:
                        details.update(joint=name, motor_model=model, observed_c=float(value),
                                       threshold_c=limit-self.module.TEST_STOP_MARGIN_C,
                                       torque_limit_c=limit, shutdown_c=shutdown, defaulted=defaulted)
                        break
        return dict(reason=reason, code=code, category=category, details=details)
