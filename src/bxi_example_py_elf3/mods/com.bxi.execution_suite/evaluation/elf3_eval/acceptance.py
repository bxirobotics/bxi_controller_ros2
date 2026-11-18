"""Configurable engineering screening of encoder following; never controls motors."""
from copy import deepcopy
import math

from .contract import BODY

PROFILE = {
    "id": "elf3-dynamic-screening-v1",
    "basis": "Provisional engineering screening; not an ISO or OEM acceptance limit",
    "comparison": "actual_minus_model",
    "minimum_span_s": 2.0,
    "minimum_inference_events": 50,
    "minimum_time_coverage": 0.95,
    "limits_deg": {
        "mae_deg": 3.0, "rmse_deg": 5.0, "p95_abs_deg": 10.0,
        "bias_deg": 3.0, "max_abs_deg": 20.0,
    },
    "joint_overrides": {},
    "sources": [
        "https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/parameters.html",
        "https://www.iso.org/standard/22244.html",
    ],
}
METRICS = tuple(PROFILE["limits_deg"])


def validate(profile):
    if not isinstance(profile,dict):raise ValueError('acceptance profile must be a JSON object')
    p = deepcopy(profile)
    allowed = set(PROFILE)
    if set(p) - allowed:
        raise ValueError("unknown acceptance profile fields")
    required = allowed - {'sources', 'joint_overrides'}
    if required - set(p):raise ValueError('acceptance profile missing required fields')
    if p.get("comparison") not in ("actual_minus_model", "actual_minus_command"):
        raise ValueError("unsupported acceptance comparison")
    if not all(isinstance(p.get(k),str) and p[k].strip() for k in ('id','basis')):
        raise ValueError("acceptance id and basis required")
    for key in ("minimum_span_s", "minimum_time_coverage"):
        value = p[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError("invalid " + key)
    if p["minimum_time_coverage"] > 1:
        raise ValueError("coverage must not exceed one")
    n = p["minimum_inference_events"]
    if isinstance(n, bool) or not isinstance(n, int) or n < 1:
        raise ValueError("positive inference count required")
    if not isinstance(p['limits_deg'],dict) or set(p["limits_deg"]) != set(METRICS):
        raise ValueError("all five angle limits must be specified")
    if not isinstance(p.get('joint_overrides',{}),dict) or set(p.get("joint_overrides", {})) - set(BODY):
        raise ValueError("unknown joint override")
    for limits in (p["limits_deg"], *p.get("joint_overrides", {}).values()):
        if not isinstance(limits, dict) or set(limits) - set(METRICS):
            raise ValueError("unknown joint metric")
        for limit in limits.values():
            if isinstance(limit, bool) or not isinstance(limit, (float, int)) or not math.isfinite(limit) or limit <= 0:
                raise ValueError("limits must be positive finite degrees")
    return p


def grade_segment(segment, profile):
    p = validate(profile)
    out = dict(segment=segment["id"], model_id=segment.get("model_id"),
               model_hash=segment.get("model_hash"), status="not_assessed",
               reasons=[], joints={})
    if segment.get("phase") != "active":
        out["reasons"].append("transition_or_hold_excluded")
        return out
    if not segment.get("model_id") and p["comparison"] == "actual_minus_model":
        out["reasons"].append("model_output_unobserved")
        return out
    if segment.get("span_s", 0) < p["minimum_span_s"]:
        out["reasons"].append("insufficient_duration")
    if p["comparison"] == "actual_minus_model" and segment.get("independent_inference_events", 0) < p["minimum_inference_events"]:
        out["reasons"].append("insufficient_independent_model_events")
    rows = segment.get("comparisons", {}).get(p["comparison"], {})
    failed = False
    incomplete = bool(out["reasons"])
    for name in BODY:
        row = rows.get(name, {})
        limits = dict(p["limits_deg"], **p.get("joint_overrides", {}).get(name, {}))
        problems = []
        coverage = row.get("time_coverage_fraction")
        if (isinstance(coverage, bool) or not isinstance(coverage, (int, float)) or not math.isfinite(coverage)
                or coverage < p["minimum_time_coverage"]):
            problems.append("insufficient_time_coverage")
        values = {}
        violations = {}
        for metric in METRICS:
            value = row.get(metric)
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
                problems.append("missing_" + metric)
                continue
            values[metric] = value
            observed = abs(value) if metric == "bias_deg" else value
            if observed < 0:
                problems.append("invalid_" + metric)
            elif observed > limits[metric]:
                violations[metric] = dict(observed_deg=value, limit_deg=limits[metric])
        status = "insufficient_data" if problems else "screening_fail" if violations else "screening_pass"
        failed |= bool(violations)
        incomplete |= bool(problems)
        out["joints"][name] = dict(status=status, reasons=problems,
                                  values_deg=values, limits_deg=limits,
                                  violations=violations)
    out["observed_exceedance"] = failed
    out["status"] = "insufficient_data" if incomplete else "screening_fail" if failed else "screening_pass"
    return out


def execution_status(events):
    """Keep an early return distinct from completed playback and precision."""
    runs = []
    pending = {}
    for e in events:
        state = e.get('state', '')
        if state.rsplit('/', 1)[-1] not in ('dance', 'sonic06', 'sonic11'):
            continue
        if not state.startswith(('com.bxi.execution_suite/', 'com.bxi.dance_test/')):
            continue
        key = (e.get('session'), state)
        status = e.get('status')
        if status == 'started':
            run = dict(state=state, session=e.get('session'), status='incomplete', started_s=e.get('t'))
            runs.append(run); pending[key] = run
        elif status in ('playback_complete', 'cancelled', 'failed', 'interrupted'):
            run = pending.pop(key, None)
            if run is None:
                run = dict(state=state, session=e.get('session'), status='incomplete')
                runs.append(run)
            run.update(status=status, ended_s=e.get('t'), reason=e.get('reason'))
    status = ('not_observed' if not runs else 'failed' if any(r['status']=='failed' for r in runs)
              else 'incomplete' if any(r['status']!='playback_complete' for r in runs) else 'playback_complete')
    return dict(status=status, runs=runs)


def assess(result, profile=None):
    p = validate(PROFILE if profile is None else profile)
    rows = [grade_segment(s, p) for s in result.get("segments", [])]
    assessed = [s for s in rows if s["status"] != "not_assessed"]
    status = ("not_assessed" if not assessed else
              "screening_fail" if any(s["status"] == "screening_fail" for s in assessed) else
              "insufficient_data" if any(s["status"] == "insufficient_data" for s in assessed) else
              "screening_pass")
    command_profile = dict(p, comparison='actual_minus_command')
    data=result.get('data_integrity',{})
    recorder=data.get('recorder',{})
    integrity_issues=bool(data.get('import_issues') or recorder.get('writer_errors')
                          or any(recorder.get('queue_lost',{}).values()) or recorder.get('callback_errors'))
    if integrity_issues and status=='screening_pass':status='insufficient_data'
    return dict(profile=p, status=status, segments=rows,
                data_integrity_status='issues_observed' if integrity_issues else 'no_reported_issues',
                incomplete_segments=[s['segment'] for s in rows if s['status']=='insufficient_data'],
                command_tracking_segments=[grade_segment(s, command_profile) for s in result.get('segments', [])],
                execution=execution_status(result.get('test_events', [])),
                observed_exceedance=any(s.get("observed_exceedance", False) for s in rows),
                completion="reported_separately_in_test_events",
                semantics="All 29 joints assessed separately; uncompensated errors only; screening is not a hardware diagnosis",
                automatic_control_action="none")
