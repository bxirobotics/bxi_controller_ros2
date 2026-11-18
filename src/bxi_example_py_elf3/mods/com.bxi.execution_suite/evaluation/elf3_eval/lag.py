"""Causal trajectory lag search, always reported separately from raw error."""
import numpy as np
from .math_core import causal_indices

def estimate(times, reference, actual, valid, *, max_lag, step, max_gap,
             min_samples=100, min_duration=2., min_motion_deg=5.):
    valid = valid & np.isfinite(reference) & np.isfinite(actual)
    base = dict(estimated_lag_ms=None, samples=int(valid.sum()),
                min_samples=min_samples, min_duration_s=min_duration, min_motion_deg=min_motion_deg)
    if valid.sum()<min_samples: return dict(base, status='insufficient_samples')
    if np.ptp(reference[valid])<min_motion_deg: return dict(base,status='insufficient_motion')
    breaks = ~valid
    breaks = breaks.copy()
    breaks[1:] |= (~valid[:-1]) | (np.diff(times)>max_gap) | (np.diff(times)<=0)
    epochs = np.cumsum(breaks)
    lags = np.arange(0,max_lag+step*.5,step)
    indices = [causal_indices(times,times-l,max_age=max_gap) for l in lags]
    good = valid.copy()
    for ix in indices:
        safe = np.maximum(ix,0)
        good &= (ix>=0)&valid[safe]&(epochs==epochs[safe])
    if good.sum()<min_samples or np.ptp(times[good])<min_duration:
        return dict(base,status='insufficient_contiguous_duration')
    scores = [float(np.sqrt(np.mean((actual[good]-reference[ix[good]])**2))) for ix in indices]
    best = int(np.argmin(scores))
    return dict(base,status='search_boundary' if best==len(lags)-1 else 'trajectory_estimate',
                estimated_lag_ms=float(lags[best]*1000), samples=int(good.sum()),
                compensated_rmse_deg=scores[best],zero_lag_same_samples_rmse_deg=scores[0],
                search_max_ms=max_lag*1000,step_ms=step*1000)
