"""Functions adapted from SONIC tracking_eval; see PROVENANCE.json."""

import numpy as np

def named(values, names, expected):
    if len(values)!=len(names) or len(set(names))!=len(names):raise ValueError('joint name/size mismatch')
    lookup = dict(zip(names,values))
    # Missing and nonfinite names remain explicit NaN, never default pose/zero.
    return np.array([lookup.get(k,np.nan) for k in expected],dtype=float)

def causal_indices(times, queries, max_age=.05):
    times, queries = np.asarray(times),np.asarray(queries)
    if not len(times):return np.full(len(queries),-1,dtype=int)
    indices = np.searchsorted(times,queries,side='right')-1
    ages = queries-times[np.maximum(indices,0)]
    return np.where((indices>=0)&(ages>=-1e-9)&(ages<=max_age),indices,-1)

def metrics(error):
    x = np.asarray(error,dtype=float)
    x = x[np.isfinite(x)]
    if not len(x):return dict(n=0,mae_deg=None,rmse_deg=None,bias_deg=None,p95_abs_deg=None,max_abs_deg=None)
    return dict(n=int(len(x)),mae_deg=float(np.mean(np.abs(x))),rmse_deg=float(np.sqrt(np.mean(x*x))),
        bias_deg=float(np.mean(x)),p95_abs_deg=float(np.percentile(np.abs(x),95)),max_abs_deg=float(np.max(np.abs(x))))
