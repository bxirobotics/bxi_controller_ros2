"""File reference adapter for the real SONIC policy inside the BXI runtime."""
import hashlib
import importlib
import os
from pathlib import Path
import sys
import time
from types import ModuleType

import numpy as np

from .observation import register_policy, policy_output
from .resources import model_root


def sonic_module(root):
    root=Path(root).resolve()
    name='_bxi_execution_sonic_'+hashlib.sha256(str(root).encode()).hexdigest()[:12]
    if name+'.policy' not in sys.modules:
        package=ModuleType(name);package.__path__=[str(root)];sys.modules[name]=package
        module=importlib.import_module(name+'.policy')
        module._capture_tracking=None # This instance uses the generic eval tap.
    return sys.modules[name+'.policy']


class SonicFilePolicy:
    def __init__(self, reference, *, clip_id, logger):
        root=model_root('sonic')
        module=sonic_module(root)
        self.module=module;self.clip_id=clip_id
        self.reference_path=Path(reference)
        with np.load(reference,allow_pickle=False) as d:
            self.reference={k:np.array(d[k],dtype=np.float32) for k in ('term1_local','root_quat','wrist')}
        self.frames=len(self.reference['root_quat'])
        for key,width in (('term1_local',72),('root_quat',4),('wrist',6)):
            a=self.reference[key]
            if a.shape!=(self.frames,width) or not np.isfinite(a).all() or self.frames<10:
                raise ValueError('invalid SONIC file reference '+key)
        if np.any(np.linalg.norm(self.reference['root_quat'],axis=1)<1e-6):raise ValueError('invalid root quaternion')
        model=root/'assets/sonic_action_token.onnx'
        self.inner=module.SonicTeleopPolicy(str(model),str(root/'assets/stream_reference.npz'),
                                          use_smpl_ref_zmq=False,backend='onnxruntime')
        self.inner.bind_logger(logger)
        self.inner._active_reference=self._active_reference
        self.cursor=0.;self._frame=0.;self._current_reference=None
        self.asset_info=dict(source_type='smpl',motion_names_observed=False,frames=self.frames,
            reference_rate_hz=50,clip_id=clip_id,decode_rule='SONIC original action clip/scale/default/source blend',
            reference_transport='file, same gather/infer/advance order; no PICO or network delay',
            action_scale=self.inner.action_scale.tolist(),default_q=self.inner.default_dof_pos.tolist(),
            source_policy_sha256=hashlib.sha256((root/'policy.py').read_bytes()).hexdigest())
        register_policy(self,str(model),adapter='sonic_file_'+clip_id,
                policy_class=type(self.inner).__name__,source_files=[reference],details=self.asset_info)

    @property
    def output(self):return self.inner.output

    def _active_reference(self):return self._current_reference,'live',time.monotonic()

    def reset(self,frame):
        self.cursor=self._frame=0.
        self.inner.reset(frame)

    def step(self,frame,dt,*,advance=True):
        if not advance or self.finished():return self.output
        index=int(self.cursor);indices=np.minimum(index+np.arange(10),self.frames-1)
        self._current_reference=self.module.SmplReferenceFrame(
            term1_local=self.reference['term1_local'][indices],root_quat=self.reference['root_quat'][indices],
            wrist=self.reference['wrist'][indices],head_joint_pos=np.zeros((10,2),np.float32),
            frame_index=index,sequence=index+1,source_stale=False,source_age_ms=0.,
            valid_horizon=10,clamp_slots=int(np.count_nonzero(index+np.arange(10)>=self.frames)))
        started=time.monotonic();result=self.inner.step(frame,dt,advance=True)
        self._frame=float(index)
        policy_output(self,started,True,reference_frame=index)
        self.cursor+=50*dt
        return result

    def finished(self,trim=0):return self.cursor>=self.frames-trim
    def close(self):self.inner.close()
