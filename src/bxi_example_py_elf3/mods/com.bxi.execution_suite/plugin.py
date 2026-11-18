"""Three independently selected execution tests in one BXI Mod."""
import os

from bxi_example_py_elf3.framework.mod_api import ModDefinition, ResourceKey
from .model_test_safety import ModelTemperatureGuard
from .recorder import Recorder
from .remote_guard import SuiteRemoteGuard
from .resources import model_root
from .state import DanceTestState


class _StartupLogger:
    def info(self, message): pass
    def warning(self, message): pass
    def error(self, message): pass


def load_cxk(resource):
    from .policy import load_dance_policy
    assets = model_root('cxk') / 'assets'
    return load_dance_policy(assets/'cxkv5.npz', assets/'cxkv5_15k.onnx',
                             variant='gravity_isaaclab_v3', start_frame=0, fixed_pos=True)


def load_sonic(resource, clip):
    from .sonic_file import SonicFilePolicy
    return SonicFilePolicy(resource.asset('assets/sonic'+clip+'.npz'),
                           clip_id=clip, logger=_StartupLogger())


def create_mod(context):
    recorder = Recorder()
    thermal = ModelTemperatureGuard(context.mod_root.parent / 'com.bxi.suspended_tests' / 'config')
    factories = {}
    preload = 'startup' if os.environ.get('BXI_EXECUTION_SUITE') == '1' else 'on_demand'
    for name, loader in (('dance', load_cxk), ('sonic06', lambda r: load_sonic(r, '06')),
                         ('sonic11', lambda r: load_sonic(r, '11'))):
        key = ResourceKey('com.bxi.execution_suite/' + name + '_policy')
        context.register_resource(key, loader, policy=preload)
        handle = context.resource(key)
        factories[name] = lambda state, handle=handle: DanceTestState(
            state.name, state.state_id, handle, recorder=recorder, temperature_guard=thermal)
    return ModDefinition(state_factories=factories, state_observer=recorder.end_cycle,
                         remote_event_filter=SuiteRemoteGuard())
