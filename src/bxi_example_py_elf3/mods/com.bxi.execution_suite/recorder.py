"""Own one passive recording process per loaded Mod, using only public ROS topics."""
import os
from pathlib import Path
import signal
import subprocess
import sys


class Recorder:
    def __init__(self):
        self.ctx = None
        self.users = 0
        self.child = None
        self.ready_fd = None
        self.ready = False

    def bind(self, ctx):
        self.users += 1
        if self.users != 1:
            return
        node = ctx.ros_node
        prefix = getattr(node, 'topic_prefix', '')
        if prefix not in ('simulation/', 'hardware/') or not hasattr(node, 'resolve_topic_name'):
            return
        from .observation import address, end_cycle
        self.observe_cycle = end_cycle
        self.ctx = ctx
        topic = node.resolve_topic_name
        default = '/var/log/bxi_log/execution_eval' if prefix == 'hardware/' else '~/.ros/execution_eval'
        mode = getattr(node, 'state_machine_info_topic', '') or prefix + 'state_machine_info'
        folder = Path(__file__).resolve().parent
        env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
        env['PYTHONPATH'] = str(folder / 'evaluation') + os.pathsep + env.get('PYTHONPATH', '')
        self.ready_fd, sender = os.pipe()
        os.set_blocking(self.ready_fd, False)
        try:
            self.child = subprocess.Popen(
                [sys.executable, '-u', str(folder / 'recorder.py'), str(os.getpid()), str(sender),
                 address, os.environ.get('ELF3_EVAL_OUTPUT') or default, prefix,
                 topic(prefix+'actuator_states'), topic(prefix+'actuators_cmds'), topic(mode),
                 node.get_fully_qualified_name(), str(int(node.get_parameter("use_sim_time").value))],
                env=env, pass_fds=(sender,), stdin=subprocess.DEVNULL, start_new_session=True)
        except OSError as exc:
            node.get_logger().error(f'MODEL_TEST recorder startup failed: {exc}')
        finally:
            os.close(sender)

    def ready_now(self):
        if self.child is None or self.child.poll() is not None:
            return False
        if not self.ready:
            try:
                self.ready = os.read(self.ready_fd, 1) == b'1'
            except BlockingIOError:
                pass
        return self.ready

    def end_cycle(self, state):
        if self.ctx is not None:
            self.observe_cycle(self.ctx, state)

    def unbind(self):
        self.users -= 1
        if self.users:
            return
        if self.child is not None and self.child.poll() is None:
            self.child.terminate()  # closes capture; pending report worker drains independently
            try:
                self.child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
        if self.ready_fd is not None:
            os.close(self.ready_fd)
        self.ctx = self.ready_fd = self.child = None
        self.ready = False


def main():
    import ctypes
    import yaml
    from elf3_eval.auto_record import auto_record
    parent, sender, address, output, prefix, feedback, command, mode, publisher, sim_clock = sys.argv[1:]
    # Linux sends SIGTERM even if the controller exits without unbinding states.
    if ctypes.CDLL(None, use_errno=True).prctl(1, signal.SIGTERM, 0, 0, 0) or os.getppid() != int(parent):
        return
    robot_id = 'simulation' if prefix == 'simulation/' else None
    if robot_id is None:
        try:
            path = Path(os.environ.get('BXI_ROBOT_CONFIG_FILE') or '/opt/bxi/robot_config.yaml')
            value = (yaml.safe_load(path.read_text()) or {}).get('robot_id')
            if value is not None:
                robot_id = 'ELF3-' + str(value).removeprefix('ELF3-')
        except (OSError, ValueError, yaml.YAMLError):
            pass
    def ready():
        os.write(int(sender), b'1')
        os.close(int(sender))
    auto_record(output, seconds=0, robot_id=robot_id, address=address, state_topic=feedback,
                command_topic=command, mode_topic=mode, ready=ready, publisher=publisher, sim_clock=bool(int(sim_clock)))


if __name__ == '__main__':
    main()
