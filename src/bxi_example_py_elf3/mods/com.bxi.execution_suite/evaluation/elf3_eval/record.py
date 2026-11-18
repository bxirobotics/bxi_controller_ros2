"""Passive bounded recorder. ROS imports only inside record()."""
from collections import Counter
import base64
import gzip
import json
import math
import os
from pathlib import Path
import queue
import signal
import socket
import threading
import time
from .contract import dump, clean

class AsyncWriter:
    def __init__(self,path,capacity=4096,*,sink=None):
        self.path=Path(path);self.queue=queue.Queue(capacity)
        self.accepted=Counter();self.dropped=Counter();self.errors=[];self.written=0
        from .diagnostics import FrameworkDiagnostics
        self.framework_diagnostics = FrameworkDiagnostics()
        self.sink=sink
        self.stream=None if sink else gzip.open(self.path,'xb',compresslevel=1)
        self.thread=threading.Thread(target=self._run,daemon=True);self.thread.start()

    def submit(self,event):
        if self.errors:return False
        try:self.queue.put_nowait(event);self.accepted[event.get('kind','unknown')]+=1;return True
        except queue.Full:self.dropped[event.get('kind','unknown')]+=1;return False

    def _run(self):
        try:
            while True:
                try:e=self.queue.get(timeout=.1)
                except queue.Empty:
                    if self.sink:self.sink.tick(time.monotonic())
                    continue
                if e is None:break
                if 'state_json' in e:
                    raw=json.loads(e['state_json'])
                    e['state']=raw.get('current',{}).get('name','')
                    e['phase']='blend' if raw.get('transition') else 'unknown'
                    if self.sink:
                        for notice in self.framework_diagnostics.update(raw, e):
                            self.sink.write(notice)
                if '_packet' in e:
                    packet=e.pop('_packet')
                    if e.pop('_legacy',False):
                        e.update(kind='trace',value={'packet_b64':base64.b64encode(packet).decode()})
                    else:
                        payload=json.loads(packet)
                        payload['received_monotonic']=e['received_monotonic']
                        payload['received_wall_ns']=e['received_wall_ns']
                        e=payload
                if self.sink:self.sink.write(e)
                else:self.stream.write((json.dumps(clean(e),separators=(',',':'),allow_nan=False)+'\n').encode())
                self.written+=1
        except Exception as exc:self.errors.append(repr(exc))
        finally:
            try:
                if self.sink:self.sink.close()
                else:self.stream.close()
            except Exception as exc:self.errors.append(repr(exc))

    def close(self):
        # Bounded shutdown even if disk writer failed with a full queue.
        deadline=time.monotonic()+10
        while self.thread.is_alive() and time.monotonic()<deadline:
            try:self.queue.put(None,timeout=.1);break
            except queue.Full:pass
        self.thread.join(timeout=max(0,deadline-time.monotonic()))
        if self.thread.is_alive():self.errors.append('writer shutdown timeout; capture incomplete')

def recording_deadline(started, seconds):
    if not math.isfinite(seconds) or not 0<=seconds<=86400:
        raise ValueError('seconds must be in [0,86400]; 0 records until stopped')
    return None if seconds==0 else started+seconds


def record(output,seconds=60,address='elf3_model_eval_v1',socket_only=False,
           legacy_sonic=False,state_topic='/hardware/actuator_states',
           command_topic='/hardware/actuators_cmds',mode_topic='/hardware/state_machine_info',robot_id=None,
           _sink=None, ready=None, publisher=None, sim_clock=False):
    recording_deadline(0,seconds)
    out=Path(output);out.mkdir(parents=True,exist_ok=True)
    if any(out.iterdir()):raise FileExistsError('recording directory must be empty')
    sockets=[];node=None;rclpy=None;writer=None
    started=time.monotonic(); stop=threading.Event(); old_signals={};callback_errors=Counter()
    deadline=recording_deadline(started,seconds)
    dump(out/'metadata.json',dict(schema=1,hostname=socket.gethostname(),robot_id=robot_id,
        execution_suite_mode=os.environ.get('BXI_EXECUTION_SUITE_MODE'),
        boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        started_monotonic=started,started_wall_ns=time.time_ns(),hardware_commands_sent=0,
        model_link_method="mod_cycle_window" if publisher else None, controller_publisher=publisher,
        topics=dict(state=state_topic,command=command_topic,mode=mode_topic),
        socket_only=socket_only,legacy_sonic=legacy_sonic,queue_capacity=4096,
        requested_duration_s=seconds,until_stopped=deadline is None))
    try:
        for name,legacy in [(address,False)]+([('elf3_sonic_tracking_v1',True)] if legacy_sonic else []):
            s=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
            try:s.bind('\0'+name)
            except Exception:s.close();raise
            s.setblocking(False);sockets.append((s,legacy))
        writer=AsyncWriter(out/'events.jsonl.gz',sink=_sink)
        if _sink:
            _sink.statistics=lambda:dict(queue_lost=dict(writer.dropped),
                callback_errors=dict(callback_errors),writer_errors=list(writer.errors))
        if not socket_only:
            import rclpy
            from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
            from communication.msg import ActuatorStates,ActuatorCmds
            from std_msgs.msg import String
            rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'] if sim_clock else [])
            node=rclpy.create_node('elf3_eval_observer_'+str(os.getpid()),enable_rosout=False)
            qos=QoSProfile(depth=256,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
            sole_publisher = False
            def refresh_publisher():
                nonlocal sole_publisher
                endpoints = node.get_publishers_info_by_topic(command_topic)
                sole_publisher = (len(endpoints) == 1 and
                    endpoints[0].node_namespace.rstrip('/') + '/' + endpoints[0].node_name == publisher)
            if publisher:
                refresh_publisher()
                node.create_timer(.25, refresh_publisher)
            def motor(msg,kind):
                try:
                    rx=time.monotonic();ros_now=node.get_clock().now().nanoseconds
                    wall=time.time_ns();is_cmd=kind=='command'
                    v=dict(stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                           frame_id=msg.header.frame_id,
                           names=list(msg.actuators_name if is_cmd else msg.name),
                           position=list(msg.pos if is_cmd else msg.position),
                           velocity=list(msg.vel if is_cmd else msg.velocity))
                    for field in (('kp','kd','torque') if is_cmd else ('effort',)):v[field]=list(getattr(msg,field))
                    if not is_cmd:
                        for field in ('motor_temperature','driver_temperature'):
                            v[field]=list(getattr(msg,field,()))
                    writer.submit(dict(kind=kind,received_monotonic=rx,received_wall_ns=wall,
                                       ros_now_ns=ros_now,value=v,sole_controller_publisher=sole_publisher if is_cmd else None))
                except Exception as exc:callback_errors[type(exc).__name__]+=1
            def mode(msg):
                # JSON parsing and disk work run on the writer thread.
                writer.submit(dict(schema=1,kind='mode',t=time.monotonic(),received_monotonic=time.monotonic(),
                                   state_json=msg.data,state=msg.data))
            node.create_subscription(ActuatorStates,state_topic,lambda m:motor(m,'state'),qos)
            node.create_subscription(ActuatorCmds,command_topic,lambda m:motor(m,'command'),qos)
            node.create_subscription(String,mode_topic,mode,qos)
        for sig in (signal.SIGINT,signal.SIGTERM):
            old_signals[sig]=signal.signal(sig,lambda *_:stop.set())
        if ready is not None:ready()
        while not stop.is_set() and (deadline is None or time.monotonic()<deadline) and not writer.errors:
            if node:rclpy.spin_once(node,timeout_sec=.001)
            else:stop.wait(.001)
            for s,legacy in sockets:
                for _ in range(64):
                    try:packet=s.recv(65536)
                    except BlockingIOError:break
                    writer.submit(dict(_packet=packet,_legacy=legacy,kind='trace' if legacy else 'tap',
                        received_monotonic=time.monotonic(),received_wall_ns=time.time_ns(),
                        ros_now_ns=node.get_clock().now().nanoseconds if node else None))
    finally:
        for s,_ in sockets:s.close()
        if writer:writer.close()
        if node:node.destroy_node()
        if rclpy and rclpy.ok():rclpy.shutdown()
        for sig,handler in old_signals.items():signal.signal(sig,handler)
        capture=dict(hardware_commands_sent=0,elapsed_s=time.monotonic()-started,
            requested_duration_s=seconds,
            stop_reason=('setup_failed' if writer is None else 'writer_error' if writer.errors else
                         'signal' if stop.is_set() else 'duration_reached'),
            accepted=dict(writer.accepted) if writer else {},queue_lost=dict(writer.dropped) if writer else {},
            written=writer.written if writer else 0,writer_errors=writer.errors if writer else ['setup failed'],
            callback_errors=dict(callback_errors),enable_state='only observed; no enable command sent')
        dump(out/'capture_result.json',capture)
    return capture
