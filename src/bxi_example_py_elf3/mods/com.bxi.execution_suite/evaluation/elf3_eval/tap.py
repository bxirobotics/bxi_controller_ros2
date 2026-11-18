"""Optional Unix datagrams. No listener -> no event construction/serialization."""
import json
import socket
import threading
import time

class Tap:
    def __init__(self,address='elf3_model_eval_v1'):
        self.address='\0'+address
        self.socket=None;self.retry_at=0.;self.sent=0;self.dropped=0;self.errors=0
        self.max_overhead_ns=0;self.lock=threading.Lock()

    def capture(self,factory):
        start=time.monotonic_ns()
        if not self.lock.acquire(blocking=False):
            self.dropped+=1;return False
        try:
            if self.socket is None:
                if time.monotonic()<self.retry_at:return False
                self.retry_at=time.monotonic()+1.
                sock=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM);sock.setblocking(False)
                try:sock.connect(self.address)
                except OSError:sock.close();return False
                self.socket=sock
            event=factory()
            event.update(tap_sent=self.sent,tap_dropped=self.dropped,tap_errors=self.errors,
                         tap_previous_max_overhead_ns=self.max_overhead_ns)
            data=json.dumps(event,separators=(',',':'),allow_nan=False).encode()
            if len(data)>60000:raise ValueError('observation packet exceeds 60kB')
            self.socket.send(data);self.sent+=1;return True
        except (BlockingIOError,InterruptedError):self.dropped+=1
        except OSError:
            self.dropped+=1;self.close();self.retry_at=time.monotonic()+1.
        except Exception:self.errors+=1
        finally:
            self.max_overhead_ns=max(self.max_overhead_ns,time.monotonic_ns()-start)
            self.lock.release()
        return False

    def close(self):
        if self.socket is not None:self.socket.close();self.socket=None
