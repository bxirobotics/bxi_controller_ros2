"""Streaming import to indexed SQLite; originals are never overwritten."""
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path
import sqlite3
import zlib
import numpy as np
from .adapters.legacy import convert
from .contract import BODY, clean, vector

def open_db(path, readonly=False):
    if readonly:
        return sqlite3.connect(Path(path).resolve().as_uri()+'?mode=ro', uri=True)
    return sqlite3.connect(path)

def import_recording(source, output, *, source_age_max=1., clock_jump=.02):
    if not np.isfinite([source_age_max,clock_jump]).all() or min(source_age_max,clock_jump)<=0:
        raise ValueError('source age and clock jump thresholds must be positive finite seconds')
    source, output = Path(source).resolve(), Path(output)
    if source.is_dir():
        source = source / ('events.jsonl.gz' if (source/'events.jsonl.gz').exists() else 'events.jsonl')
    if output.exists(): raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    db = open_db(output)
    db.executescript('CREATE TABLE events(id INTEGER PRIMARY KEY,epoch INTEGER,kind TEXT,t REAL,data BLOB);'
                     'CREATE TABLE metadata(key TEXT PRIMARY KEY,value TEXT);')
    counts = Counter(); digest = hashlib.sha256(); epoch = 0
    previous_rx = previous_offset = None
    last = {}; seqs = {}; boot = None; tap_counters={}
    opener = gzip.open if source.suffix == '.gz' else open
    with opener(source, 'rb') as stream, db:
        for number, line in enumerate(stream, 1):
            digest.update(line)
            try:
                raw = json.loads(line)
                e = convert(raw, source_age_max=source_age_max) if 'value' in raw else dict(raw)
                counts['input:'+str(raw.get('kind'))] += 1
                if e is None: continue
                if e.get('schema') != 1: raise ValueError('unknown schema')
                k = e['kind']; rx = float(e['received_monotonic']); t = float(e['t'])
                if not np.isfinite([rx, t]).all(): raise ValueError('nonfinite time')
                reasons = []
                offset = e.get('ros_now_ns')
                offset = offset*1e-9-rx if offset is not None else None
                if previous_rx is not None and rx < previous_rx:
                    reasons.append('receive_clock_reversed')
                if offset is not None and previous_offset is not None and abs(offset-previous_offset)>clock_jump:
                    reasons.append('ros_clock_jump')
                if boot and e.get('boot_id') and boot != e['boot_id']: reasons.append('host_restart')
                boot = e.get('boot_id', boot)
                previous_rx = rx
                if offset is not None: previous_offset = offset
                if reasons:
                    epoch += 1; last.clear(); seqs.clear()
                    counts.update(reasons)
                e['epoch'] = epoch
                if e.get('session') and 'tap_dropped' in e:
                    key=e['session']
                    values={k:e.get(k,0) for k in ('tap_sent','tap_dropped','tap_errors')}
                    if key not in tap_counters:tap_counters[key]={'first':values}
                    tap_counters[key]['last']=values
                if k in ('actual', 'command', 'model'):
                    if e.get('unit') != 'rad': raise ValueError('only physical radians are accepted')
                    q = vector(e['q'], e['names'])
                    e['missing_joints'] = int((~np.isfinite(q)).sum())
                    counts['missing_joint_values'] += e['missing_joints']
                    e['time_valid'] = bool(e.get('time_valid', True))
                    if not e['time_valid']: counts['invalid_source_time'] += 1
                    if k in last and t <= last[k]:
                        reason = 'duplicate_timestamp' if t == last[k] else 'out_of_order'
                        counts[reason+':'+k] += 1
                        e['time_valid'] = False
                        # Keep the bad event, and prevent pairing through its boundary.
                        db.execute('INSERT INTO events(epoch,kind,t,data) VALUES(?,?,?,?)',
                                   (epoch, 'barrier', rx, zlib.compress(json.dumps({'kind':'barrier','t':rx,'reason':reason}).encode())))
                    else: last[k] = t
                    if k == 'model':
                        for key in ('model_id', 'session', 'seq', 'phase', 'position_target'):
                            if key not in e: raise ValueError('model missing '+key)
                        if not e.get('model_hash'):counts['model_hash_unobserved']+=1
                        key = (e['session'], e['model_id'], e.get('model_hash'))
                        sequence = int(e['seq'])
                        if key in seqs:
                            delta = sequence-seqs[key]
                            if delta <= 0:
                                counts['duplicate_or_reversed_sequence'] += 1
                                e['time_valid'] = False
                                e['sequence_anomaly'] = True
                            elif delta > 1:
                                counts['model_sequence_missing'] += delta-1
                                e['sequence_gap'] = delta-1
                        seqs[key] = sequence
                    if k == 'command' and e.get('kp') is not None:
                        vector(e['kp'], e['names'])
                e['source_line'] = number
                db.execute('INSERT INTO events(epoch,kind,t,data) VALUES(?,?,?,?)',
                           (epoch,k,t,zlib.compress(json.dumps(clean(e),separators=(',',':'),allow_nan=False).encode(),1)))
                counts['stored:'+k] += 1
            except (ValueError, KeyError, TypeError, OverflowError) as exc:
                counts['rejected:'+str(exc)] += 1
                # Invalid events cannot leave an old target eligible across this point.
                if previous_rx is not None:
                    e = {'kind':'barrier','t':previous_rx,'reason':'invalid_event','source_line':number}
                    db.execute('INSERT INTO events(epoch,kind,t,data) VALUES(?,?,?,?)',
                               (epoch,'barrier',previous_rx,zlib.compress(json.dumps(e).encode())))
    meta = dict(schema=1, sonic_trace_fields_version=1, source=str(source), decompressed_sha256=digest.hexdigest(),
                source_bytes=source.stat().st_size, counts=dict(counts), clock_epochs=epoch+1,
                producer_cumulative_counters=tap_counters,
                import_options=dict(source_age_max_s=source_age_max, clock_jump_s=clock_jump),
                original_data='Complete source retained at source path; SQLite is a derived index, not a replacement.')
    for name in ('metadata.json','capture_result.json'):
        path = source.parent/name
        if path.exists(): meta[name] = json.loads(path.read_text())
    db.execute('INSERT INTO metadata VALUES(?,?)', ('import', json.dumps(meta)))
    db.execute('CREATE INDEX timeline ON events(epoch,t)'); db.commit(); db.close()
    return meta

def load_epoch(db, epoch):
    temperature_layouts={}
    for (data,) in db.execute('SELECT data FROM events WHERE epoch=? ORDER BY t,id', (epoch,)):
        e=json.loads(zlib.decompress(data))
        if e['kind'] in ('model','command','actual'):
            # Share names and store compact numeric arrays instead of millions of
            # repeated Python strings/floats. The source/index retains all names.
            e['q']=vector(e['q'],e['names'])
            if e.get('kp') is not None:e['kp']=vector(e['kp'],e['names'])
            e['names']=BODY
            if 'temperature_names' in e:
                names=tuple(e['temperature_names'])
                e['temperature_names']=temperature_layouts.setdefault(names,names)
                for field in ('motor_temperature','driver_temperature'):
                    if field in e:e[field]=np.asarray(e[field],dtype=float)
        yield e
