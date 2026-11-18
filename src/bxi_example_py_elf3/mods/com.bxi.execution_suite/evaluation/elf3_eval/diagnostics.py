"""Summarize recorded diagnostics without changing scoring or robot control."""
from collections import Counter
import math
from numbers import Real


class TemperatureSummary:
    def __init__(self):
        self.fields = {'motor_temperature': {}, 'driver_temperature': {}}
        self.invalid_arrays = Counter()

    def add(self, event):
        if event.get('kind') != 'actual':return
        names = event.get('temperature_names', event.get('names', ()))
        for field, rows in self.fields.items():
            values = event.get(field)
            if values is None:continue
            if len(names) != len(set(names)) or len(values) != len(names):
                self.invalid_arrays[field] += 1
                continue
            for name, value in zip(names, values):
                row = rows.setdefault(name, dict(samples=0, invalid=0, minimum_c=None, maximum_c=None))
                if not isinstance(value, Real) or isinstance(value, bool) or not math.isfinite(value):
                    row['invalid'] += 1
                    continue
                value=float(value)
                row['samples'] += 1
                row['minimum_c'] = value if row['minimum_c'] is None else min(value, row['minimum_c'])
                row['maximum_c'] = value if row['maximum_c'] is None else max(value, row['maximum_c'])

    def report(self, source):
        topic = source.get('metadata.json', {}).get('topics', {}).get('state', '')
        simulated = 'simulation' in topic.strip('/').split('/')[:-1]
        observed = any(r['samples'] for rows in self.fields.values() for r in rows.values())
        return dict(status='not_assessed_simulation' if simulated else 'observed' if observed else 'unobserved',
                    unit='degC', by_joint=self.fields, invalid_arrays=dict(self.invalid_arrays),
                    note='Motor protection thresholds come from robot motor configuration; driver thresholds are not inferred.')


def summarize_events(events):
    # Older recordings contain lifecycle events without severity or fault code.
    rows = []
    for event in events:
        if event.get('status') not in ('failed', 'cancelled', 'interrupted', 'not_assessed', 'warning'):
            continue
        row = dict(event)
        row.setdefault('severity', 'error' if row['status']=='failed' else 'warning' if row['status'] in ('cancelled','interrupted','warning') else 'info')
        row.setdefault('category', 'operator' if row['status'] in ('cancelled','interrupted') else 'unclassified')
        rows.append(row)
    return dict(events=rows, by_severity=dict(Counter(r['severity'] for r in rows)),
                by_category=dict(Counter(r.get('category') or 'unclassified' for r in rows)))


class FrameworkDiagnostics:
    """Observe official status snapshots. Attribute timing only within the same model state."""
    def __init__(self):
        self.previous = None

    def update(self, raw, event):
        state = event['state']
        timing = raw.get('control_timing') or {}
        previous = self.previous
        self.previous = (state, timing)
        if not previous or not previous[0].startswith('com.bxi.execution_suite/'):
            return []
        common = dict(schema=1, kind='test_event', t=event['t'],
                      received_monotonic=event['received_monotonic'], state=previous[0],
                      source='framework_state_machine_info')
        if state == 'com.bxi.basic_actions/imu_protection':
            return [dict(common, status='warning', severity='error', code='IMU_PROTECTION',
                         category='sensor', reason='controller entered locked IMU protection')]
        if state != previous[0] or timing.get('state') != state or previous[1].get('state') != state:
            return []
        deltas = {k: timing[k] - previous[1][k]
                  for k in ('total_budget_overruns', 'total_deadline_misses', 'total_skipped_periods')
                  if k in timing and k in previous[1]}
        if any(d < 0 for d in deltas.values()) or not any(deltas.values()):
            return []
        return [dict(common, status='warning', severity='warning', category='timing', code='CONTROL_TIMING',
                     reason='official scheduler counters increased', details=deltas)]
