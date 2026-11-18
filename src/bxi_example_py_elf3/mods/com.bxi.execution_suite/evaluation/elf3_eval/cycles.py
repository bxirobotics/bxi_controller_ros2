"""Match independent ROS commands to public BXI cycle boundaries in the same clock epoch."""
from bisect import bisect_left
from collections import Counter


def link_commands(events):
    commands = sorted((e for e in events if e['kind'] == 'command'),
                      key=lambda e: e.get('source_stamp_ns', 0))
    stamps = [e.get('source_stamp_ns', 0) for e in commands]
    for command in commands:
        command['association'] = 'mod_cycle_window'
        command['model_seq'] = None
    markers = [e for e in events if e['kind'] == 'mod_cycle']
    keys = Counter((m['session'], m['cycle']) for m in markers)
    for left, right in zip(markers, markers[1:]):
        start, end = left['ros_stamp_ns'], right['ros_stamp_ns']
        if (right['session'] != left['session'] or right['cycle'] != left['cycle'] + 1
                or keys[left['session'], left['cycle']] != 1 or keys[right['session'], right['cycle']] != 1
                or not 0 < end-start <= 3 * left['period_s'] * 1e9
                or left.get('model_seq') is None):
            continue
        first, after = bisect_left(stamps, start), bisect_left(stamps, end)
        if after != first + 1:
            continue
        command = commands[first]
        if not command.get('time_valid', True) or not command.get('sole_controller_publisher'):
            continue
        for key in ('model_id', 'model_hash', 'model_session', 'model_seq', 'state', 'phase'):
            command[key] = left[key]
