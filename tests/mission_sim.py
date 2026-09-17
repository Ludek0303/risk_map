"""Reconstruct the waypoints ArduCopter actually visits from a mission's
`items` list, following MAV_CMD_DO_JUMP (177) the way the autopilot executes
it. Repeated laps are flown via a jump back to an earlier row rather than
being written out again, so tests that need the real flight path (as opposed
to the physical file rows) replay it with this instead of reading `items`
directly.
"""


def flown_items(items, commands=(16, 21, 22)):
    """Items actually executed, in execution order. `commands=None` returns
    everything except the DO_JUMP items themselves (pure control flow),
    which is useful for replaying interleaved speed-change (178) commands."""
    by_row = {index + 1: item for index, item in enumerate(items)}  # row 0 is home, prepended by the exporter
    row = 1
    jump_counters = {}
    visited = []
    while row <= len(items):
        item = by_row[row]
        if item['command'] == 177:
            target, repeat = int(item['params'][0]), int(item['params'][1])
            remaining = jump_counters.setdefault(row, repeat)
            if remaining > 0:
                jump_counters[row] -= 1
                row = target
                continue
            row += 1
            continue
        if commands is None or item['command'] in commands:
            visited.append(item)
        row += 1
    return visited
