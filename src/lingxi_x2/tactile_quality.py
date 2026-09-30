"""Raw O10 display/replay quality, never calibrated contact or control approval."""
import math

FINGERS = ('thumb', 'index', 'middle', 'ring', 'little')
SURFACES = {'palm': (5, 5), 'back_of_hand': (6, 6), **{f: (4, 4) for f in FINGERS}}
STALE_AFTER_S = .5  # Display heuristic, not a manufacturer sensor deadline.


def validate_frame(frame, side):
    if not isinstance(frame, dict) or frame.get('side') != side or frame.get('unit') != 'raw_uint8':
        raise ValueError('Invalid tactile side/unit')
    fingers = frame.get('fingertips')
    if not isinstance(fingers, dict) or set(fingers) != set(FINGERS):
        raise ValueError('Expected exactly five named fingertips')
    surfaces = [frame.get('palm'), frame.get('back_of_hand'), *[fingers[f] for f in FINGERS]]
    for surface, (name, shape) in zip(surfaces, SURFACES.items()):
        if (not isinstance(surface, dict) or surface.get('name') != name
                or not isinstance(surface.get('shape'), (list, tuple))
                or any(type(v) is not int for v in surface['shape']) or tuple(surface['shape']) != shape):
            raise ValueError(f'Invalid tactile layout: {name}')
        values = surface.get('values')
        if (not isinstance(values, (list, tuple)) or len(values) != math.prod(shape)
                or any(type(v) is not int or not 0 <= v <= 255 for v in values)):
            raise ValueError(f'Expected raw uint8 integer cells: {name}')
    stamp = frame.get('timestamp')
    if (not isinstance(stamp, dict) or any(type(stamp.get(k)) is not int for k in ('sec', 'nanosec', 'monotonic_ns'))
            or stamp['sec'] < 0 or not 0 <= stamp['nanosec'] < 10**9 or stamp['monotonic_ns'] < 0):
        raise ValueError('Invalid tactile timestamp')
    if not isinstance(frame.get('source'), str):
        raise ValueError('Missing tactile source')
    return surfaces


def review_tactile(frames, reference_ns):
    if type(reference_ns) is not int or reference_ns < 0:
        raise ValueError('Invalid reference clock')
    if not isinstance(frames, dict) or not set(frames) <= {'left', 'right'}:
        raise ValueError('Expected left/right tactile mapping')
    result = {'unit': 'raw_uint8', 'reference_monotonic_ns': reference_ns,
              'stale_after_s': STALE_AFTER_S, 'sides': {},
              'tactile_contact_verified': False, 'physical_saturation_verified': False,
              'physical_orientation_verified': False}
    for side in ('left', 'right'):
        frame = frames.get(side)
        row = {'status': 'missing', 'age_s': None, 'source': None, 'synthetic': False,
               'cell_count': None, 'nonzero_cells': None, 'peak_raw_uint8': None, 'raw_ceiling_cells': None}
        result['sides'][side] = row
        if side not in frames:
            continue
        try:
            surfaces = validate_frame(frame, side)
        except ValueError as exc:
            row.update(status='invalid', error=str(exc))
            continue
        values = [v for surface in surfaces for v in surface['values']]
        age = (reference_ns-frame['timestamp']['monotonic_ns'])/1e9
        row.update(status='time_error' if age < 0 else 'stale' if age > STALE_AFTER_S else 'fresh',
                   age_s=age, source=frame['source'], synthetic=frame['source'].startswith(('mock/', 'synthetic/')),
                   cell_count=len(values), nonzero_cells=sum(v != 0 for v in values),
                   peak_raw_uint8=max(values), raw_ceiling_cells=sum(v == 255 for v in values))
    return result
