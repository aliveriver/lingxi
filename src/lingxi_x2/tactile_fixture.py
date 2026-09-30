"""Deterministic synthetic tactile sequence; no client, ROS or physical simulation."""
import json
from pathlib import Path

from .models import Observation, Side, TactileFrame, TactileSurface, Timestamp
from .recording import observation_to_dict
from .tactile_quality import FINGERS

PHASES = ('zero', 'contact_pattern', 'raw_ceiling', 'repeat_frame', 'stale_frame',
          'missing_right', 'missing_both', 'released')


def synthetic_observations():
    previous = None
    for i, phase in enumerate(PHASES):
        captured = (i+1)*1_000_000_000
        frames = {}
        for side in Side:
            if phase == 'missing_both' or (phase == 'missing_right' and side == Side.RIGHT): continue
            if phase in ('repeat_frame', 'stale_frame'):
                frames[side] = previous[side]
                continue
            stamp = Timestamp(1_700_000_000+i, 0, captured-10_000_000)
            values = [0]*25
            if phase not in ('zero', 'released'):
                values = list(range(1, 26)) if side == Side.LEFT else list(range(101, 126))
            if phase == 'raw_ceiling': values[12] = 255
            frames[side] = TactileFrame(stamp, side, TactileSurface('palm', (5, 5), tuple(values)),
                TactileSurface('back_of_hand', (6, 6), (0,)*36),
                {f: TactileSurface(f, (4, 4), tuple(64 if phase == 'contact_pattern' and f == 'index' and j == 5 else 0 for j in range(16))) for f in FINGERS},
                source='synthetic/tactile-contact-fixture-v1')
        yield Observation(captured, tactile=frames)
        if phase not in ('repeat_frame', 'stale_frame'): previous = frames


def write_synthetic_recording(output):
    metadata = {'record_type': 'metadata', 'format': 'lingxi-x2-jsonl-v1',
                'status': {'backend': 'synthetic_tactile_fixture', 'connected': False},
                'synthetic': True, 'robot_connected': False, 'hardware_validated': False,
                'execution_authorized': False, 'tactile_contact_verified': False,
                'phases_by_sample_index': list(PHASES),
                'evidence': 'SYNTHETIC software fixture; not sensor measurements or physical contact simulation'}
    with Path(output).open('x', encoding='utf-8') as stream:
        stream.write(json.dumps(metadata)+'\n')
        for observation in synthetic_observations():
            stream.write(json.dumps(observation_to_dict(observation), allow_nan=False)+'\n')
    return {'output': str(output), 'samples': len(PHASES), 'synthetic': True,
            'robot_connected': False, 'execution_authorized': False}
