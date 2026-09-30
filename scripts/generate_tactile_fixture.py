"""Create a clearly labelled synthetic tactile recording, exclusively and offline."""
import argparse
import json
from pathlib import Path
from lingxi_x2.tactile_fixture import write_synthetic_recording


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True, type=Path)
    args = p.parse_args()
    print(json.dumps(write_synthetic_recording(args.output)))
