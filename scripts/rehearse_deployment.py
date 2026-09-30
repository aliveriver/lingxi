"""Validate a frozen package and rehearse in an internally created temp fixture."""
import argparse
import json
from pathlib import Path

from lingxi_x2.deployment_review import verify_bundle, inspect_target, rehearse


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--archive', type=Path, required=True)
    p.add_argument('--sidecar', type=Path, required=True)
    p.add_argument('--sha256', required=True)
    p.add_argument('--inspect-target', type=Path, help='Read only; real configs/manifest required')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists(): p.error('Output exists; refusing to overwrite evidence')
    bundle = verify_bundle(a.archive, a.sidecar, a.sha256)
    inspection = inspect_target(bundle, a.inspect_target) if a.inspect_target else None
    result = {k: v for k, v in bundle.items() if k != 'data'}
    result.update(kind='offline_deployment_rehearsal_v1', robot_connected=False,
                  deployed_to_pc2=False, execution_authorized=False,
                  target_inspection=inspection, rehearsal=rehearse(bundle))
    with a.output.open('x') as stream: json.dump(result, stream, indent=2, allow_nan=False)
    print(f'Archive verified; synthetic rehearsal complete. {a.output}')


if __name__ == '__main__': main()
