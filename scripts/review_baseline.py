"""Additional offline baseline qualification or qualified on/off pair review."""
import argparse
import json
from pathlib import Path
from lingxi_x2.baseline_review import qualify_file, compare_qualified_files


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('traces', type=Path, nargs='+', help='One trace, or off then on')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if len(a.traces) not in (1, 2): p.error('Need one trace or an off/on pair')
    if a.output.exists(): p.error('Output exists; refusing overwrite')
    result = qualify_file(a.traces[0]) if len(a.traces) == 1 else compare_qualified_files(*a.traces)
    with a.output.open('x') as stream: json.dump(result, stream, indent=2, allow_nan=False)
    accepted = result.get('qualified_for_pair', result.get('verdict') == 'paired_numeric_comparison')
    print('Qualified offline evidence' if accepted else 'Not qualified/comparable; see report')
    return 0 if accepted else 2


if __name__ == '__main__': raise SystemExit(main())
