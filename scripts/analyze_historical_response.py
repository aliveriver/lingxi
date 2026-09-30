"""Read local historical traces and exclusively create a descriptive report."""
import argparse
import json
from pathlib import Path

from lingxi_x2.response_diagnostic import analyze_files


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('traces', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; historical evidence must be preserved')
    report = analyze_files(args.traces)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    print(f'{len(report["sessions"])} separate historical sessions; no execution authorization. {args.output}')


if __name__ == '__main__':
    main()
