"""Produce per-run and aggregate CSV metrics from recorded experiments."""

import argparse
from pathlib import Path

from experiment.metrics import analyze


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    args = parser.parse_args()
    rows = analyze(args.path)
    print(f"{len(rows)} runs; {sum(row['valid'] for row in rows)} valid. CSV files in {args.path}")
