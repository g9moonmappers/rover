"""
Deler triad_features.csv i train/val/test filer.
"""

from __future__ import annotations

import argparse
import csv
import random
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple



def _read_csv_rows(path: Path) -> Tuple[List[str], List[Dict[str, str]]]:
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"CSV has no header row: {path}")
        rows = [dict(r) for r in reader]
    return list(reader.fieldnames), rows


def _write_csv_rows(path: Path, fieldnames: Sequence[str], rows: Sequence[Dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(fieldnames))
        writer.writeheader()
        writer.writerows(rows)


def _stratified_split_indices(
    labels: Sequence[str],
    *,
    train_frac: float,
    val_frac: float,
    test_frac: float,
    rng: random.Random,
) -> Tuple[List[int], List[int], List[int]]:

    if abs((train_frac + val_frac + test_frac) - 1.0) > 1e-6:
        raise ValueError("train_frac + val_frac + test_frac must sum to 1.0")

    by_label: Dict[str, List[int]] = {}
    for idx, lab in enumerate(labels):
        by_label.setdefault(lab, []).append(idx)

    train_idx: List[int] = []
    val_idx: List[int] = []
    test_idx: List[int] = []

    for lab, idxs in by_label.items():
        rng.shuffle(idxs)
        n = len(idxs)
        n_train = max(1, int(round(n * train_frac))) if n >= 3 else max(1, int(n * train_frac))
        n_remaining = n - n_train
        n_val = int(round(n_remaining * (val_frac / (val_frac + test_frac)))) if (val_frac + test_frac) > 0 else 0
        n_val = max(0, min(n_remaining, n_val))
        n_test = n_remaining - n_val

        train_idx.extend(idxs[:n_train])
        val_idx.extend(idxs[n_train : n_train + n_val])
        test_idx.extend(idxs[n_train + n_val : n_train + n_val + n_test])

    rng.shuffle(train_idx)
    rng.shuffle(val_idx)
    rng.shuffle(test_idx)
    return train_idx, val_idx, test_idx


def _random_split_indices(
    n: int,
    *,
    train_frac: float,
    val_frac: float,
    test_frac: float,
    rng: random.Random,
) -> Tuple[List[int], List[int], List[int]]:
    if abs((train_frac + val_frac + test_frac) - 1.0) > 1e-6:
        raise ValueError("train_frac + val_frac + test_frac must sum to 1.0")
    idxs = list(range(n))
    rng.shuffle(idxs)
    n_train = int(round(n * train_frac))
    n_val = int(round(n * val_frac))
    n_test = n - n_train - n_val
    return idxs[:n_train], idxs[n_train : n_train + n_val], idxs[n_train + n_val : n_train + n_val + n_test]


def main() -> int:
    parser = argparse.ArgumentParser(description="Splitt processed datasett til train/val/test CSV.")
    parser.add_argument(
        "--input",
        type=str,
        default="ml/datasets/processed/triad_features.csv",
        help="Input processed feature CSV.",
    )
    parser.add_argument(
        "--out-dir",
        type=str,
        default="ml/datasets/processed",
        help="Output directory for train/val/test CSVs.",
    )
    parser.add_argument(
        "--label-column",
        type=str,
        default="label_object",
        help="Optional label column for stratification.",
    )
    parser.add_argument("--seed", type=int, default=42, help="Random seed.")
    parser.add_argument("--train-frac", type=float, default=0.7, help="Train fraction (default 0.7).")
    parser.add_argument("--val-frac", type=float, default=0.15, help="Val fraction (default 0.15).")
    parser.add_argument("--test-frac", type=float, default=0.15, help="Test fraction (default 0.15).")
    args = parser.parse_args()

    input_path = Path(args.input)
    if not input_path.exists():
        print(f"Input CSV not found: {input_path}")
        print("Hint: generate synthetic data and run extract_triad_features.py first.")
        return 0

    fieldnames, rows = _read_csv_rows(input_path)
    if not rows:
        print(f"No rows in input CSV: {input_path}")
        return 0

    rng = random.Random(int(args.seed))

    label_column: Optional[str] = args.label_column if args.label_column in fieldnames else None
    labels: List[str] = []
    can_stratify = False
    if label_column is not None:
        labels = [r.get(label_column, "") for r in rows]
        distinct = {l for l in labels if l}
        can_stratify = len(distinct) >= 2

    if label_column is None or not can_stratify:
        if label_column is None:
            print("Label column not found. Splitting without stratify.")
        else:
            print("Label column has <2 classes (or missing). Splitting without stratify.")
        train_idx, val_idx, test_idx = _random_split_indices(
            len(rows),
            train_frac=float(args.train_frac),
            val_frac=float(args.val_frac),
            test_frac=float(args.test_frac),
            rng=rng,
        )
    else:
        train_idx, val_idx, test_idx = _stratified_split_indices(
            labels,
            train_frac=float(args.train_frac),
            val_frac=float(args.val_frac),
            test_frac=float(args.test_frac),
            rng=rng,
        )

    train_rows = [rows[i] for i in train_idx]
    val_rows = [rows[i] for i in val_idx]
    test_rows = [rows[i] for i in test_idx]

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    train_path = out_dir / "train.csv"
    val_path = out_dir / "val.csv"
    test_path = out_dir / "test.csv"
    _write_csv_rows(train_path, fieldnames, train_rows)
    _write_csv_rows(val_path, fieldnames, val_rows)
    _write_csv_rows(test_path, fieldnames, test_rows)

    print(f"Wrote: {train_path} ({len(train_rows)})")
    print(f"Wrote: {val_path} ({len(val_rows)})")
    print(f"Wrote: {test_path} ({len(test_rows)})")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

