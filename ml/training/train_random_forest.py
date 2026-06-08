"""
Trener Random Forest på train.csv etter split_dataset.py.

Treningsdata holdes adskilt fra val/test for å unngå datalekkasje:
- Trener kun på train.csv
- Rapporterer val-metrikker på val.csv
- test.csv brukes kun via evaluate_model.py
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Tuple

import pandas as pd


NON_FEATURE_COLUMNS_DEFAULT = {
    "sample_id",
    "label_material",
    "label_object",
    "sand_type",
    "lysforhold",
    "avstand_cm",
    "position_id",
    "angle_id",
    "run_id",
    "triad_file",
    "triad_file_resolved",
    "diameter_mm",
    "size_group",
    "surface_condition",
    "buried_level",
}

DEFAULT_TRAIN_CSV = Path("ml/datasets/processed/train.csv")
DEFAULT_VAL_CSV = Path("ml/datasets/processed/val.csv")
FULL_DATASET_NAME = "triad_features.csv"


def _try_import_joblib() -> Tuple[object | None, str | None]:
    """Importerer joblib bare når scriptet faktisk skal lagre modellfiler."""
    try:
        import joblib

        return joblib, None
    except Exception as exc:
        return None, str(exc)


def _select_feature_columns(df, *, label_column: str) -> list[str]:
    """Velger numeriske feature-kolonner fra den prosesserte Triad CSV."""
    cols: list[str] = []
    for c in df.columns:
        if c == label_column:
            continue
        if c in NON_FEATURE_COLUMNS_DEFAULT:
            continue
        cols.append(c)

    return [c for c in cols if pd.api.types.is_numeric_dtype(df[c])]


def _print_split_metrics(
    split_name: str,
    y_true,
    y_pred,
    encoder,
) -> None:
    from sklearn.metrics import accuracy_score, classification_report, confusion_matrix

    acc = accuracy_score(y_true, y_pred)
    all_label_indices = list(range(len(encoder.classes_)))

    print(f"{split_name} accuracy: {acc:.4f}")
    print()
    print(f"{split_name} confusion matrix (labels are encoded indices):")
    print(confusion_matrix(y_true, y_pred, labels=all_label_indices))
    print()
    print(f"{split_name} classification report:")
    print(
        classification_report(
            y_true,
            y_pred,
            labels=all_label_indices,
            target_names=list(encoder.classes_),
            zero_division=0,
        )
    )


def train_and_evaluate(
    train_df: pd.DataFrame,
    *,
    val_df: pd.DataFrame | None = None,
    label_column: str,
    random_state: int = 42,
):
    from sklearn.ensemble import RandomForestClassifier
    from sklearn.preprocessing import LabelEncoder

    if label_column not in train_df.columns:
        raise ValueError(f"Missing label column: {label_column!r}")

    train_df = train_df.dropna(subset=[label_column]).copy()
    if train_df.empty:
        raise ValueError("No rows left in training data after dropping missing labels.")

    feature_columns = _select_feature_columns(train_df, label_column=label_column)
    if not feature_columns:
        raise ValueError(
            "No numeric feature columns found. "
            "Expected columns like mean_0..mean_35, std_0.., min_0.., max_0.., rms_*, ratio_*."
        )

    X_train = train_df[feature_columns].fillna(0.0)
    y_train_raw = train_df[label_column].astype(str)

    encoder = LabelEncoder()
    y_train = encoder.fit_transform(y_train_raw)

    model = RandomForestClassifier(
        n_estimators=300,
        random_state=random_state,
        n_jobs=-1,
        class_weight="balanced_subsample",
    )
    model.fit(X_train, y_train)
    print(f"Trained on {len(X_train)} rows from training split.")

    if val_df is None or val_df.empty:
        print("No validation split provided; skipping validation metrics.")
        return model, encoder

    if label_column not in val_df.columns:
        raise ValueError(f"Missing label column in validation data: {label_column!r}")

    val_df = val_df.dropna(subset=[label_column]).copy()
    if val_df.empty:
        print("Validation split has no labeled rows; skipping validation metrics.")
        return model, encoder

    known_labels = set(encoder.classes_)
    val_labels = val_df[label_column].astype(str)
    known_mask = val_labels.isin(known_labels)
    skipped = int((~known_mask).sum())
    if skipped:
        print(
            f"Warning: skipped {skipped} validation row(s) with labels not seen during training."
        )
    val_df = val_df[known_mask].copy()
    if val_df.empty:
        print("No validation rows left after filtering unknown labels.")
        return model, encoder

    X_val = val_df[feature_columns].fillna(0.0)
    y_val = encoder.transform(val_df[label_column].astype(str))
    y_pred = model.predict(X_val)
    print()
    _print_split_metrics("Validation", y_val, y_pred, encoder)

    return model, encoder


def _reject_full_dataset_path(path: Path) -> None:
    if path.name == FULL_DATASET_NAME:
        raise ValueError(
            f"Refusing to train on {path.name}: run split_dataset.py first and train on train.csv "
            "to avoid data leakage into val/test."
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Tren Random Forest (Triad baseline).")
    parser.add_argument(
        "--train-input",
        type=str,
        default=str(DEFAULT_TRAIN_CSV),
        help="Path to training split CSV (default: train.csv).",
    )
    parser.add_argument(
        "--val-input",
        type=str,
        default=str(DEFAULT_VAL_CSV),
        help="Path to validation split CSV for in-training metrics (default: val.csv).",
    )
    parser.add_argument(
        "--input",
        type=str,
        default=None,
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--label-column",
        type=str,
        default="label_object",
        help="Label column name (default: label_object).",
    )
    parser.add_argument(
        "--model-output",
        type=str,
        default="ml/models/random_forest.joblib",
        help="Where to write the trained model artifact (.joblib).",
    )
    parser.add_argument(
        "--encoder-output",
        type=str,
        default="ml/models/label_encoder.joblib",
        help="Where to write the label encoder artifact (.joblib).",
    )
    args = parser.parse_args()

    train_path = Path(args.train_input if args.input is None else args.input)
    val_path = Path(args.val_input)

    if args.input is not None:
        print("Warning: --input is deprecated; use --train-input instead.")

    try:
        _reject_full_dataset_path(train_path)
    except ValueError as exc:
        print(str(exc))
        return 2

    if not train_path.exists():
        print(f"Training CSV not found: {train_path}")
        print("Hint: run extract_triad_features.py and split_dataset.py first.")
        return 2

    joblib, joblib_err = _try_import_joblib()
    if joblib is None:
        print("joblib is not available. Install it in your ML environment to save models.")
        if joblib_err:
            print(f"Import error: {joblib_err}")
        return 2

    try:
        train_df = pd.read_csv(train_path)
    except Exception as exc:  # noqa: BLE001
        print(f"[train_random_forest] Could not read training CSV: {exc}")
        return 2

    val_df: pd.DataFrame | None = None
    if val_path.exists():
        try:
            val_df = pd.read_csv(val_path)
        except Exception as exc:  # noqa: BLE001
            print(f"[train_random_forest] Could not read validation CSV: {exc}")
            return 2
    else:
        print(f"Validation CSV not found: {val_path}")
        print("Continuing without validation metrics.")

    try:
        model, encoder = train_and_evaluate(
            train_df,
            val_df=val_df,
            label_column=args.label_column,
        )
    except Exception as exc:  # noqa: BLE001
        print("[train_random_forest] ERROR while training. Likely missing scikit-learn or invalid data.")
        print(str(exc))
        return 2

    model_out = Path(args.model_output)
    model_out.parent.mkdir(parents=True, exist_ok=True)
    joblib.dump(model, model_out)  # type: ignore[union-attr]
    print(f"Saved model: {model_out}")

    enc_out = Path(args.encoder_output)
    enc_out.parent.mkdir(parents=True, exist_ok=True)
    joblib.dump(encoder, enc_out)  # type: ignore[union-attr]
    print(f"Saved label encoder: {enc_out}")
    print("Evaluate on held-out test data with: python ml/training/evaluate_model.py")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
