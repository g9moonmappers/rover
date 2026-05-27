"""
Evaluer en trent maskinlæringsmodell på testdata.

Standard input:
- ml/models/random_forest.joblib
- ml/models/label_encoder.joblib
- ml/datasets/processed/test.csv
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import List


IKKE_FEATURE_KOLONNER = {
    "sample_id",
    "label_material",
    "label_object",
    "sand_type",
    "lysforhold",
    "avstand_cm",
    "position_id",
    "angle_id",
    "triad_file",
    "triad_file_resolved",
    "diameter_mm",
    "size_group",
    "surface_condition",
    "buried_level",
    "run_id",
}


def lag_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Evaluer lagret Random Forest-modell.")
    parser.add_argument(
        "--model_path",
        type=str,
        default="ml/models/random_forest.joblib",
        help="Sti til lagret modell.",
    )
    parser.add_argument(
        "--encoder_path",
        type=str,
        default="ml/models/label_encoder.joblib",
        help="Sti til lagret label encoder.",
    )
    parser.add_argument(
        "--input",
        type=str,
        default="ml/datasets/processed/test.csv",
        help="CSV-fil med test-features.",
    )
    parser.add_argument(
        "--label-column",
        type=str,
        default="label_object",
        help="Kolonnen som inneholder fasit/label.",
    )
    parser.add_argument(
        "--cm-out",
        type=str,
        default="ml/datasets/processed/confusion_matrix.png",
        help="Hvor confusion matrix-bilde skal lagres hvis matplotlib finnes.",
    )
    return parser


def sjekk_at_fil_finnes(sti: Path, melding: str) -> bool:
    if sti.exists():
        return True

    print(f"Fant ikke fil: {sti}")
    print(melding)
    return False


def finn_feature_kolonner(data, label_kolonne: str, pd) -> List[str]:
    feature_kolonner: List[str] = []

    for kolonne in data.columns:
        if kolonne == label_kolonne:
            continue
        if kolonne in IKKE_FEATURE_KOLONNER:
            continue
        if pd.api.types.is_numeric_dtype(data[kolonne]):
            feature_kolonner.append(kolonne)

    return feature_kolonner


def lag_confusion_matrix_bilde(cm, output_sti: Path) -> None:
    try:
        import matplotlib.pyplot as plt  # type: ignore
    except Exception:
        print("matplotlib er ikke tilgjengelig. Hopper over PNG-output.")
        return

    try:
        figur = plt.figure(figsize=(6, 6))
        akse = figur.add_subplot(1, 1, 1)
        akse.imshow(cm, interpolation="nearest")
        akse.set_title("Confusion matrix")
        akse.set_xlabel("Predicted")
        akse.set_ylabel("True")
        plt.tight_layout()

        output_sti.parent.mkdir(parents=True, exist_ok=True)
        figur.savefig(output_sti, dpi=150)
        plt.close(figur)
        print(f"Lagret confusion matrix PNG: {output_sti}")
    except Exception:
        print("Klarte ikke å lagre confusion matrix-bilde. Hopper over PNG-output.")


def main() -> int:
    parser = lag_parser()
    args = parser.parse_args()

    modell_sti = Path(args.model_path)
    encoder_sti = Path(args.encoder_path)
    input_sti = Path(args.input)

    if not sjekk_at_fil_finnes(modell_sti, "Tren og lagre modellen først."):
        return 0
    if not sjekk_at_fil_finnes(encoder_sti, "Tren og lagre label encoder først."):
        return 0
    if not sjekk_at_fil_finnes(input_sti, "Lag train/val/test-split først."):
        return 0

    try:
        import joblib  # type: ignore
    except Exception:
        print("joblib mangler. Installer avhengigheter i ML-miljøet.")
        return 0

    try:
        import pandas as pd  # type: ignore
        from sklearn.metrics import classification_report, confusion_matrix  # type: ignore
    except Exception as feil:
        print("Mangler avhengigheter for evaluering. Installer pandas og scikit-learn.")
        print(f"Importfeil: {feil}")
        return 0

    modell = joblib.load(modell_sti)
    encoder = joblib.load(encoder_sti)
    data = pd.read_csv(input_sti)

    if args.label_column not in data.columns:
        raise ValueError(f"Mangler label-kolonne: {args.label_column!r}")

    data = data.dropna(subset=[args.label_column]).copy()

    feature_kolonner = finn_feature_kolonner(data, args.label_column, pd)
    if not feature_kolonner:
        raise ValueError("Fant ingen numeriske feature-kolonner i input CSV.")

    x_test = data[feature_kolonner].fillna(0.0)
    y_fasit_tekst = data[args.label_column].astype(str)

    try:
        y_fasit = encoder.transform(y_fasit_tekst)
    except Exception:
        y_fasit = y_fasit_tekst

    y_predikert = modell.predict(x_test)

    print("Confusion matrix:")
    cm = confusion_matrix(y_fasit, y_predikert)
    print(cm)
    print()

    print("Classification report:")
    try:
        klassenavn = list(getattr(encoder, "classes_", []))
        if klassenavn:
            print(classification_report(y_fasit, y_predikert, target_names=klassenavn))
        else:
            print(classification_report(y_fasit, y_predikert))
    except Exception:
        print(classification_report(y_fasit, y_predikert))

    lag_confusion_matrix_bilde(cm, Path(args.cm_out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
