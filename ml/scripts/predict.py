"""Prediker label fra rå burst-CSV (bruker ml/models/*.joblib)."""

from __future__ import annotations

import _bootstrap_sys_path 

import argparse
from pathlib import Path


def main() -> int:
    p = argparse.ArgumentParser(description="Prediker label fra rå Triad burst-CSV (Random Forest).")
    p.add_argument("--raw", required=True, help="Path to raw burst CSV (e.g. ml/datasets/raw/S0001_triad_raw.csv).")
    p.add_argument("--model", default="ml/models/random_forest.joblib", help="Path to trained model (.joblib).")
    p.add_argument("--encoder", default="ml/models/label_encoder.joblib", help="Path to label encoder (.joblib).")
    p.add_argument(
        "--sam-refs",
        default="ml/datasets/processed/sam_reference_spectra.json",
        help="SAM class reference spectra JSON (from extract_triad_features).",
    )
    p.add_argument(
        "--max-rows",
        type=int,
        default=11,
        help="How many burst rows to use. Default is 11 because the current training data uses 11 rows.",
    )
    args = p.parse_args()

    raw_path = Path(args.raw)
    if not raw_path.exists():
        print(f"Raw CSV not found: {raw_path}")
        return 2

    try:
        import joblib  # type: ignore
        import pandas as pd  # type: ignore
    except Exception as exc:  # noqa: BLE001
        print("Missing dependencies. Install: pandas scikit-learn joblib")
        print(f"Import error: {exc}")
        return 2

    from ml.training.extract_triad_features import (  # type: ignore
        _flatten_features_for_csv,
        extract_features_from_burst,
        load_sam_reference_spectra,
        read_triad_raw_csv,
    )

    sam_path = Path(args.sam_refs)
    if not sam_path.exists():
        print(f"SAM reference file not found: {sam_path} (run extract_triad_features.py first)")
        return 2

    burst = read_triad_raw_csv(raw_path)
    if args.max_rows is not None and args.max_rows > 0 and len(burst) > args.max_rows:
        # Treningsdataene i dette datasettet har 11 rader per måling.
        # Derfor kutter vi live-målinger likt, slik at features beregnes på samme måte.
        print(f"Using first {args.max_rows} rows from {len(burst)} live rows.")
        burst = burst[: args.max_rows]
    sam_refs = load_sam_reference_spectra(sam_path)
    feats = extract_features_from_burst(burst, sam_refs=sam_refs)
    flat = _flatten_features_for_csv(feats)

    model = joblib.load(Path(args.model))
    encoder = joblib.load(Path(args.encoder))

    # sklearn husker feature_names_in_ fra trening — live-rad må matche kolonnene.
    trenings_kolonner = list(getattr(model, "feature_names_in_", []))
    if trenings_kolonner:
        X = pd.DataFrame([flat]).reindex(columns=trenings_kolonner, fill_value=0.0).fillna(0.0)
    else:
        X = pd.DataFrame([flat]).fillna(0.0)

    probs = getattr(model, "predict_proba", None)
    if callable(probs):
        pvec = model.predict_proba(X)[0]
        idx = int(pvec.argmax())
        classes = list(getattr(encoder, "classes_", []))
        label = classes[idx] if classes else str(idx)
        conf = float(pvec[idx])
        print(f"prediction={label} confidence={conf:.3f}")
        if classes:
            detaljer = ", ".join(f"{classes[i]}={float(pvec[i]):.3f}" for i in range(len(classes)))
            print(f"probabilities: {detaljer}")
        return 0

    pred = model.predict(X)[0]
    classes = list(getattr(encoder, "classes_", []))
    try:
        label = classes[int(pred)]
    except Exception:
        label = str(pred)
    print(f"prediction={label}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

