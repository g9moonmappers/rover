"""
Feature ekstraksjon for Triad (36 kanaler fra to sensorer).

Leser burst-CSV via metadata.csv, 
lager mean/std/min/max per kanal
band-ratio og SAM-features.
Skriver triad_features.csv og sam_reference_spectra.json for predict.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Dict, Iterable, Iterator, List, Mapping, Optional, Sequence, Tuple

import numpy as np


NUM_SENSORS_DEFAULT = 2
CHANNELS_PER_SENSOR_DEFAULT = 18

# Band-ratio denominators for å unngå deling på null
_RATIO_EPS = 1e-6

# SAM denominator stabilizer for å unngå deling på null
_SAM_EPS = 1e-8

# referanse spekter og SAM features.
SAM_CLASS_ORDER: Tuple[str, ...] = ("aluminium", "steel", "sand")
SAM_FEATURE_NAMES: Dict[str, str] = {
    "aluminium": "sam_to_aluminium",
    "steel": "sam_to_steel",
    "sand": "sam_to_sand",
}

WAVELENGTHS_NM: Tuple[int, ...] = (
    410,
    435,
    460,
    485,
    510,
    535,
    560,
    585,
    610,
    645,
    680,
    705,
    730,
    760,
    810,
    860,
    900,
    940,
)


def _safe_float(x: str) -> float:
    try:
        return float(x)
    except Exception as exc:  # noqa: BLE001
        raise ValueError(f"Could not parse float from: {x!r}") from exc


def _canonical_sam_class(label_material: str) -> Optional[str]:
    """
    Map metadata label_material to aluminium | steel | sand for SAM references.
    """
    s = (label_material or "").strip().lower()
    if not s:
        return None
    if "alu" in s or "alumin" in s:
        return "aluminium"
    if "steel" in s or "stål" in s or "stal" in s or "staal" in s or "stainless" in s:
        return "steel"
    if "sand" in s or "regolith" in s or "torr_sand" in s:
        return "sand"
    return None


def spectral_angle_mapper(x: Sequence[float], r: Sequence[float], *, eps: float = _SAM_EPS) -> float:
    """Spektralvinkel (rad) mellom sample og referanseklasse."""
    xa = np.asarray(x, dtype=np.float64).ravel()
    ra = np.asarray(r, dtype=np.float64).ravel()
    if xa.shape != ra.shape:
        raise ValueError(f"SAM: shape mismatch {xa.shape} vs {ra.shape}")
    nx = float(np.linalg.norm(xa))
    nr = float(np.linalg.norm(ra))
    if nx < eps or nr < eps:
        return float(0.5 * np.pi)
    cos_theta = float(np.dot(xa, ra) / (nx * nr + eps))
    cos_theta = float(np.clip(cos_theta, -1.0, 1.0))
    return float(np.arccos(cos_theta))


def build_sam_reference_spectra(
    labeled_mean36: Sequence[Tuple[str, Sequence[float]]],
) -> Dict[str, np.ndarray]:
    """
    Bygg referanse spekter for SAM features.
    """
    bins: Dict[str, List[np.ndarray]] = defaultdict(list)
    for label_material, mean_36 in labeled_mean36:
        cls = _canonical_sam_class(label_material)
        if cls is None:
            continue
        bins[cls].append(np.asarray(mean_36, dtype=np.float64).ravel())

    refs: Dict[str, np.ndarray] = {}
    for cls in SAM_CLASS_ORDER:
        if bins[cls]:
            refs[cls] = np.mean(np.stack(bins[cls], axis=0), axis=0)
        else:
            refs[cls] = np.zeros(CHANNELS_PER_SENSOR_DEFAULT * 2, dtype=np.float64)
    return refs


def compute_sam_features(
    mean_36: Sequence[float],
    refs: Mapping[str, np.ndarray],
) -> Dict[str, float]:
    """Beregn SAM vinkler for hver referanseklasse vs mean_36."""
    out: Dict[str, float] = {}
    for cls in SAM_CLASS_ORDER:
        name = SAM_FEATURE_NAMES[cls]
        r = refs.get(cls)
        if r is None:
            r = np.zeros(CHANNELS_PER_SENSOR_DEFAULT * 2, dtype=np.float64)
        out[name] = spectral_angle_mapper(mean_36, r)
    return out


def save_sam_reference_spectra(path: Path, refs: Mapping[str, np.ndarray]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {k: np.asarray(v, dtype=np.float64).tolist() for k, v in refs.items()}
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def load_sam_reference_spectra(path: Path) -> Dict[str, np.ndarray]:
    if not path.exists():
        raise FileNotFoundError(f"SAM reference file not found: {path}")
    data = json.loads(path.read_text(encoding="utf-8"))
    out: Dict[str, np.ndarray] = {}
    for cls in SAM_CLASS_ORDER:
        if cls not in data:
            raise ValueError(f"SAM refs JSON missing key {cls!r}")
        out[cls] = np.asarray(data[cls], dtype=np.float64).ravel()
        if out[cls].size != CHANNELS_PER_SENSOR_DEFAULT * 2:
            raise ValueError(f"SAM ref for {cls}: expected length 36, got {out[cls].size}")
    return out


def _expected_column_names(prefix_a: str, prefix_b: str) -> List[str]:
    cols: List[str] = []
    cols.extend([f"{prefix_a}_{wl}" for wl in WAVELENGTHS_NM])
    cols.extend([f"{prefix_b}_{wl}" for wl in WAVELENGTHS_NM])
    return cols


def _detect_schema(fieldnames: Sequence[str]) -> Tuple[str, str]:
    """
    Sjekk om rå CSV bruker S0/S1 eller L/R navn.
    """
    fields = set(fieldnames)
    s0 = {f"S0_{wl}" for wl in WAVELENGTHS_NM}
    s1 = {f"S1_{wl}" for wl in WAVELENGTHS_NM}
    l0 = {f"L_{wl}" for wl in WAVELENGTHS_NM}
    r1 = {f"R_{wl}" for wl in WAVELENGTHS_NM}

    if s0.issubset(fields) and s1.issubset(fields):
        return "S0", "S1"
    if l0.issubset(fields) and r1.issubset(fields):
        return "L", "R"


    examples = ", ".join(_expected_column_names("S0", "S1")[:4] + ["..."])
    raise ValueError(
        "Could not detect spectral column schema. "
        "Expected either S0_410..S0_940 + S1_410..S1_940, or L_410..L_940 + R_410..R_940. "
        f"Example columns: {examples}"
    )


def read_triad_raw_csv(
    path: str | Path,
    *,
    channels_per_sensor: int = CHANNELS_PER_SENSOR_DEFAULT,
) -> List[List[float]]:
    """
    Read a Triad raw burst CSV file and return a list of rows (each row has 36 floats).

    - Requires a header row.
    - Ignores extra columns.
    - Raises a clear error for missing required spectral columns.
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(str(path))

    samples: List[List[float]] = []
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"Raw CSV has no header row: {path}")

        p0, p1 = _detect_schema(reader.fieldnames)
        required = _expected_column_names(p0, p1)

        missing = [c for c in required if c not in reader.fieldnames]
        if missing:
            raise ValueError(f"Missing required spectral columns in {path}: {missing}")

        for row in reader:
            values = [_safe_float(row[c]) for c in required]
            if len(values) != 2 * channels_per_sensor:
                raise ValueError(
                    f"Expected {2 * channels_per_sensor} spectral values per row, got {len(values)} in {path}"
                )
            samples.append(values)

    if not samples:
        raise ValueError(f"No data rows found in raw CSV: {path}")

    return samples


def _split_sensors(
    sample_36: Sequence[float],
    *,
    channels_per_sensor: int = CHANNELS_PER_SENSOR_DEFAULT,
) -> Tuple[List[float], List[float]]:
    if len(sample_36) != 2 * channels_per_sensor:
        raise ValueError(
            f"Expected {2 * channels_per_sensor} values, got {len(sample_36)}"
        )
    s0 = list(sample_36[:channels_per_sensor])
    s1 = list(sample_36[channels_per_sensor : 2 * channels_per_sensor])
    return s0, s1


def _mean(xs: Sequence[float]) -> float:
    return sum(xs) / max(1, len(xs))


def _std(xs: Sequence[float]) -> float:
    if len(xs) < 2:
        return 0.0
    m = _mean(xs)
    var = sum((x - m) ** 2 for x in xs) / (len(xs) - 1)
    return math.sqrt(var)


def _rms(xs: Sequence[float]) -> float:
    if not xs:
        return 0.0
    return math.sqrt(sum(x * x for x in xs) / len(xs))


def compute_basic_features(sample_36: Sequence[float]) -> Dict[str, List[float]]:
    """
    Beregn grunnleggende statistiske features per kanal.
    """
    x = list(sample_36)
    if len(x) != 36:
        raise ValueError(f"Expected 36 channels, got {len(x)}")

    return {
        "mean_36": x,
        "std_36": [0.0] * 36,
        "min_36": x,
        "max_36": x,
    }


def compute_burst_basic_features(samples: Sequence[Sequence[float]]) -> Dict[str, List[float]]:
    """
    Beregn mean/std/min/max per kanal over flere samples (burst).
    """
    if not samples:
        raise ValueError("No samples provided")
    n = len(samples)
    if any(len(s) != 36 for s in samples):
        raise ValueError("All samples must have length 36")

    means: List[float] = []
    stds: List[float] = []
    mins: List[float] = []
    maxs: List[float] = []
    for i in range(36):
        col = [float(samples[j][i]) for j in range(n)]
        means.append(_mean(col))
        stds.append(_std(col))
        mins.append(min(col))
        maxs.append(max(col))
    return {"mean_36": means, "std_36": stds, "min_36": mins, "max_36": maxs}


def _wl_index(wavelength_nm: int) -> int:
    try:
        return WAVELENGTHS_NM.index(wavelength_nm)
    except ValueError as exc:
        raise ValueError(f"Unknown wavelength {wavelength_nm} nm for Triad layout") from exc


def compute_band_ratio_features(mean_36: Sequence[float]) -> Dict[str, float]:
    """Fem band-ratio fra mean_36 (S0: 0-17, S1: 18-35)."""
    if len(mean_36) != 36:
        raise ValueError(f"Expected 36 channels for ratios, got {len(mean_36)}")
    m = [float(x) for x in mean_36]

    i410 = _wl_index(410)
    i485 = _wl_index(485)
    i560 = _wl_index(560)
    i610 = _wl_index(610)
    i730 = _wl_index(730)
    i940 = _wl_index(940)

    def s0(k: int) -> float:
        return m[k]

    def s1(k: int) -> float:
        return m[CHANNELS_PER_SENSOR_DEFAULT + k]

    eps = _RATIO_EPS
    return {
        "ratio_S0_410_940": s0(i410) / (s0(i940) + eps),
        "ratio_S1_410_940": s1(i410) / (s1(i940) + eps),
        "ratio_S0_560_730": s0(i560) / (s0(i730) + eps),
        "ratio_S1_560_730": s1(i560) / (s1(i730) + eps),
        "ratio_S0_485_610": s0(i485) / (s0(i610) + eps),
    }


def compute_rms_features(
    sample_36: Sequence[float],
    *,
    channels_per_sensor: int = CHANNELS_PER_SENSOR_DEFAULT,
) -> Dict[str, float]:
    """
    Beregn RMS features på 36-kanal vektoren.
    """
    if len(sample_36) != 2 * channels_per_sensor:
        raise ValueError(f"Expected {2 * channels_per_sensor} channels, got {len(sample_36)}")
    s0, s1 = _split_sensors(sample_36, channels_per_sensor=channels_per_sensor)
    return {
        "rms_total": _rms(sample_36),
        "rms_sensor_0": _rms(s0),
        "rms_sensor_1": _rms(s1),
    }


def compute_derivative_features(
    mean_36: Sequence[float],
    *,
    channels_per_sensor: int = CHANNELS_PER_SENSOR_DEFAULT,
) -> Dict[str, List[float]]:
    """
    Beregn enkle spektrale derivater fra mean_36.
    """
    if len(mean_36) != 2 * channels_per_sensor:
        raise ValueError(f"Expected {2 * channels_per_sensor} channels for derivatives, got {len(mean_36)}")

    s0, s1 = _split_sensors(mean_36, channels_per_sensor=channels_per_sensor)
    derivative_s0 = [float(s0[i + 1]) - float(s0[i]) for i in range(channels_per_sensor - 1)]
    derivative_s1 = [float(s1[i + 1]) - float(s1[i]) for i in range(channels_per_sensor - 1)]
    return {
        "derivative_s0": derivative_s0,
        "derivative_s1": derivative_s1,
    }


def extract_features_from_burst(
    samples: Sequence[Sequence[float]],
    *,
    channels_per_sensor: int = CHANNELS_PER_SENSOR_DEFAULT,
    sam_refs: Optional[Mapping[str, np.ndarray]] = None,
) -> Dict[str, object]:
    """
    Ekstrakt features fra en burst.
    """
    basic = compute_burst_basic_features(samples)
    rms = compute_rms_features(basic["mean_36"], channels_per_sensor=channels_per_sensor)
    ratios = compute_band_ratio_features(basic["mean_36"])
    derivatives = compute_derivative_features(basic["mean_36"], channels_per_sensor=channels_per_sensor)
    out: Dict[str, object] = {**basic, **rms, **ratios, **derivatives}
    if sam_refs is not None:
        out.update(compute_sam_features(basic["mean_36"], sam_refs))
    return out


def _flatten_features_for_csv(features: Mapping[str, object]) -> Dict[str, float]:
    """
    Konverter interne features til en flatt dict for CSV kolonner.
    """
    out: Dict[str, float] = {}
    for key in ("mean_36", "std_36", "min_36", "max_36"):
        arr = features[key]
        if not isinstance(arr, list):
            raise TypeError(f"Expected list for {key}, got {type(arr)}")
        if len(arr) != 36:
            raise ValueError(f"Expected 36 values for {key}, got {len(arr)}")
        prefix = key.replace("_36", "")
        for i, v in enumerate(arr):
            out[f"{prefix}_{i}"] = float(v)

    for key in ("rms_total", "rms_sensor_0", "rms_sensor_1"):
        out[key] = float(features[key])  
    ratio_keys = (
        "ratio_S0_410_940",
        "ratio_S1_410_940",
        "ratio_S0_560_730",
        "ratio_S1_560_730",
        "ratio_S0_485_610",
    )
    for key in ratio_keys:
        if key not in features:
            raise KeyError(f"Missing ratio feature {key!r}")
        out[key] = float(features[key])  # type: ignore[arg-type]

    for key in ("sam_to_aluminium", "sam_to_steel", "sam_to_sand"):
        if key in features:
            out[key] = float(features[key])  # type: ignore[arg-type]

    for key, prefix in (("derivative_s0", "derivative_S0"), ("derivative_s1", "derivative_S1")):
        arr = features.get(key)
        if not isinstance(arr, list):
            raise TypeError(f"Expected list for {key}, got {type(arr)}")
        if len(arr) != CHANNELS_PER_SENSOR_DEFAULT - 1:
            raise ValueError(f"Expected {CHANNELS_PER_SENSOR_DEFAULT - 1} values for {key}, got {len(arr)}")
        for i, v in enumerate(arr):
            wl_a = WAVELENGTHS_NM[i]
            wl_b = WAVELENGTHS_NM[i + 1]
            out[f"{prefix}_{wl_a}_{wl_b}"] = float(v)

    _assert_finite_features(out)
    return out


def _assert_finite_features(flat: Mapping[str, float]) -> None:
    for k, v in flat.items():
        if math.isnan(v) or math.isinf(v):
            raise ValueError(f"Non-finite feature {k}={v}")


def _read_metadata_rows(metadata_csv: Path) -> Iterator[Dict[str, str]]:
    if not metadata_csv.exists():
        raise FileNotFoundError(str(metadata_csv))
    with metadata_csv.open("r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"Metadata CSV has no header row: {metadata_csv}")
        if "sample_id" not in reader.fieldnames:
            raise ValueError(f"Metadata CSV missing required column 'sample_id': {metadata_csv}")
        if "triad_file" not in reader.fieldnames:
            raise ValueError(f"Metadata CSV missing required column 'triad_file': {metadata_csv}")
        for row in reader:
            if not row.get("sample_id"):
                continue
            yield row 


def build_processed_table(
    *,
    metadata_csv: Path,
    raw_dir: Path,
) -> Tuple[List[Dict[str, object]], Dict[str, np.ndarray]]:
    """
    Bygg en prosessert feature tabell per metadata rad.
    """
    pending: List[Tuple[Dict[str, str], List[List[float]]]] = []
    missing_files: List[str] = []

    for meta in _read_metadata_rows(metadata_csv):
        sample_id = meta["sample_id"].strip()
        triad_rel = (meta.get("triad_file") or "").strip()
        if not triad_rel:
            raise ValueError(f"metadata row for sample_id={sample_id} has empty triad_file")

        raw_path = (raw_dir / triad_rel).resolve()
        if not raw_path.exists():
            missing_files.append(f"{sample_id}: {raw_path}")
            continue

        burst = read_triad_raw_csv(raw_path)
        pending.append((dict(meta), burst))

    if missing_files:
        joined = "\n".join(missing_files[:20])
        more = "" if len(missing_files) <= 20 else f"\n... and {len(missing_files) - 20} more"
        raise FileNotFoundError(
            "Missing raw triad files referenced by metadata:\n"
            f"{joined}{more}\n\n"
            "Fix by ensuring --raw-dir matches the triad_file paths."
        )

    if not pending:
        raise ValueError("No processed rows produced (check metadata contents and raw files).")

    labeled_means: List[Tuple[str, List[float]]] = []
    for meta, burst in pending:
        lm = (meta.get("label_material") or "").strip()
        mean_36 = compute_burst_basic_features(burst)["mean_36"]
        labeled_means.append((lm, mean_36))

    sam_refs = build_sam_reference_spectra(labeled_means)

    rows_out: List[Dict[str, object]] = []
    for meta, burst in pending:
        sample_id = meta["sample_id"].strip()
        triad_rel = (meta.get("triad_file") or "").strip()
        raw_path = (raw_dir / triad_rel).resolve()

        feats = extract_features_from_burst(burst, sam_refs=sam_refs)
        flat = _flatten_features_for_csv(feats)

        out_row: Dict[str, object] = dict(meta)
        out_row["sample_id"] = sample_id
        out_row["triad_file_resolved"] = str(raw_path)
        out_row.update(flat)
        rows_out.append(out_row)

    return rows_out, sam_refs


def write_processed_csv(rows: Sequence[Mapping[str, object]], output_csv: Path) -> None:
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    preferred_first = [
        "sample_id",
        "label_object",
        "label_material",
        "triad_file",
        "triad_file_resolved",
        "sand_type",
        "lysforhold",
        "avstand_cm",
        "position_id",
        "angle_id",
        "diameter_mm",
        "size_group",
        "surface_condition",
        "buried_level",
        "run_id",
    ]
    all_keys: List[str] = sorted({k for r in rows for k in r.keys()})
    fieldnames: List[str] = []
    for k in preferred_first:
        if k in all_keys:
            fieldnames.append(k)
    for k in all_keys:
        if k not in fieldnames:
            fieldnames.append(k)

    with output_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in rows:
            writer.writerow({k: r.get(k, "") for k in fieldnames})


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Ekstraher Triad-features og lag processed CSV.")
    parser.add_argument(
        "--metadata",
        type=str,
        default="ml/datasets/metadata.csv",
        help="Path to metadata CSV (must include sample_id and triad_file).",
    )
    parser.add_argument(
        "--raw-dir",
        type=str,
        default="ml/datasets",
        help="Base directory used to resolve triad_file paths from metadata.",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="ml/datasets/processed/triad_features.csv",
        help="Where to write processed feature CSV.",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)

    metadata_csv = Path(args.metadata)
    raw_dir = Path(args.raw_dir)
    output_csv = Path(args.output)

    try:
        rows, sam_refs = build_processed_table(metadata_csv=metadata_csv, raw_dir=raw_dir)
    except Exception as exc:  # noqa: BLE001
        print(f"[extract_triad_features] ERROR: {exc}")
        return 2

    write_processed_csv(rows, output_csv)
    sam_json = output_csv.parent / "sam_reference_spectra.json"
    save_sam_reference_spectra(sam_json, sam_refs)
    print(f"[extract_triad_features] Wrote processed features: {output_csv} (rows={len(rows)})")
    print(f"[extract_triad_features] Wrote SAM reference spectra: {sam_json}")
    if rows:
        sample_keys = rows[0].keys()
        numeric_feat = [
            k
            for k in sample_keys
            if k.startswith(("mean_", "std_", "min_", "max_", "rms_", "ratio_", "sam_to_", "derivative_"))
        ]
        ratio_preview = {
            k: float(rows[0][k])  
            for k in sorted(sample_keys)
            if str(k).startswith("ratio_")
        }
        sam_preview = {
            k: float(rows[0][k])  
            for k in sorted(sample_keys)
            if str(k).startswith("sam_to_")
        }
        print(
            f"[extract_triad_features] Numeric feature columns: {len(numeric_feat)} "
            "(36×4 + 3 RMS + 5 ratios + 3 SAM + 34 derivatives)."
        )
        print(f"[extract_triad_features] Example ratio_* (first row): {ratio_preview}")
        print(f"[extract_triad_features] Example sam_to_* (first row): {sam_preview}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

