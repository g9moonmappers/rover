"""
Fyll inn manglende standardverdier i ml/datasets/metadata.csv.
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Dict, List, Optional


DEFAULT_METADATA_PATH = "ml/datasets/metadata.csv"
UNKNOWN_MATERIAL = "ukjent"

REQUIRED_COLUMNS = {"sample_id", "label_object", "label_material"}

# Må samsvare med ml/configs/labels.yaml.
LABEL_OBJECT_TO_MATERIAL = {
    "aluminium_kule": "aluminium",
    "jern_kule": "jern",
    "stål_kule": "stål",
    "titan_kule": "titan",
}


def _parse_sample_num(sample_id: str) -> Optional[int]:
    sample_id = sample_id.strip()
    if not sample_id.startswith("S"):
        return None

    rest = sample_id[1:]
    if not rest.isdigit():
        return None

    return int(rest)


def _format_run_id(sequence_number: int) -> str:
    if sequence_number < 100:
        return f"R{sequence_number:02d}"
    return f"R{sequence_number}"


def _read_metadata_csv(path: Path) -> tuple[List[str], List[Dict[str, str]]]:
    """Les metadata.csv og kontroller at nødvendige kolonner finnes."""
    if not path.exists():
        raise SystemExit(f"Fant ikke metadata: {path}")

    with path.open("r", newline="") as file:
        reader = csv.DictReader(file)
        if not reader.fieldnames:
            raise SystemExit("metadata.csv has no header")

        fieldnames = list(reader.fieldnames)
        missing = REQUIRED_COLUMNS - set(fieldnames)
        if missing:
            raise SystemExit(f"metadata.csv missing columns: {sorted(missing)}")

        rows = [dict(row) for row in reader]

    return fieldnames, rows


def _find_first_sample_number_per_label(rows: List[Dict[str, str]]) -> Dict[str, int]:
    """
    Finn laveste sample-nummer per objektklasse.
    """
    first_sample_by_label: Dict[str, int] = {}

    for row in rows:
        sample_id = str(row.get("sample_id", "")).strip()
        label_object = str(row.get("label_object", "")).strip()
        sample_number = _parse_sample_num(sample_id)

        if not label_object or sample_number is None:
            continue

        current_min = first_sample_by_label.get(label_object)
        if current_min is None or sample_number < current_min:
            first_sample_by_label[label_object] = sample_number

    return first_sample_by_label


def _fill_label_material(row: Dict[str, str]) -> int:
    """
    Fyll label_material fra label_object når det er trygt.
    """
    label_object = str(row.get("label_object", "")).strip()
    target_material = LABEL_OBJECT_TO_MATERIAL.get(label_object)
    if target_material is None:
        return 0

    current_material = str(row.get("label_material", "")).strip()
    if current_material and current_material.lower() != UNKNOWN_MATERIAL:
        return 0

    row["label_material"] = target_material
    return 1


def _fill_run_id(row: Dict[str, str], first_sample_by_label: Dict[str, int]) -> int:
    """
    Fyll run_id hvis kolonnen finnes og verdien er tom.
    """
    if "run_id" not in row or str(row.get("run_id", "")).strip():
        return 0

    label_object = str(row.get("label_object", "")).strip()
    sample_number = _parse_sample_num(str(row.get("sample_id", "")).strip())
    first_sample_number = first_sample_by_label.get(label_object)

    if sample_number is None or first_sample_number is None:
        return 0

    sequence_number = sample_number - (first_sample_number - 1)
    row["run_id"] = _format_run_id(sequence_number)
    return 1


def fill_metadata_defaults(rows: List[Dict[str, str]]) -> int:
    """
    Fyll alle støttede standardverdier i radene.

    Returnerer antall felt som ble oppdatert.
    """
    first_sample_by_label = _find_first_sample_number_per_label(rows)
    updates = 0

    for row in rows:
        updates += _fill_label_material(row)
        updates += _fill_run_id(row, first_sample_by_label)

    return updates


def _write_backup(path: Path) -> Path:
    """Lag en enkel backup før vi endrer metadatafilen."""
    backup_path = path.with_suffix(path.suffix + ".bak")
    backup_path.write_bytes(path.read_bytes())
    return backup_path


def _write_metadata_csv(path: Path, fieldnames: List[str], rows: List[Dict[str, str]]) -> None:
    """
    Skriv metadata atomisk via en midlertidig fil for reduserer risikoen for en halvskrevet CSV ved feil.
    """
    temporary_path = path.with_suffix(path.suffix + ".tmp")

    with temporary_path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    temporary_path.replace(path)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Fyll manglende felt i metadata.csv.")
    parser.add_argument("--metadata-path", default=DEFAULT_METADATA_PATH)
    parser.add_argument("--backup", action="store_true", help="Lag metadata.csv.bak for endring.")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    metadata_path = Path(args.metadata_path)
    fieldnames, rows = _read_metadata_csv(metadata_path)
    updated_fields = fill_metadata_defaults(rows)

    if args.backup:
        backup_path = _write_backup(metadata_path)
        print(f"[fill_metadata_defaults] Backup lagret: {backup_path}")

    _write_metadata_csv(metadata_path, fieldnames, rows)

    print(f"[fill_metadata_defaults] Ferdig. Oppdaterte {updated_fields} felt i {metadata_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

