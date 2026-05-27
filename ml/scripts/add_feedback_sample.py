"""Kopier live Triad-CSV til raw/ og legg til metadata-rad med korrigert fasit."""

from __future__ import annotations

import argparse
import csv
import shutil
from pathlib import Path
from typing import Dict, List, Optional


STORRELSE_TIL_DIAMETER = {
    "liten": "4.5",
    "medium": "11",
}


def lag_parser() -> argparse.ArgumentParser:
    """Lager terminalargumentene for feedback-scriptet."""
    parser = argparse.ArgumentParser(description="Legg til live-måling som korrigert treningssample.")
    parser.add_argument("--raw", required=True, help="Live CSV-fil, f.eks. ml/datasets/live/S9999_triad_raw.csv.")
    parser.add_argument("--correct-size", required=True, choices=["liten", "medium"], help="Riktig størrelse/fasit.")
    parser.add_argument("--sample-id", default=None, help="Ny sample-id, f.eks. S0054. Hvis tom velges neste ledige.")
    parser.add_argument("--metadata", default="ml/datasets/metadata.csv", help="Sti til metadata.csv.")
    parser.add_argument("--raw-dir", default="ml/datasets/raw", help="Mappen der ny raw-fil skal lagres.")
    parser.add_argument("--max-rows", type=int, default=11, help="Antall datarader som lagres. 11 matcher dagens trening.")
    parser.add_argument("--label-object", default="stål_kule")
    parser.add_argument("--label-material", default="stål")
    parser.add_argument("--sand-type", default="torr_sand")
    parser.add_argument("--lysforhold", default="lampelys")
    parser.add_argument("--avstand-cm", default="12")
    parser.add_argument("--position-id", default="")
    parser.add_argument("--angle-id", default="A30")
    parser.add_argument("--run-id", default="")
    parser.add_argument("--surface-condition", default="synlig")
    parser.add_argument("--buried-level", default="0")
    parser.add_argument("--notes", default="", help="Valgfri kommentar. Brukes bare hvis metadata har notes-kolonne.")
    parser.add_argument("--overwrite", action="store_true", help="Tillat overskriving av raw-fil hvis den finnes.")
    return parser


def les_metadata(metadata_sti: Path) -> tuple[List[str], List[Dict[str, str]]]:
    """Leser metadata.csv og returnerer header + rader."""
    if not metadata_sti.exists():
        raise FileNotFoundError(f"Fant ikke metadata-fil: {metadata_sti}")

    with metadata_sti.open("r", newline="", encoding="utf-8") as fil:
        reader = csv.DictReader(fil)
        if reader.fieldnames is None:
            raise ValueError("metadata.csv mangler header-rad.")
        return list(reader.fieldnames), list(reader)


def finn_neste_sample_id(rader: List[Dict[str, str]]) -> str:
    """Finner neste ledige sample-id basert på største S-nummer i metadata."""
    storste_nummer = 0
    for rad in rader:
        sample_id = rad.get("sample_id", "").strip()
        if len(sample_id) >= 2 and sample_id[0].upper() == "S" and sample_id[1:].isdigit():
            storste_nummer = max(storste_nummer, int(sample_id[1:]))
    return f"S{storste_nummer + 1:04d}"


def sjekk_sample_id(sample_id: str) -> None:
    """Enkel sjekk så sample-id ligner på de andre i datasettet."""
    if len(sample_id) != 5 or not sample_id.startswith("S") or not sample_id[1:].isdigit():
        raise ValueError("sample-id må se slik ut: S0054")


def kopier_raw_fil(kilde: Path, maal: Path, max_rows: int, overwrite: bool) -> int:
    """
    Kopierer live CSV til raw-mappen.

    Hvis max_rows er satt, lagres header + de første N dataradene. Dette gjør at
    feedback-data får samme lengde som treningsdataene vi allerede har.
    """
    if not kilde.exists():
        raise FileNotFoundError(f"Fant ikke live-fil: {kilde}")
    if maal.exists() and not overwrite:
        raise FileExistsError(f"Raw-fil finnes allerede: {maal}. Bruk --overwrite hvis dette er riktig.")

    maal.parent.mkdir(parents=True, exist_ok=True)

    if max_rows <= 0:
        shutil.copyfile(kilde, maal)
        with maal.open("r", newline="", encoding="utf-8") as fil:
            return max(0, sum(1 for _ in fil) - 1)

    with kilde.open("r", newline="", encoding="utf-8") as inn, maal.open("w", newline="", encoding="utf-8") as ut:
        reader = csv.reader(inn)
        writer = csv.writer(ut)

        try:
            header = next(reader)
        except StopIteration as exc:
            raise ValueError(f"Live-filen er tom: {kilde}") from exc

        writer.writerow(header)
        antall = 0
        for rad in reader:
            if antall >= max_rows:
                break
            writer.writerow(rad)
            antall += 1
    return antall


def lag_metadata_rad(args: argparse.Namespace, sample_id: str, triad_fil: str, header: List[str]) -> Dict[str, str]:
    """Lager en metadata-rad som passer til kolonnene i metadata.csv."""
    diameter = STORRELSE_TIL_DIAMETER[args.correct_size]
    run_id = args.run_id or f"R{sample_id[1:]}"

    verdier = {
        "sample_id": sample_id,
        "label_object": args.label_object,
        "label_material": args.label_material,
        "triad_file": triad_fil,
        "sand_type": args.sand_type,
        "lysforhold": args.lysforhold,
        "avstand_cm": str(args.avstand_cm),
        "position_id": args.position_id,
        "angle_id": args.angle_id,
        "run_id": run_id,
        "diameter_mm": diameter,
        "size_group": args.correct_size,
        "surface_condition": args.surface_condition,
        "buried_level": str(args.buried_level),
        "notes": args.notes,
    }
    return {kolonne: verdier.get(kolonne, "") for kolonne in header}


def skriv_metadata(metadata_sti: Path, header: List[str], rader: List[Dict[str, str]]) -> None:
    """Skriver metadata tilbake til CSV."""
    with metadata_sti.open("w", newline="", encoding="utf-8") as fil:
        writer = csv.DictWriter(fil, fieldnames=header)
        writer.writeheader()
        for rad in rader:
            writer.writerow({kolonne: rad.get(kolonne, "") for kolonne in header})


def main(argv: Optional[List[str]] = None) -> int:
    parser = lag_parser()
    args = parser.parse_args(argv)

    metadata_sti = Path(args.metadata)
    raw_dir = Path(args.raw_dir)
    live_fil = Path(args.raw)

    try:
        header, rader = les_metadata(metadata_sti)
        sample_id = args.sample_id or finn_neste_sample_id(rader)
        sjekk_sample_id(sample_id)

        eksisterende = {rad.get("sample_id", "").strip() for rad in rader}
        if sample_id in eksisterende:
            raise ValueError(f"sample_id finnes allerede i metadata.csv: {sample_id}")

        raw_filnavn = f"{sample_id}_triad_raw.csv"
        raw_maal = raw_dir / raw_filnavn
        antall_rader = kopier_raw_fil(live_fil, raw_maal, args.max_rows, args.overwrite)

        triad_fil = f"raw/{raw_filnavn}"
        ny_rad = lag_metadata_rad(args, sample_id, triad_fil, header)
        rader.append(ny_rad)
        skriv_metadata(metadata_sti, header, rader)
    except Exception as exc:  
        print(f"[add_feedback_sample] FEIL: {exc}")
        return 2

    print(f"La til feedback-sample: {sample_id}")
    print(f"Riktig fasit: {args.correct_size}")
    print(f"Raw-fil: {raw_maal}")
    print(f"Datarader lagret: {antall_rader}")
    print()
    print("Tren modellen på nytt med:")
    print(
        "python ml/training/extract_triad_features.py "
        "--metadata ml/datasets/metadata.csv --raw-dir ml/datasets"
    )
    print("python ml/training/split_dataset.py")
    print(
        "python ml/training/train_random_forest.py --input ml/datasets/processed/train.csv "
        "--label-column size_group --model-output ml/models/random_forest_size.joblib "
        "--encoder-output ml/models/label_encoder_size.joblib --train-all"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
