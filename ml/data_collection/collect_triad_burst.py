"""
Samle inn en Triad BURST over serial og lagre den som rå CSV.

Scriptet samarbeider med Arduino sketch "moonmapper_triad_logger":
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple


DEFAULT_BAUD = 115200
DEFAULT_TIMEOUT_S = 5.0
DEFAULT_OUTPUT_DIR = "ml/datasets/raw"
DEFAULT_METADATA_PATH = "ml/datasets/metadata.csv"

BURST_ROWS_EXPECTED = 20
SPECTRAL_COLUMNS_EXPECTED = 36
BASE_COLUMNS_EXPECTED = 3  # sample_id, burst_index, timestamp_ms
CSV_HEADER_PREFIX = "sample_id,burst_index,timestamp_ms,"

METADATA_HEADER: List[str] = [
    "sample_id",
    "label_object",
    "label_material",
    "triad_file",
    "sand_type",
    "lysforhold",
    "avstand_cm",
    "position_id",
    "angle_id",
    "run_id",
    "diameter_mm",
    "size_group",
    "surface_condition",
    "buried_level",
]

# avhengighet for serial kommunikasjon med Arduino
def _require_pyserial():
    try:
        import serial  # type: ignore

        return serial
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError("pyserial is not installed") from exc


def _list_available_serial_ports() -> List[str]:
    _require_pyserial()

    try:
        from serial.tools import list_ports  # type: ignore
    except Exception:
        return []

    ports: List[str] = []
    for port_info in list_ports.comports():
        description = getattr(port_info, "description", "")
        device = getattr(port_info, "device", "")
        if description:
            ports.append(f"{device} - {description}")
        else:
            ports.append(str(device))
    return ports


def _print_available_serial_ports() -> None:
    ports = _list_available_serial_ports()

    if not ports:
        print("Fant ingen serial porter.")
        print("sjekk at Arduino er koblet til og at porten er riktig.")
        return

    print("Tilgjengelige serial porter:")
    for port in ports:
        print(f"  {port}")


def _readline_with_timeout(ser, *, timeout_s: float) -> Optional[str]:
    deadline = time.time() + timeout_s
    buffer = bytearray()

    while time.time() < deadline:
        chunk = ser.read(1)

        if not chunk:
            time.sleep(0.01)
            continue

        if chunk == b"\n":
            return buffer.decode("utf-8", errors="replace").strip()

        if chunk != b"\r":
            buffer.extend(chunk)

    return None


def _send_command_and_expect_ok(ser, *, command: str, timeout_s: float) -> None:
    ser.write((command.strip() + "\n").encode("utf-8"))
    ser.flush()

    response = _readline_with_timeout(ser, timeout_s=timeout_s)
    if response is None:
        raise RuntimeError(f"Timeout waiting for response to command: {command!r}")
    if response.startswith("ERROR"):
        raise RuntimeError(f"Arduino error: {response}")
    if not response.startswith("OK"):
        raise RuntimeError(f"Unexpected Arduino response to {command!r}: {response}")


def _print_debug_line(*, enabled: bool, line: str) -> None:
    if enabled:
        print(f"[collect_triad_burst][debug] RX: {line}", file=sys.stderr)


def _wait_after_background(
    *,
    wait_after_sb_s: float,
    interactive_after_background: bool,
) -> None:
    time.sleep(0.2)  # Kort stabilisering etter SB før bruker/objekt håndtering.

    if wait_after_sb_s > 0:
        print(
            f"[collect_triad_burst] Vent {wait_after_sb_s:g}s etter SB "
            "(legg objekt i sanden før skann)...",
            file=sys.stderr,
        )
        time.sleep(wait_after_sb_s)

    if interactive_after_background:
        print(
            "[collect_triad_burst] Bakgrunn tatt (SB). "
            "Legg objekt ferdig, trykk Enter for å starte BURST...",
            file=sys.stderr,
        )
        try:
            input()
        except EOFError:
            # for autonom kjøring, uten interaktiv terminal.
            pass


def _read_csv_header_from_arduino(ser, *, timeout_s: float, debug: bool) -> str:
    """
    ignorerer INFO/DEBUG linjer og les frem til Arduino sender CSV headeren for BURST.
    """
    deadline = time.time() + timeout_s

    while time.time() < deadline:
        remaining_s = max(0.1, deadline - time.time())
        line = _readline_with_timeout(ser, timeout_s=remaining_s)

        if line is None:
            break

        _print_debug_line(enabled=debug, line=line)

        if line.startswith("ERROR"):
            raise RuntimeError(f"Arduino error: {line}")
        if line.startswith(CSV_HEADER_PREFIX):
            return line

    raise RuntimeError(
        "Arduino har ikke sendt CSV headeren (timeout). "
        "Sjekk at den riktige sketch er flashet og at BURST sender CSV headeren."
    )


def _read_burst_rows_from_arduino(ser, *, timeout_s: float, debug: bool) -> List[str]:
    rows: List[str] = []

    for row_index in range(BURST_ROWS_EXPECTED):
        row_deadline = time.time() + timeout_s
        accepted_line: Optional[str] = None

        while time.time() < row_deadline:
            remaining_s = max(0.1, row_deadline - time.time())
            candidate = _readline_with_timeout(ser, timeout_s=remaining_s)

            if candidate is None:
                break
            if not candidate:
                continue

            _print_debug_line(enabled=debug, line=candidate)

            if candidate.startswith("ERROR"):
                raise RuntimeError(f"Arduino error during burst: {candidate}")

            if "," in candidate:
                accepted_line = candidate
                break

        if accepted_line is None:
            raise RuntimeError(
                f"Timeout waiting for burst row {row_index} "
                f"(expected {BURST_ROWS_EXPECTED} rows)."
            )

        rows.append(accepted_line)

    return rows


def collect_burst(
    *,
    port: str,
    baud: int,
    sample_id: str,
    sample_background: bool = False,
    wait_after_sb_s: float = 0.0,
    interactive_after_background: bool = False,
    debug: bool = False,
    timeout_s: float = DEFAULT_TIMEOUT_S,
) -> Tuple[str, List[str]]:

    serial = _require_pyserial()

    try:
        # timeout=0 gjør lesingen non blocking. Timeout håndteres i våre egne lesefunksjoner.
        ser = serial.Serial(port=port, baudrate=baud, timeout=0)
    except Exception as exc:  # noqa: BLE001
        hint = _platform_port_hint(port)
        raise RuntimeError(f"Could not open serial port {port!r} at {baud}: {exc}\n{hint}") from exc

    with ser:
        # Arduino resetter ofte når serial porten åpnes. Pausen gjør oppstarten stabil.
        time.sleep(2.0)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        if sample_background:
            _send_command_and_expect_ok(ser, command="SB", timeout_s=timeout_s)
            _wait_after_background(
                wait_after_sb_s=wait_after_sb_s,
                interactive_after_background=interactive_after_background,
            )

        ser.write(f"BURST {sample_id}\n".encode("utf-8"))
        ser.flush()

        header = _read_csv_header_from_arduino(ser, timeout_s=timeout_s, debug=debug)
        rows = _read_burst_rows_from_arduino(ser, timeout_s=timeout_s, debug=debug)
        return header, rows


def write_raw_csv(*, output_path: Path, header_line: str, rows: List[str]) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

    header = next(csv.reader([header_line]))
    parsed_rows = [next(csv.reader([row])) for row in rows]

    expected_min_cols = BASE_COLUMNS_EXPECTED + SPECTRAL_COLUMNS_EXPECTED
    if len(header) < expected_min_cols:
        raise RuntimeError(f"Header has too few columns: {len(header)} < {expected_min_cols}")

    for row_index, row in enumerate(parsed_rows):
        if len(row) != len(header):
            raise RuntimeError(f"Row {row_index} has {len(row)} columns, expected {len(header)}")

    with output_path.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(header)
        writer.writerows(parsed_rows)


def ensure_metadata_file(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    if path.exists():
        return

    with path.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(METADATA_HEADER)


def append_metadata_row(path: Path, row: Dict[str, str]) -> None:
    ensure_metadata_file(path)

    with path.open("a", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=METADATA_HEADER)
        writer.writerow({key: row.get(key, "") for key in METADATA_HEADER})


def _triad_file_reference(output_path: Path) -> str:
    """
    Lag filreferanse til metadata.csv.

    Når råfilen ligger under ml/datasets, lagrer vi en relativ sti som
    raw/S0001_triad_raw.csv. Det gjør datasettet lettere å flytte mellom maskiner.
    """
    try:
        return str(output_path.relative_to(Path("ml/datasets")).as_posix())
    except ValueError:
        return str(output_path.as_posix())


def _build_metadata_row(args: argparse.Namespace, *, triad_file: str) -> Dict[str, str]:
    """Samle terminalargumentene som skal bli en rad i metadata.csv."""
    return {
        "sample_id": str(args.sample_id),
        "label_object": str(args.label_object),
        "label_material": str(args.label_material),
        "triad_file": triad_file,
        "sand_type": str(args.sand_type),
        "lysforhold": str(args.lysforhold),
        "avstand_cm": str(args.avstand_cm),
        "position_id": str(args.position_id),
        "angle_id": str(args.angle_id),
        "run_id": str(args.run_id),
        "diameter_mm": str(args.diameter_mm),
        "size_group": str(args.size_group),
        "surface_condition": str(args.surface_condition),
        "buried_level": str(args.buried_level),
    }


def _build_parser() -> argparse.ArgumentParser:
    """Definer alle terminalargumentene"""
    parser = argparse.ArgumentParser(description="Samle Triad burst over serial og lagre rå CSV.")
    parser.add_argument(
        "--list-ports",
        action="store_true",
        help="List serial ports and exit.",
    )
    parser.add_argument(
        "--port",
        required=False,
        help="Serial port (Windows: COM3/COM4, Linux/WSL: /dev/ttyACM0).",
    )
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baud rate (default 115200).")
    parser.add_argument("--sample-id", required=False, help="Sample ID (e.g. S0001).")
    parser.add_argument(
        "--sample-background",
        action="store_true",
        help="Send SB (sample sand background) before BURST.",
    )
    parser.add_argument(
        "--wait-after-sb",
        "--vent-etter-sb",
        dest="wait_after_sb",
        type=float,
        default=0.0,
        metavar="SEC",
        help=(
            "Etter SB: automatisk pause i SEC sekunder før BURST for autonom kjøring. "
        ),
    )
    parser.add_argument(
        "--interactive-after-background",
        action="store_true",
        help=(
            "Etter SB: stopp og vent på Enter før BURST (manuell start; ingen timer). "
        ),
    )
    parser.add_argument(
        "--output-dir",
        default=DEFAULT_OUTPUT_DIR,
        help="Output directory (default: ml/datasets/raw).",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite existing raw file if it already exists.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT_S,
        help="Line timeout seconds.",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Print received serial lines to stderr for debugging.",
    )
    parser.add_argument(
        "--metadata-path",
        default=DEFAULT_METADATA_PATH,
        help="Path to metadata CSV (default: ml/datasets/metadata.csv).",
    )
    parser.add_argument(
        "--append-metadata",
        action="store_true",
        help="Append a metadata row after successful burst collection.",
    )

    parser.add_argument("--label-object", default="", help="Metadata: label_object")
    parser.add_argument("--label-material", default="", help="Metadata: label_material")
    parser.add_argument("--sand-type", default="", help="Metadata: sand_type (e.g. torr_sand).")
    parser.add_argument("--lysforhold", default="", help="Metadata: lysforhold (f.eks. lampelys).")
    parser.add_argument("--avstand-cm", default="", help="Metadata: avstand_cm sensor objekt i cm.")
    parser.add_argument("--position-id", default="P1", help="Metadata: position_id (grid/felt).")
    parser.add_argument("--angle-id", default="A1", help="Metadata: angle_id / vinkelkode.")
    parser.add_argument("--run-id", default="", help="Metadata: run_id")
    parser.add_argument("--diameter-mm", default="", help="Metadata: diameter_mm")
    parser.add_argument("--size-group", default="", help="Metadata: size_group")
    parser.add_argument(
        "--surface-condition",
        "--surface_condition",
        default="",
        help="Metadata: surface_condition (f.eks. grov, glatt, sand_dekke).",
    )
    parser.add_argument(
        "--buried-level",
        "--buried_level",
        default="",
        help="Metadata: buried_level (f.eks. 0, delvis, full).",
    )
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    """Kjør innsamling fra terminal og returner exit kode."""
    parser = _build_parser()
    args = parser.parse_args(argv)

    if args.list_ports:
        _print_available_serial_ports()
        return 0

    if not args.port:
        parser.error("--port is required when not using --list-ports")
    if not args.sample_id:
        parser.error("--sample-id is required when collecting data")

    output_dir = Path(args.output_dir)
    output_path = output_dir / f"{args.sample_id}_triad_raw.csv"

    if output_path.exists() and not args.overwrite:
        print(
            f"[collect_triad_burst] ERROR: raw file already exists: {output_path}. "
            "Use --overwrite to replace it.",
            file=sys.stderr,
        )
        return 2

    try:
        header, rows = collect_burst(
            port=args.port,
            baud=int(args.baud),
            sample_id=str(args.sample_id),
            sample_background=bool(args.sample_background),
            wait_after_sb_s=float(args.wait_after_sb),
            interactive_after_background=bool(args.interactive_after_background),
            debug=bool(args.debug),
            timeout_s=float(args.timeout),
        )
        write_raw_csv(output_path=output_path, header_line=header, rows=rows)
    except Exception as exc:  # noqa: BLE001
        print(f"[collect_triad_burst] ERROR: {exc}", file=sys.stderr)
        return 2

    if args.append_metadata:
        metadata_path = Path(args.metadata_path)
        metadata_row = _build_metadata_row(
            args,
            triad_file=_triad_file_reference(output_path),
        )

        try:
            append_metadata_row(metadata_path, metadata_row)
            print(f"[collect_triad_burst] Appended metadata row to: {metadata_path}")
        except Exception as exc:  # noqa: BLE001
            print(f"[collect_triad_burst] ERROR: failed to append metadata: {exc}", file=sys.stderr)
            return 2

    print(f"[collect_triad_burst] Saved: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
