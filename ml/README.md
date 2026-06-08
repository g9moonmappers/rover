# Maskinlæring — Triad spektral klassifisering (`ml/`)

Denne guiden er for **hele teamet** som jobber med MoonMapper-roveren. Den forklarer hva som er implementert under `ml/`, hvordan du installerer avhengigheter på **Windows, macOS og Linux**, og alle kommandoer for datainnsamling, trening og prediksjon.

> **Kortversjon:** ML trenger **ikke** ROS 2. Du trenger Python 3.10+ og pakkene i `requirements-ml.txt`. ROS/simulering dokumenteres i [src/digital_tvilling.md](../src/digital_tvilling.md).

---

## Innhold

1. [Hva gjør ML-delen?](#1-hva-gjør-ml-delen)
2. [Mappestruktur](#2-mappestruktur)
3. [Hva er implementert](#3-hva-er-implementert)
4. [Hardware og Arduino](#4-hardware-og-arduino)
5. [Operativsystem og Python](#5-operativsystem-og-python)
6. [Installasjon av avhengigheter](#6-installasjon-av-avhengigheter)
7. [Aktivere ML-miljø (hver gang)](#7-aktivere-ml-miljø-hver-gang)
8. [Datasett og metadata](#8-datasett-og-metadata)
9. [Arbeidsflyt fra rå data til modell](#9-arbeidsflyt-fra-rå-data-til-modell)
10. [Alle scripts og kommandoer](#10-alle-scripts-og-kommandoer)
11. [Konfigurasjon (`configs/`)](#11-konfigurasjon-configs)
12. [Feilsøking](#12-feilsøking)
13. [Ordliste (ML for nybegynnere)](#13-ordliste-ml-for-nybegynnere)

---

## 1. Hva gjør ML-delen?

Roveren har **to Triad-spektralsensorer** (AS7265X, 18 bølgelengder hver → 36 kanaler totalt). ML-pipelinen:

1. **Samler inn** korte «burst»-målinger over USB/serial fra Arduino.
2. **Lagrer** rå CSV + metadata (materiale, lys, avstand, …).
3. **Ekstraherer** numeriske features (gjennomsnitt, std, SAM-lignende ratioer, …).
4. **Trener** en **Random Forest** til å gjenkjenne objekt-/materialklasser.
5. **Predikerer** label på nye målinger.

Dette er **uavhengig** av Gazebo/ROS — men resultatene skal senere kunne brukes på roveren for materialgjenkjenning.

---

## 2. Mappestruktur

```
ml/
├── configs/              # labels.yaml, feature_config.yaml
├── data_collection/      # Serial-innsamling fra Arduino
├── datasets/
│   ├── raw/              # Rå burst-CSV (S0001_triad_raw.csv, …)
│   ├── live/             # Live/test-innsamling (T0001, S9999, …)
│   ├── processed/        # Features, train/val/test split, SAM-referanser
│   └── metadata.csv      # en rad per prøve (kobler sample_id → fil + labels)
├── models/               # Trenede .joblib (gitignored — ikke på GitHub)
├── scripts/              # predict.py, add_feedback_sample.py
└── training/             # extract, split, train, evaluate
```

**Rot av repo:**

| Fil | Formål |
|-----|--------|
| `requirements-ml.txt` | Python-pakker |
| `scripts/activate_ml.sh` | Aktiverer `.venv-ml` |
| `arduino/moonmapper_triad_logger/` | Firmware til innsamling |

---

## 3. Hva er implementert

### 3.1 Datainnsamling

| Script | Beskrivelse |
|--------|-------------|
| `data_collection/collect_triad_burst.py` | Snakker med Arduino over serial, lagrer `*_triad_raw.csv`, kan oppdatere `metadata.csv` |
| `data_collection/fill_metadata_defaults.py` | Fyller tomme metadata-felt med standardverdier |

**Rå CSV-format:** `sample_id`, `burst_index`, `timestamp_ms`, deretter 36 spektralkolonner (`S0_410` … `N1_940`).

### 3.2 Feature-ekstraksjon og datasett

| Script | Beskrivelse |
|--------|-------------|
| `training/extract_triad_features.py` | Leser `metadata.csv` + rå filer → `processed/triad_features.csv` + `sam_reference_spectra.json` |
| `training/split_dataset.py` | Deler features i `train.csv`, `val.csv`, `test.csv` (stratified) |

Features inkluderer bl.a. (styrt av `feature_config.yaml`): mean/std/min/max per kanal, RMS, ratioer, derivater.

### 3.3 Trening og evaluering

| Script | Beskrivelse |
|--------|-------------|
| `training/train_random_forest.py` | Trener Random Forest, lagrer `ml/models/random_forest.joblib` + `label_encoder.joblib` |
| `training/evaluate_model.py` | Evaluerer på `test.csv`, kan vise confusion matrix (matplotlib) |

### 3.4 Prediksjon og feedback

| Script | Beskrivelse |
|--------|-------------|
| `scripts/predict.py` | Predikerer label fra en rå burst-CSV |
| `scripts/add_feedback_sample.py` | Flytter live-prøve til `raw/` og oppdaterer metadata (læring i felt) |

### 3.5 Klasser (labels)

Definert i `configs/labels.yaml`:

- **Objekter:** `aluminium_kule`, `jern_kule`, `stål_kule`, `titan_kule`
- **Materialer:** `aluminium`, `jern`, `stål`, `titan`, `ukjent`
- **Primær label for trening:** `label_object` (kan overstyres med `--label-column`)

---

## 4. Hardware og Arduino

1. Last opp **`arduino/moonmapper_triad_logger/moonmapper_triad_logger.ino`** til Arduino (SparkFun AS7265X + I2C-mux).
2. Koble USB — noter port (f.eks. `/dev/ttyACM0` på Linux, `COM3` på Windows).
3. Serial **115200 baud**. Kommandoer fra PC: `SB` (bakgrunn/sand), `BURST <sample_id>`, `PS`.

**Biblioteker i Arduino IDE:** SparkFun AS7265X, SparkFun I2C Mux.

---

## 5. Operativsystem og Python

| OS | ML (Python) | Serial (Arduino) |
|----|-------------|-------------------|
| **Ubuntu / Debian** | ✅ Anbefalt | ✅ `/dev/ttyACM0` |
| **Windows 10/11** | ✅ | ✅ `COM3` (Device Manager) |
| **macOS** | ✅ | ✅ `/dev/tty.usbmodem*` |

**Python-versjon:** 3.10, 3.11 eller 3.12 (testet med 3.12 på Ubuntu 24.04).

Du trenger **ikke** ROS for ML. Du trenger **ikke** NVIDIA-GPU — Random Forest kjører på CPU.

---

## 6. Installasjon av avhengigheter

Alle kommandoer kjøres fra **repo-rot** (mappen som inneholder `ml/` og `src/`).

### 6.1 Linux / macOS (anbefalt — automatisk)

```bash
cd /sti/til/rover
source scripts/activate_ml.sh
```

Første gang oppretter skriptet `.venv-ml` og kjører `pip install -r requirements-ml.txt`.

### 6.2 Linux / macOS (manuelt)

```bash
cd /sti/til/rover
python3 -m venv .venv-ml
source .venv-ml/bin/activate
pip install -r requirements-ml.txt
export PYTHONPATH="$(pwd):${PYTHONPATH:-}"
```

### 6.3 Windows (PowerShell)

```powershell
cd C:\sti\til\rover
python -m venv .venv-ml
.\.venv-ml\Scripts\Activate.ps1
pip install -r requirements-ml.txt
$env:PYTHONPATH = "$PWD;$env:PYTHONPATH"
```

Hvis `Activate.ps1` blokkeres: `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`

### 6.4 Windows (cmd)

```cmd
cd C:\sti\til\rover
python -m venv .venv-ml
.venv-ml\Scripts\activate.bat
pip install -r requirements-ml.txt
set PYTHONPATH=%CD%;%PYTHONPATH%
```

### 6.5 Pakker som installeres

Fra `requirements-ml.txt`:

| Pakke | Bruk |
|-------|------|
| numpy | Arrays, feature-matematikk |
| pandas | CSV, treningstabeller |
| scikit-learn | Random Forest, split, metrics |
| joblib | Lagre/laste modeller |
| matplotlib | Plots i `evaluate_model.py` |
| pyserial | USB til Arduino |

### 6.6 Verifiser installasjon

```bash
python -c "import numpy, pandas, sklearn, joblib, serial; print('ML OK')"
```

---

## 7. Aktivere ML-miljø (hver gang)

**Linux / macOS:**

```bash
cd /sti/til/rover
source scripts/activate_ml.sh
```

**Windows:** se §6.3 eller 6.4.

Du skal se `(.venv-ml)` i prompten. Alle `python ml/...`-kommandoer kjøres etter dette.

---

## 8. Datasett og metadata

### 8.1 `metadata.csv`

en rad per prøve. Viktige kolonner:

| Kolonne | Betydning |
|---------|-----------|
| `sample_id` | Unik ID, f.eks. `S0042` |
| `label_object` | Klasse (kule-type) |
| `label_material` | Materialklassifisering |
| `triad_file` | Relativ sti til rå CSV, f.eks. `raw/S0042_triad_raw.csv` |
| `sand_type`, `lysforhold`, `avstand_cm` | Eksperimentforhold |
| `position_id`, `angle_id`, `run_id` | Posisjon / vinkel / kjøring |

### 8.2 Navngiving rå filer

- **Treningsdata:** `ml/datasets/raw/S####_triad_raw.csv`
- **Live/test:** `ml/datasets/live/T####_triad_raw.csv` eller `S9999_...`

### 8.3 Git og modeller

Filer i `ml/models/*.joblib` er **ikke** på GitHub (`.gitignore`). Hver utvikler må trene lokalt eller dele modeller via Teams/USB.

**Eksempelmodeller** (hvis du har dem lokalt): `random_forest_example_binary.joblib`, `label_encoder_example_binary.joblib`.

---

## 9. Arbeidsflyt fra rå data til modell

```
Arduino BURST → raw/Sxxxx_triad_raw.csv
       ↓
metadata.csv (rad per prøve)
       ↓
extract_triad_features.py → processed/triad_features.csv
       ↓
split_dataset.py → train.csv, val.csv, test.csv
       ↓
train_random_forest.py → models/*.joblib
       ↓
evaluate_model.py / predict.py
```

**Minimal engangskjøring (fra repo-rot, med venv aktiv):**

```bash
python ml/training/extract_triad_features.py
python ml/training/split_dataset.py
python ml/training/train_random_forest.py
python ml/training/evaluate_model.py
```

---

## 10. Alle scripts og kommandoer

Alle kommandoer: **repo-rot**, `source scripts/activate_ml.sh` (eller venv aktiv).

### 10.1 `collect_triad_burst.py` — innsamling

```bash
# List serial-porter
python ml/data_collection/collect_triad_burst.py --list-ports

# Linux — typisk port
python ml/data_collection/collect_triad_burst.py \
  --port /dev/ttyACM0 \
  --sample-id S0054 \
  --label-object stål_kule \
  --label-material stål \
  --sand-type torr_sand \
  --lysforhold lampelys \
  --avstand-cm 15 \
  --append-metadata

# Windows
python ml/data_collection/collect_triad_burst.py --port COM3 --sample-id S0054 --append-metadata

# Overskriv eksisterende fil
python ml/data_collection/collect_triad_burst.py --port /dev/ttyACM0 --sample-id S0054 --overwrite
```

| Argument | Beskrivelse |
|----------|-------------|
| `--port` | Serial-port (påkrevd) |
| `--sample-id` | ID i filnavn og CSV |
| `--list-ports` | Vis tilgjengelige porter |
| `--baud` | Standard 115200 |
| `--output-dir` | Standard `ml/datasets/raw` |
| `--append-metadata` | Legg til rad i `metadata.csv` |
| `--overwrite` | Erstatt eksisterende råfil |
| `--sample-background` | Kjør bakgrunnskalibrering (sand) |
| `--debug` | Mer serial-logging |

### 10.2 `fill_metadata_defaults.py`

```bash
python ml/data_collection/fill_metadata_defaults.py
python ml/data_collection/fill_metadata_defaults.py --metadata ml/datasets/metadata.csv --dry-run
```

### 10.3 `extract_triad_features.py`

```bash
python ml/training/extract_triad_features.py

python ml/training/extract_triad_features.py \
  --metadata ml/datasets/metadata.csv \
  --raw-dir ml/datasets \
  --output ml/datasets/processed/triad_features.csv
```

**Output:** `triad_features.csv` + `sam_reference_spectra.json`.

### 10.4 `split_dataset.py`

```bash
python ml/training/split_dataset.py

python ml/training/split_dataset.py \
  --input ml/datasets/processed/triad_features.csv \
  --train-out ml/datasets/processed/train.csv \
  --val-out ml/datasets/processed/val.csv \
  --test-out ml/datasets/processed/test.csv \
  --test-ratio 0.15 --val-ratio 0.15 --seed 42
```

### 10.5 `train_random_forest.py`

```bash
# Standard: tren på train.csv, rapporter val-metrikker på val.csv
python ml/training/train_random_forest.py \
  --train-input ml/datasets/processed/train.csv \
  --val-input ml/datasets/processed/val.csv \
  --label-column label_object

# Materialklassifisering
python ml/training/train_random_forest.py \
  --label-column label_material
```

Trener **kun** på `train.csv`. `test.csv` holdes helt ute til `evaluate_model.py`.
Trenings på `triad_features.csv` blokkeres for å unngå datalekkasje.

**Output (standard):**

- `ml/models/random_forest.joblib`
- `ml/models/label_encoder.joblib`

### 10.6 `evaluate_model.py`

```bash
python ml/training/evaluate_model.py

python ml/training/evaluate_model.py \
  --model-path ml/models/random_forest.joblib \
  --encoder-path ml/models/label_encoder.joblib \
  --test-csv ml/datasets/processed/test.csv
```

### 10.7 `predict.py`

```bash
python ml/scripts/predict.py \
  --raw ml/datasets/raw/S0006_triad_raw.csv

python ml/scripts/predict.py \
  --raw ml/datasets/live/T0001_triad_raw.csv \
  --model ml/models/random_forest.joblib \
  --encoder ml/models/label_encoder.joblib \
  --max-rows 11
```

### 10.8 `add_feedback_sample.py`

```bash
python ml/scripts/add_feedback_sample.py \
  --live ml/datasets/live/T0008_triad_raw.csv \
  --label-object jern_kule \
  --label-material jern
```

Flytter live-fil til `raw/` og oppdaterer metadata — deretter kjør extract → split → train på nytt.

---

## 11. Konfigurasjon (`configs/`)

### `labels.yaml`

Definerer gyldige klassenavn for objekt og materiale. Endre kun etter teamavtale (krever ny merking av data).

### `feature_config.yaml`

- `triad`: 2 sensorer × 18 kanaler = 36
- `features`: hvilke feature-typer som genereres
- `burst.recommended_samples_per_object`: 20 (minimum 10)

---

## 12. Feilsøking

| Symptom | Løsning |
|---------|---------|
| `ModuleNotFoundError: ml` | Kjør fra repo-rot; `export PYTHONPATH=$(pwd)` eller `activate_ml.sh` |
| `pyserial is not installed` | `pip install pyserial` i venv |
| `Permission denied` på `/dev/ttyACM0` | `sudo usermod -aG dialout $USER` — logg ut/inn |
| `raw file already exists` | Bruk `--overwrite` eller ny `sample_id` |
| `Missing dependencies pandas sklearn` | `source scripts/activate_ml.sh` |
| Tom `train.csv` etter split | For få rader i `triad_features.csv` — samle mer data |
| `No numeric feature columns` | Kjør `extract_triad_features.py` først |
| Arduino svarer ikke | Sjekk baud 115200, riktig port, USB-kabel, sketch lastet opp |
| Modell finnes ikke ved predict | Tren med `train_random_forest.py` eller kopier `.joblib` til `ml/models/` |

**Linux serial-tips:**

```bash
ls /dev/ttyACM* /dev/ttyUSB*
dmesg | tail   # etter plugging USB
```

**Windows serial-tips:** Enhetsbehandling → COM-port; lukk Arduino Serial Monitor før Python kjører.

---

## 13. Ordliste (ML for nybegynnere)

| Begrep | Forklaring |
|--------|------------|
| **Burst** | Kort serie spektralmålinger (typisk 20 rader) |
| **Feature** | Tall utledet fra rå spektrum (mean, std, …) |
| **Label** | Riktig svar (f.eks. `stål_kule`) |
| **Train/val/test** | Trening, validering, test — hold test-data hemmelig til slutt |
| **Random Forest** | Ensemble av beslutningstrær — robust, lite tuning |
| **joblib** | Filformat for lagrede modeller i scikit-learn |
| **LabelEncoder** | Mapper tekst-labels til tall for ML |
| **SAM** | Spectral Angle Mapper — vinkel mellom spektrum og referanse |
| **metadata.csv** | «Regneark» som binder filer til labels og forsøksforhold |

---

## Relatert dokumentasjon

- [src/digital_tvilling.md](../src/digital_tvilling.md) — Gazebo, ROS 2, autonomi
- [README.md](../README.md) — hardware, Jetson, eldre oppsett
- `requirements-ml.txt`, `scripts/activate_ml.sh`
- Arduino: `arduino/moonmapper_triad_logger/moonmapper_triad_logger.ino`

**Repo:** https://github.com/g9moonmappers/rover
