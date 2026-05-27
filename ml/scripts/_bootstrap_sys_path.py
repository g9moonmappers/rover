"""Legg til workspace root på sys.path slik at import ml. skal fungerer når scripts kjøres fra repo root."""

from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent.parent
_rp = str(_ROOT)
if _rp not in sys.path:
    sys.path.insert(0, _rp)
