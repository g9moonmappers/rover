#!/usr/bin/env bash
# Aktiver Python-miljø for ml/. Kjør: source scripts/activate_ml.sh
_WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_VENV="${_WS_ROOT}/.venv-ml"

if [[ ! -f "${_VENV}/bin/activate" ]]; then
  echo "Oppretter .venv-ml og installerer avhengigheter …" >&2
  python3 -m venv "${_VENV}"
  "${_VENV}/bin/pip" install -r "${_WS_ROOT}/requirements-ml.txt"
fi

# shellcheck source=/dev/null
source "${_VENV}/bin/activate"
export PYTHONPATH="${_WS_ROOT}:${PYTHONPATH:-}"
cd "${_WS_ROOT}" || return 1 2>/dev/null || exit 1
