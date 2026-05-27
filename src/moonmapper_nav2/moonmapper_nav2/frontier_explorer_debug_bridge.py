"""Valgfri bro til arkivert frontier-debug (lastes bare når parametre er på)."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import TYPE_CHECKING, List, Optional

from moonmapper_nav2.frontier_grid import ScoredGoalCandidate, ValidatedGoal

if TYPE_CHECKING:
    from moonmapper_nav2.frontier_explorer import FrontierExplorer

_archive_mod = None


def _archive_modules_dir() -> Path:
    return (
        Path(__file__).resolve().parents[3]
        / "arkiverte_koder"
        / "nav_config_opprydding"
        / "modules"
    )


def _load_archive_debug():
    global _archive_mod
    if _archive_mod is not None:
        return _archive_mod
    mod_dir = str(_archive_modules_dir())
    if mod_dir not in sys.path:
        sys.path.insert(0, mod_dir)
    import frontier_explorer_debug as fe_debug  # noqa: WPS433

    _archive_mod = fe_debug
    return _archive_mod


def log_map_pick_debug(
    explorer: "FrontierExplorer",
    data,
    rx: float,
    ry: float,
    w: int,
    h: int,
    ox: float,
    oy: float,
    res: float,
    unk: int,
    free: int,
    occ: int,
) -> None:
    if not bool(explorer.get_parameter("enable_map_debug_log").value):
        return
    mod = _load_archive_debug()
    mod.log_map_pick_diagnostics(
        explorer.get_logger(),
        explorer._map,
        data,
        rx,
        ry,
        w,
        h,
        ox,
        oy,
        res,
        unk,
        free,
        occ,
    )


def publish_debug_markers(
    explorer: "FrontierExplorer",
    clusters,
    selected: Optional[ValidatedGoal],
    safe: List[ScoredGoalCandidate],
    rejected: List[ScoredGoalCandidate],
) -> None:
    if not bool(explorer.get_parameter("publish_debug_markers").value):
        return
    mod = _load_archive_debug()
    mod.publish_debug_markers(explorer, clusters, selected, safe, rejected)


def maybe_log_map_robot_diag(explorer: "FrontierExplorer", rx: float, ry: float) -> None:
    if not bool(explorer.get_parameter("enable_map_debug_log").value):
        return
    mod = _load_archive_debug()
    mod.maybe_log_map_robot_diag(
        explorer.get_logger(), "_logged_map_tf_diag", explorer, rx, ry
    )
