"""Kartmaske, frontier-klynger, BFS og valg av trygt navigasjonsmal."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field, replace
from enum import Enum
from typing import Dict, List, Optional, Sequence, Tuple

from moonmapper_nav2.frontier_utils import (
    FrontierCluster,
    cell_value,
    _is_free,
    _is_unknown,
    _is_occupied,
    world_to_map,
)


class RejectReason(str, Enum):
    NO_APPROACH_CELL = "no_approach_cell"


@dataclass
class ApproachPickStats:
    candidates_total: int = 0
    rejected_out_of_bounds: int = 0
    rejected_annulus: int = 0
    rejected_unknown: int = 0
    rejected_occupied: int = 0
    rejected_not_reachable: int = 0
    rejected_not_passable: int = 0
    rejected_too_close_obstacle: int = 0
    rejected_blacklist: int = 0
    rejected_robot_distance: int = 0

    def summary_line(self) -> str:
        parts = [
            f"candidates_total={self.candidates_total}",
            f"rejected_unknown={self.rejected_unknown}",
            f"rejected_occupied={self.rejected_occupied}",
            f"rejected_not_reachable={self.rejected_not_reachable}",
            f"rejected_not_passable={self.rejected_not_passable}",
            f"rejected_too_close_obstacle={self.rejected_too_close_obstacle}",
            f"rejected_blacklist={self.rejected_blacklist}",
            f"rejected_robot_distance={self.rejected_robot_distance}",
        ]
        return " ".join(parts)


@dataclass
class ValidatedGoal:
    wx: float
    wy: float
    yaw: float
    approach_ixy: Tuple[int, int]
    cluster: FrontierCluster
    approach_method: str
    obstacle_clearance: float = 0.0
    unknown_clearance: float = 0.0
    score: float = 0.0
    cluster_id: int = -1


@dataclass
class SafeGoalPickConfig:
    min_goal_dist_m: float = 1.0
    max_goal_dist_m: float = 6.0
    min_obstacle_clearance_m: float = 0.45
    min_obstacle_clearance_floor_m: float = 0.30
    preferred_obstacle_clearance_m: float = 0.65
    min_unknown_clearance_m: float = 0.10
    preferred_unknown_clearance_m: float = 0.25
    approach_radius_min_m: float = 0.2
    approach_radius_max_m: float = 2.0
    score_distance_weight: float = 1.0
    score_obstacle_weight: float = 3.0
    score_cluster_weight: float = 1.5
    score_unknown_weight: float = 1.0
    min_free_space_around_goal_m: float = 0.45
    avoid_corner_goals: bool = True
    allow_corner_fallback: bool = True
    corner_penalty: float = 5.0
    corner_check_radius_m: float = 0.5
    require_passable_for_approach: bool = False
    neighbor_fallback_m: float = 1.0
    min_passage_clearance_m: float = 0.38
    min_passage_clearance_floor_m: float = 0.28
    narrow_passage_penalty_weight: float = 4.0
    explored_revisit_penalty: float = 2.5


@dataclass
class ScoredGoalCandidate:
    cluster_id: int
    cluster: FrontierCluster
    wx: float
    wy: float
    mx: int
    my: int
    dist_robot: float
    obstacle_clearance: float
    unknown_clearance: float
    score: float
    is_corner: bool
    method: str
    unsafe: bool = False
    reject_reason: str = ""


@dataclass
class SafeGoalPickResult:
    goal: Optional[ValidatedGoal]
    safe_candidates: List[ScoredGoalCandidate] = field(default_factory=list)
    rejected_unsafe: List[ScoredGoalCandidate] = field(default_factory=list)
    clearance_relaxed: bool = False
    clearance_used_m: float = 0.45


def _idx(mx: int, my: int, w: int) -> int:
    return my * w + mx


def collect_frontier_clusters(
    data: Sequence[int],
    w: int,
    h: int,
    unknown_value: int,
    free_threshold: int,
    occupied_threshold: int,
    min_cluster_size: int,
) -> List[FrontierCluster]:
    """Frie celler (kjent, ikke okkupert) med minst én ukjent 4-nabo."""
    vis = [False] * (w * h)
    clusters: List[FrontierCluster] = []
    neigh = ((1, 0), (-1, 0), (0, 1), (0, -1))
    for my in range(h):
        for mx in range(w):
            i = _idx(mx, my, w)
            v = int(data[i])
            if vis[i]:
                continue
            if not _is_free(v, free_threshold, occupied_threshold, unknown_value):
                continue
            touches_unknown = False
            for dx, dy in neigh:
                nv = cell_value(list(data), w, h, mx + dx, my + dy)
                if nv is not None and int(nv) == unknown_value:
                    touches_unknown = True
                    break
            if not touches_unknown:
                continue
            stack = [(mx, my)]
            vis[i] = True
            cells: List[Tuple[int, int]] = []
            while stack:
                cx, cy = stack.pop()
                cells.append((cx, cy))
                for dx, dy in neigh:
                    nx, ny = cx + dx, cy + dy
                    ni = _idx(nx, ny, w)
                    if nx < 0 or ny < 0 or nx >= w or ny >= h or vis[ni]:
                        continue
                    tv = int(data[ni])
                    if not _is_free(tv, free_threshold, occupied_threshold, unknown_value):
                        continue
                    touches_u = False
                    for ddx, ddy in neigh:
                        uv = cell_value(list(data), w, h, nx + ddx, ny + ddy)
                        if uv is not None and int(uv) == unknown_value:
                            touches_u = True
                            break
                    if not touches_u:
                        continue
                    vis[ni] = True
                    stack.append((nx, ny))
            if len(cells) >= min_cluster_size:
                clusters.append(FrontierCluster(cells=cells))
    return clusters


# Fri/okkupert/ukjent + inflasjon rundt hindringer til bool-maske for planlegger.
def build_passable_mask(
    data: Sequence[int],
    w: int,
    h: int,
    unknown_value: int,
    occupied_threshold: int,
    free_threshold: int,
    unknown_as_blocked: bool,
    inflation_radius_m: float,
    resolution: float,
) -> Tuple[List[bool], List[bool]]:
    """Kjent-frie passerbare celler med erosjon rundt hindring/ukjent (inflasjon)."""
    n = w * h
    raw_blocked = [False] * n
    for i in range(n):
        v = int(data[i])
        if _is_unknown(v, unknown_value):
            raw_blocked[i] = unknown_as_blocked
        elif _is_occupied(v, occupied_threshold):
            raw_blocked[i] = True
        elif not _is_free(v, free_threshold, occupied_threshold, unknown_value):
            raw_blocked[i] = True

    steps = max(0, int(round(float(inflation_radius_m) / max(resolution, 1e-6))))
    blocked = [bool(x) for x in raw_blocked]
    tmp = [False] * n
    for _ in range(steps):
        for my in range(h):
            for mx in range(w):
                i = _idx(mx, my, w)
                if raw_blocked[i]:
                    tmp[i] = True
                    continue
                neigh_blk = False
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        nx, ny = mx + dx, my + dy
                        if nx < 0 or ny < 0 or nx >= w or ny >= h:
                            continue
                        if blocked[_idx(nx, ny, w)]:
                            neigh_blk = True
                            break
                    if neigh_blk:
                        break
                tmp[i] = neigh_blk
        blocked, tmp = tmp, [False] * n

    passable = [False] * n
    for i in range(n):
        v = int(data[i])
        if _is_unknown(v, unknown_value):
            passable[i] = False
        elif _is_occupied(v, occupied_threshold):
            passable[i] = False
        elif not _is_free(v, free_threshold, occupied_threshold, unknown_value):
            passable[i] = False
        else:
            passable[i] = not blocked[i]
    inflation_dbg = blocked
    return passable, inflation_dbg


def _bfs_mask(
    seed: Optional[Tuple[int, int]], passable: Sequence[bool], w: int, h: int
) -> List[bool]:
    reach = [False] * (w * h)
    if seed is None:
        return reach
    sx, sy = seed
    if sx < 0 or sy < 0 or sx >= w or sy >= h:
        return reach
    si = _idx(sx, sy, w)
    if not passable[si]:
        return reach
    q = [seed]
    reach[si] = True
    head = 0
    neigh = ((1, 0), (-1, 0), (0, 1), (0, -1))
    while head < len(q):
        cx, cy = q[head]
        head += 1
        for dx, dy in neigh:
            nx, ny = cx + dx, cy + dy
            if nx < 0 or ny < 0 or nx >= w or ny >= h:
                continue
            ni = _idx(nx, ny, w)
            if reach[ni] or not passable[ni]:
                continue
            reach[ni] = True
            q.append((nx, ny))
    return reach




def build_bfs_seed_radii_m(max_m: float, step_m: float) -> Tuple[float, ...]:
    checkpoints = [0.25, 0.5, 1.0, 1.5, 2.0, 3.0]
    max_m = float(max(max_m, 0.0))
    rings = {float(r) for r in checkpoints if r <= max_m + 1e-9}
    step = float(max(step_m, 0.05))
    cur = step
    while cur <= max_m + 1e-9:
        rings.add(round(cur, 4))
        cur += step
    out = tuple(sorted(rings))
    if out:
        return out
    if max_m <= 0.0:
        return (0.25,)
    return tuple(sorted({min(0.25, max_m), max_m}))


def count_raw_frontier_cells(
    data: Sequence[int],
    w: int,
    h: int,
    unknown_value: int,
    free_threshold: int,
    occupied_threshold: int,
) -> int:
    neigh = ((1, 0), (-1, 0), (0, 1), (0, -1))
    n_cells = 0
    lst = list(data)
    for my in range(h):
        for mx in range(w):
            i = _idx(mx, my, w)
            v = int(lst[i])
            if not _is_free(v, free_threshold, occupied_threshold, unknown_value):
                continue
            touches_unknown = False
            for dx, dy in neigh:
                nv = cell_value(lst, w, h, mx + dx, my + dy)
                if nv is not None and int(nv) == unknown_value:
                    touches_unknown = True
                    break
            if touches_unknown:
                n_cells += 1
    return n_cells


def global_nearest_free_cell(
    rx: float,
    ry: float,
    data: Sequence[int],
    w: int,
    h: int,
    ox: float,
    oy: float,
    res: float,
    unknown_value: int,
    free_threshold: int,
    occupied_threshold: int,
) -> Tuple[Optional[Tuple[int, int]], float]:
    best: Optional[Tuple[int, int]] = None
    best_d = 1e18
    for my in range(h):
        for mx in range(w):
            v = int(data[_idx(mx, my, w)])
            if not _is_free(v, free_threshold, occupied_threshold, unknown_value):
                continue
            wx = ox + (mx + 0.5) * res
            wy = oy + (my + 0.5) * res
            d = math.hypot(wx - rx, wy - ry)
            if d < best_d:
                best_d = d
                best = (mx, my)
    if best is None:
        return None, 0.0
    return best, best_d


def _grid_bounds_for_disk(
    rx: float,
    ry: float,
    rd: float,
    ox: float,
    oy: float,
    res: float,
    w: int,
    h: int,
    margin_cells: int,
) -> Tuple[int, int, int, int]:
    """Inklusive mx/my-grenser for celler der sentrum kan ligge innenfor rd av (rx,ry)."""
    res = max(float(res), 1e-9)
    mx_lo = int(math.ceil((rx - rd - ox) / res - 0.5 - 1e-9))
    mx_hi = int(math.floor((rx + rd - ox) / res - 0.5 + 1e-9))
    my_lo = int(math.ceil((ry - rd - oy) / res - 0.5 - 1e-9))
    my_hi = int(math.floor((ry + rd - oy) / res - 0.5 + 1e-9))
    m = max(0, margin_cells)
    mx_lo -= m
    mx_hi += m
    my_lo -= m
    my_hi += m
    mx_lo = max(0, mx_lo)
    mx_hi = min(w - 1, mx_hi)
    my_lo = max(0, my_lo)
    my_hi = min(h - 1, my_hi)
    if mx_lo > mx_hi or my_lo > my_hi:
        return 0, w - 1, 0, h - 1
    return mx_lo, mx_hi, my_lo, my_hi


def global_nearest_passable_cell(
    rx: float,
    ry: float,
    passable_bfs: Sequence[bool],
    w: int,
    h: int,
    ox: float,
    oy: float,
    res: float,
) -> Tuple[Optional[Tuple[int, int]], float]:
    """Nærmeste celle med passable_bfs True (euclidisk avstand til cellesentrum i kartplanet)."""
    best: Optional[Tuple[int, int]] = None
    best_d = 1e18
    for my in range(h):
        for mx in range(w):
            i = _idx(mx, my, w)
            if not passable_bfs[i]:
                continue
            wx = ox + (mx + 0.5) * res
            wy = oy + (my + 0.5) * res
            d = math.hypot(wx - rx, wy - ry)
            if d < best_d:
                best_d = d
                best = (mx, my)
    if best is None:
        return None, 0.0
    return best, best_d


def global_nearest_passable_known_free_cell(
    rx: float,
    ry: float,
    data: Sequence[int],
    passable_bfs: Sequence[bool],
    w: int,
    h: int,
    ox: float,
    oy: float,
    res: float,
    unknown_value: int,
    free_threshold: int,
    occupied_threshold: int,
) -> Tuple[Optional[Tuple[int, int]], float]:
    """Nærmeste passable_bfs-celle som også er kjent-fri i occupancy (ikke bare ukjent overlay)."""
    best: Optional[Tuple[int, int]] = None
    best_d = 1e18
    for my in range(h):
        for mx in range(w):
            i = _idx(mx, my, w)
            if not passable_bfs[i]:
                continue
            v = int(data[i])
            if not _is_free(v, free_threshold, occupied_threshold, unknown_value):
                continue
            wx = ox + (mx + 0.5) * res
            wy = oy + (my + 0.5) * res
            d = math.hypot(wx - rx, wy - ry)
            if d < best_d:
                best_d = d
                best = (mx, my)
    if best is None:
        return None, 0.0
    return best, best_d


def find_nearest_bfs_seed(
    robot_ixy: Tuple[int, int],
    rx: float,
    ry: float,
    data: Sequence[int],
    passable_bfs: Sequence[bool],
    w: int,
    h: int,
    ox: float,
    oy: float,
    res: float,
    unknown_value: int,
    free_threshold: int,
    occupied_threshold: int,
    max_radius_m: float,
    step_m: float,
) -> Tuple[Optional[Tuple[int, int]], float, str]:
    """Nærmeste kjent-fri + passable_bfs-celle innenfor voksende euklidske skiver."""
    _ = robot_ixy
    for rd in build_bfs_seed_radii_m(max_radius_m, step_m):
        best_d = 1e18
        best: Optional[Tuple[int, int]] = None
        mx_lo, mx_hi, my_lo, my_hi = _grid_bounds_for_disk(rx, ry, rd, ox, oy, res, w, h, 1)
        for my in range(my_lo, my_hi + 1):
            for mx in range(mx_lo, mx_hi + 1):
                wc_x = ox + (mx + 0.5) * res
                wc_y = oy + (my + 0.5) * res
                dm = math.hypot(wc_x - rx, wc_y - ry)
                if dm > rd + 1e-6:
                    continue
                idx = _idx(mx, my, w)
                if not passable_bfs[idx]:
                    continue
                v = int(data[idx])
                if not _is_free(v, free_threshold, occupied_threshold, unknown_value):
                    continue
                if dm < best_d:
                    best_d = dm
                    best = (mx, my)
        if best is not None:
            return best, best_d, "nearest_known_free_radius"
    return None, 0.0, "none"


def _count_reachable(reach: Sequence[bool]) -> int:
    return sum(1 for x in reach if x)


def bridge_passable_bfs_to_known_free(
    pass_bfs: Sequence[bool],
    data: Sequence[int],
    w: int,
    h: int,
    rx: float,
    ry: float,
    ox: float,
    oy: float,
    res: float,
    unknown_val: int,
    free_th: int,
    occ_th: int,
    radius_m: float,
) -> List[bool]:
    """Kobler BFS-graf til utforskede kjent-frie celler nær roboten (kun lokal maske)."""
    out = [bool(x) for x in pass_bfs]
    if radius_m <= 0.0:
        return out
    r_cells = int(math.ceil(float(radius_m) / max(res, 1e-9))) + 2
    mx0, my0 = world_to_map(rx, ry, ox, oy, res)
    for my in range(max(0, my0 - r_cells), min(h, my0 + r_cells + 1)):
        for mx in range(max(0, mx0 - r_cells), min(w, mx0 + r_cells + 1)):
            wx = ox + (mx + 0.5) * res
            wy = oy + (my + 0.5) * res
            if math.hypot(wx - rx, wy - ry) > radius_m + 1e-6:
                continue
            i = _idx(mx, my, w)
            if _is_free(int(data[i]), free_th, occ_th, unknown_val) and not out[i]:
                out[i] = True
    return out


def apply_robot_footprint_clearing(
    passable: Sequence[bool],
    rx: float,
    ry: float,
    ox: float,
    oy: float,
    res: float,
    w: int,
    h: int,
    radius_m: float,
) -> List[bool]:
    """Skrivbar kopi med ekstra passerbar skive rundt robotpose (kun i explorer)."""
    out = [bool(x) for x in passable]
    r_cells = int(math.ceil(float(radius_m) / max(res, 1e-9))) + 2
    mx0, my0 = world_to_map(rx, ry, ox, oy, res)
    for my in range(max(0, my0 - r_cells), min(h, my0 + r_cells + 1)):
        for mx in range(max(0, mx0 - r_cells), min(w, mx0 + r_cells + 1)):
            wx = ox + (mx + 0.5) * res
            wy = oy + (my + 0.5) * res
            if math.hypot(wx - rx, wy - ry) <= radius_m + 1e-6:
                out[_idx(mx, my, w)] = True
    return out


def resolve_bfs_seed_and_masks(
    robot_ixy: Tuple[int, int],
    robot_xy: Tuple[float, float],
    data: Sequence[int],
    w: int,
    h: int,
    passable_bfs_main: Sequence[bool],
    passable_bfs_staging: Sequence[bool],
    ox: float,
    oy: float,
    res: float,
    unknown_val: int,
    free_th: int,
    occ_th: int,
    bfs_seed_radius_m: float,
    bfs_seed_step_m: float,
    min_reachable_cells: int = 500,
) -> Tuple[Optional[Tuple[int, int]], List[bool], List[bool], float, str]:
    rx, ry = robot_xy
    seed, sd_m, smeth = find_nearest_bfs_seed(
        robot_ixy,
        rx,
        ry,
        data,
        passable_bfs_main,
        w,
        h,
        ox,
        oy,
        res,
        unknown_val,
        free_th,
        occ_th,
        bfs_seed_radius_m,
        bfs_seed_step_m,
    )
    if seed is None:
        gp, gd = global_nearest_passable_known_free_cell(
            rx, ry, data, passable_bfs_main, w, h, ox, oy, res, unknown_val, free_th, occ_th
        )
        if gp is not None:
            seed, sd_m, smeth = gp, gd, "global_nearest_passable_known_free"
    rm = _bfs_mask(seed, passable_bfs_main, w, h)
    rs = _bfs_mask(seed, passable_bfs_staging, w, h)
    if seed is not None and _count_reachable(rm) < min_reachable_cells:
        gp, gd = global_nearest_passable_known_free_cell(
            rx, ry, data, passable_bfs_main, w, h, ox, oy, res, unknown_val, free_th, occ_th
        )
        if gp is not None and gp != seed:
            rm_try = _bfs_mask(gp, passable_bfs_main, w, h)
            if _count_reachable(rm_try) > _count_reachable(rm):
                seed, sd_m, smeth = gp, gd, "seed_reseed_low_reach"
                rm = rm_try
                rs = _bfs_mask(seed, passable_bfs_staging, w, h)
    return seed, rm, rs, sd_m, smeth


def format_reachability_debug_line(
    robot_xy: Tuple[float, float],
    robot_ixy: Optional[Tuple[int, int]],
    data: Sequence[int],
    w: int,
    h: int,
    passable: Sequence[bool],
    passable_staging: Sequence[bool],
    ox: float,
    oy: float,
    res: float,
    occ_th: int,
    free_th: int,
    unknown_val: int,
    seed: Optional[Tuple[int, int]],
    seed_dist_m: float,
    seed_method: str,
    reachable_main: Sequence[bool],
    reachable_staging: Sequence[bool],
    local_radius_m: float,
) -> str:
    _ = (data, w, h, ox, oy, res, occ_th, free_th, unknown_val, passable, passable_staging)
    rx, ry = robot_xy
    ri = robot_ixy
    cnt_m = sum(1 for x in reachable_main if x)
    cnt_s = sum(1 for x in reachable_staging if x)
    return (
        f"REACHABILITY robot=({rx:.2f},{ry:.2f}) cell={ri} seed={seed} "
        f"seed_d={seed_dist_m:.2f}m method={seed_method} "
        f"reachable_cells={cnt_m} staging_reach={cnt_s} r_local={local_radius_m:.2f}"
    )


def score_cluster_distance(
    cl: FrontierCluster,
    robot_xy: Tuple[float, float],
    ox: float,
    oy: float,
    res: float,
    size_weight: float,
) -> float:
    cx, cy = cl.centroid_map
    wx = ox + (cx + 0.5) * res
    wy = oy + (cy + 0.5) * res
    dist = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
    return dist - size_weight * math.sqrt(float(cl.size))


def _blacklisted(wx: float, wy: float, blacklist: Sequence[Tuple[float, float, float]], rad: float) -> bool:
    for bx, by, _t in blacklist:
        if math.hypot(wx - bx, wy - by) < rad:
            return True
    return False


def format_strict_mask_debug(
    data: Sequence[int],
    w: int,
    h: int,
    passable: Sequence[bool],
    pass_bfs: Sequence[bool],
    reach_bfs: Sequence[bool],
    unk: int,
    free: int,
    occ: int,
) -> str:
    """Antall per filter som forklarer hvorfor reach_strict kan være 0 mens reach_bfs > 0."""
    n = w * h
    reachable_bfs_count = sum(1 for x in reach_bfs if x)
    after_obstacle_inflation_count = sum(1 for x in pass_bfs if x)
    after_unknown_clearance_count = 0
    after_goal_inflation_count = sum(1 for x in passable if x)
    for i in range(n):
        if not reach_bfs[i]:
            continue
        if _is_free(int(data[i]), free, occ, unk):
            after_unknown_clearance_count += 1
    final_reach_strict_count = sum(
        1 for i in range(n) if passable[i] and reach_bfs[i]
    )
    return (
        "STRICT_MASK_DEBUG "
        f"reachable_bfs_count={reachable_bfs_count} "
        f"after_obstacle_inflation_count={after_obstacle_inflation_count} "
        f"after_unknown_clearance_count={after_unknown_clearance_count} "
        f"after_goal_inflation_count={after_goal_inflation_count} "
        f"final_reach_strict_count={final_reach_strict_count}"
    )


def build_obstacle_distance_map(
    data: Sequence[int],
    w: int,
    h: int,
    occupied_threshold: int,
    resolution: float,
) -> List[float]:
    """Avstand per celle (m) til nærmeste okkuperte celle (4-nabo BFS)."""
    n = w * h
    dist_c = [10**9] * n
    q: deque[Tuple[int, int]] = deque()
    for my in range(h):
        for mx in range(w):
            i = _idx(mx, my, w)
            if _is_occupied(int(data[i]), occupied_threshold):
                dist_c[i] = 0
                q.append((mx, my))
    while q:
        mx, my = q.popleft()
        i = _idx(mx, my, w)
        d = dist_c[i]
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = mx + dx, my + dy
            if nx < 0 or ny < 0 or nx >= w or ny >= h:
                continue
            ni = _idx(nx, ny, w)
            if dist_c[ni] > d + 1:
                dist_c[ni] = d + 1
                q.append((nx, ny))
    res = max(resolution, 1e-9)
    return [float(d) * res for d in dist_c]


def build_unknown_distance_map(
    data: Sequence[int],
    w: int,
    h: int,
    unknown_value: int,
    resolution: float,
) -> List[float]:
    """Avstand per celle (m) til nærmeste ukjente celle (4-nabo BFS)."""
    n = w * h
    dist_c = [10**9] * n
    q: deque[Tuple[int, int]] = deque()
    for my in range(h):
        for mx in range(w):
            i = _idx(mx, my, w)
            if _is_unknown(int(data[i]), unknown_value):
                dist_c[i] = 0
                q.append((mx, my))
    while q:
        mx, my = q.popleft()
        i = _idx(mx, my, w)
        d = dist_c[i]
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = mx + dx, my + dy
            if nx < 0 or ny < 0 or nx >= w or ny >= h:
                continue
            ni = _idx(nx, ny, w)
            if dist_c[ni] > d + 1:
                dist_c[ni] = d + 1
                q.append((nx, ny))
    res = max(resolution, 1e-9)
    return [float(d) * res for d in dist_c]


def _min_clearance_in_disk(
    mx: int,
    my: int,
    obstacle_dist: Sequence[float],
    w: int,
    h: int,
    radius_m: float,
    res: float,
) -> float:
    """Minste hindringsklaring (m) innenfor en skive rundt (mx, my)."""
    r = max(1, int(math.ceil(radius_m / max(res, 1e-9))))
    best = float("inf")
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy > r * r:
                continue
            cx, cy = mx + dx, my + dy
            if cx < 0 or cy < 0 or cx >= w or cy >= h:
                return 0.0
            ni = _idx(cx, cy, w)
            if ni < len(obstacle_dist):
                best = min(best, obstacle_dist[ni])
    return best if math.isfinite(best) else 0.0


def _is_corner_goal(
    mx: int,
    my: int,
    obstacle_dist: Sequence[float],
    w: int,
    h: int,
    check_radius_m: float,
    min_free_space_m: float,
    resolution: float,
) -> bool:
    """True når okkupert ligger på flere sider innenfor check_radius_m."""
    res = max(resolution, 1e-9)
    step = max(1, int(math.ceil(check_radius_m / res)))
    blocked_sides = 0
    for dx, dy in ((0, step), (0, -step), (step, 0), (-step, 0)):
        nx, ny = mx + dx, my + dy
        if nx < 0 or ny < 0 or nx >= w or ny >= h:
            blocked_sides += 1
            continue
        ni = _idx(nx, ny, w)
        if obstacle_dist[ni] < min_free_space_m:
            blocked_sides += 1
    return blocked_sides >= 2


def _score_goal_candidate(
    dist_robot: float,
    cluster_size: int,
    max_cluster_size: int,
    obstacle_clearance: float,
    unknown_clearance: float,
    is_corner: bool,
    cfg: SafeGoalPickConfig,
    explored_revisit: bool = False,
    narrow_passage_penalty: float = 0.0,
) -> float:
    norm_size = float(cluster_size) / max(1.0, float(max_cluster_size))
    obs_term = min(obstacle_clearance, cfg.preferred_obstacle_clearance_m)
    unk_term = min(unknown_clearance, cfg.preferred_unknown_clearance_m)
    score = (
        cfg.score_cluster_weight * norm_size
        - cfg.score_distance_weight * dist_robot
        + cfg.score_obstacle_weight * obs_term
        + cfg.score_unknown_weight * unk_term
    )
    if is_corner and cfg.avoid_corner_goals:
        score -= cfg.corner_penalty
    if explored_revisit:
        score -= cfg.explored_revisit_penalty
    score -= narrow_passage_penalty
    return score


def _collect_cluster_candidates(
    cluster_id: int,
    cl: FrontierCluster,
    data: Sequence[int],
    w: int,
    h: int,
    goal_reach_mask: Sequence[bool],
    passable: Optional[Sequence[bool]],
    obstacle_dist: Sequence[float],
    unknown_dist: Sequence[float],
    robot_xy: Tuple[float, float],
    ox: float,
    oy: float,
    res: float,
    occ_th: int,
    free_th: int,
    unknown_val: int,
    blacklist: Sequence[Tuple[float, float, float]],
    blacklist_radius: float,
    cfg: SafeGoalPickConfig,
    min_clearance_m: float,
    max_cluster_size: int,
    explored_explored_fn: Optional[object] = None,
) -> Tuple[List[ScoredGoalCandidate], List[ScoredGoalCandidate]]:
    rx, ry = robot_xy
    safe: List[ScoredGoalCandidate] = []
    rejected: List[ScoredGoalCandidate] = []
    _max_rejected_sample = 300
    ccx, ccy = cl.centroid_map

    def _reject(cand: ScoredGoalCandidate) -> None:
        if len(rejected) < _max_rejected_sample:
            rejected.append(cand)
    annulus = _cells_in_annulus(
        ccx, ccy, w, h, ox, oy, res, cfg.approach_radius_min_m, cfg.approach_radius_max_m
    )
    seen: set[Tuple[int, int]] = set()

    def try_cell(mx: int, my: int, wx: float, wy: float, method: str) -> None:
        if (mx, my) in seen:
            return
        seen.add((mx, my))
        gi = _idx(mx, my, w)
        rej = _validate_approach_candidate(
            mx,
            my,
            data,
            w,
            h,
            goal_reach_mask,
            passable,
            cfg.require_passable_for_approach,
            occ_th,
            unknown_val,
            free_th,
            0.0,
            res,
        )
        if rej is not None:
            _reject(
                ScoredGoalCandidate(
                    cluster_id=cluster_id,
                    cluster=cl,
                    wx=wx,
                    wy=wy,
                    mx=mx,
                    my=my,
                    dist_robot=math.hypot(wx - rx, wy - ry),
                    obstacle_clearance=obstacle_dist[gi] if gi < len(obstacle_dist) else 0.0,
                    unknown_clearance=unknown_dist[gi] if gi < len(unknown_dist) else 0.0,
                    score=0.0,
                    is_corner=False,
                    method=method,
                    unsafe=True,
                    reject_reason=rej,
                )
            )
            return
        if _blacklisted(wx, wy, blacklist, blacklist_radius):
            return
        dr = math.hypot(wx - rx, wy - ry)
        if dr < cfg.min_goal_dist_m or dr > cfg.max_goal_dist_m:
            return
        obs_clr = obstacle_dist[gi] if gi < len(obstacle_dist) else 0.0
        unk_clr = unknown_dist[gi] if gi < len(unknown_dist) else 0.0
        is_corner = _is_corner_goal(
            mx, my, obstacle_dist, w, h, cfg.corner_check_radius_m,
            cfg.min_free_space_around_goal_m, res,
        )
        cand = ScoredGoalCandidate(
            cluster_id=cluster_id,
            cluster=cl,
            wx=wx,
            wy=wy,
            mx=mx,
            my=my,
            dist_robot=dr,
            obstacle_clearance=obs_clr,
            unknown_clearance=unk_clr,
            score=0.0,
            is_corner=is_corner,
            method=method,
        )
        if obs_clr < min_clearance_m:
            cand.unsafe = True
            cand.reject_reason = "low_obstacle_clearance"
            _reject(cand)
            return
        passage_clr = _min_clearance_in_disk(
            mx, my, obstacle_dist, w, h, cfg.min_passage_clearance_m, res
        )
        if passage_clr < cfg.min_passage_clearance_floor_m:
            cand.unsafe = True
            cand.reject_reason = "narrow_passage"
            _reject(cand)
            return
        narrow_pen = 0.0
        if passage_clr < min_clearance_m:
            narrow_pen = (
                min_clearance_m - passage_clr
            ) * cfg.narrow_passage_penalty_weight
        if unk_clr < cfg.min_unknown_clearance_m:
            cand.unsafe = True
            cand.reject_reason = "low_unknown_clearance"
            _reject(cand)
            return
        if is_corner and cfg.avoid_corner_goals:
            cand.unsafe = True
            cand.reject_reason = "corner"
            _reject(cand)
            return
        revisited = False
        if explored_explored_fn is not None and explored_explored_fn(mx, my):
            revisited = unk_clr > cfg.preferred_unknown_clearance_m * 0.5
        cand.score = _score_goal_candidate(
            dr,
            cl.size,
            max_cluster_size,
            obs_clr,
            unk_clr,
            is_corner,
            cfg,
            revisited,
            narrow_pen,
        )
        safe.append(cand)

    for _dc, wx, wy, mx, my in annulus:
        try_cell(mx, my, wx, wy, "centroid_annulus")

    r_cells = int(math.ceil(cfg.neighbor_fallback_m / max(res, 1e-9))) + 1
    frontier_cells = list(cl.cells)
    if len(frontier_cells) > 48:
        step = max(1, len(frontier_cells) // 48)
        frontier_cells = frontier_cells[::step][:48]
    for fmx, fmy in frontier_cells:
        for my in range(max(0, fmy - r_cells), min(h, fmy + r_cells + 1)):
            for mx in range(max(0, fmx - r_cells), min(w, fmx + r_cells + 1)):
                if (mx, my) in cl.cells:
                    continue
                wx = ox + (mx + 0.5) * res
                wy = oy + (my + 0.5) * res
                if math.hypot(
                    wx - (ox + (fmx + 0.5) * res),
                    wy - (oy + (fmy + 0.5) * res),
                ) > cfg.neighbor_fallback_m + 1e-6:
                    continue
                try_cell(mx, my, wx, wy, "neighbor_fallback")

    return safe, rejected


def select_best_frontier_goal(
    ranked_clusters: Sequence[FrontierCluster],
    data: Sequence[int],
    w: int,
    h: int,
    robot_xy: Tuple[float, float],
    ox: float,
    oy: float,
    res: float,
    occ_th: int,
    free_th: int,
    unknown_val: int,
    passable: Sequence[bool],
    goal_reach_mask: Sequence[bool],
    blacklist: Sequence[Tuple[float, float, float]],
    blacklist_radius: float,
    cfg: SafeGoalPickConfig,
    min_clearance_override_m: Optional[float] = None,
    cluster_blacklist: Optional[Sequence[Tuple[float, float, float]]] = None,
    explored_explored_fn: Optional[object] = None,
) -> SafeGoalPickResult:
    """Scorer alle klynger; velger trygg tilnærmingcelle med høyest score."""
    obstacle_dist = build_obstacle_distance_map(data, w, h, occ_th, res)
    unknown_dist = build_unknown_distance_map(data, w, h, unknown_val, res)
    max_cl = max((cl.size for cl in ranked_clusters), default=1)
    min_clr = (
        min_clearance_override_m
        if min_clearance_override_m is not None
        else cfg.min_obstacle_clearance_m
    )
    relaxed = False
    all_safe: List[ScoredGoalCandidate] = []
    all_rejected: List[ScoredGoalCandidate] = []
    cl_bl = cluster_blacklist or []

    for cluster_id, cl in enumerate(ranked_clusters):
        ccx, ccy = cl.centroid_map
        cwx = ox + (ccx + 0.5) * res
        cwy = oy + (ccy + 0.5) * res
        if _blacklisted(cwx, cwy, cl_bl, blacklist_radius):
            continue
        s, r = _collect_cluster_candidates(
            cluster_id,
            cl,
            data,
            w,
            h,
            goal_reach_mask,
            passable,
            obstacle_dist,
            unknown_dist,
            robot_xy,
            ox,
            oy,
            res,
            occ_th,
            free_th,
            unknown_val,
            blacklist,
            blacklist_radius,
            cfg,
            min_clr,
            max_cl,
            explored_explored_fn,
        )
        all_safe.extend(s)
        all_rejected.extend(r)

    clearance_used = min_clr
    if not all_safe and min_clr > cfg.min_obstacle_clearance_floor_m:
        relaxed = True
        clearance_used = cfg.min_obstacle_clearance_floor_m
        all_safe = []
        all_rejected = []
        for cluster_id, cl in enumerate(ranked_clusters):
            ccx, ccy = cl.centroid_map
            cwx = ox + (ccx + 0.5) * res
            cwy = oy + (ccy + 0.5) * res
            if _blacklisted(cwx, cwy, cl_bl, blacklist_radius):
                continue
            s, r = _collect_cluster_candidates(
                cluster_id,
                cl,
                data,
                w,
                h,
                goal_reach_mask,
                passable,
                obstacle_dist,
                unknown_dist,
                robot_xy,
                ox,
                oy,
                res,
                occ_th,
                free_th,
                unknown_val,
                blacklist,
                blacklist_radius,
                cfg,
                clearance_used,
                max_cl,
                explored_explored_fn,
            )
            all_safe.extend(s)
            all_rejected.extend(r)

    if not all_safe and cfg.avoid_corner_goals and cfg.allow_corner_fallback:
        loose = replace(cfg, avoid_corner_goals=False)
        for cluster_id, cl in enumerate(ranked_clusters):
            ccx, ccy = cl.centroid_map
            cwx = ox + (ccx + 0.5) * res
            cwy = oy + (ccy + 0.5) * res
            if _blacklisted(cwx, cwy, cl_bl, blacklist_radius):
                continue
            s, r = _collect_cluster_candidates(
                cluster_id,
                cl,
                data,
                w,
                h,
                goal_reach_mask,
                passable,
                obstacle_dist,
                unknown_dist,
                robot_xy,
                ox,
                oy,
                res,
                occ_th,
                free_th,
                unknown_val,
                blacklist,
                blacklist_radius,
                loose,
                clearance_used,
                max_cl,
                explored_explored_fn,
            )
            all_safe.extend(s)
            all_rejected.extend(r)

    if not all_safe:
        return SafeGoalPickResult(
            goal=None,
            safe_candidates=[],
            rejected_unsafe=all_rejected,
            clearance_relaxed=relaxed,
            clearance_used_m=clearance_used,
        )

    all_safe.sort(key=lambda c: c.score, reverse=True)
    best = all_safe[0]
    ccx, ccy = best.cluster.centroid_map
    cwx = ox + (ccx + 0.5) * res
    cwy = oy + (ccy + 0.5) * res
    yaw = math.atan2(cwy - best.wy, cwx - best.wx)
    vg = ValidatedGoal(
        wx=best.wx,
        wy=best.wy,
        yaw=yaw,
        approach_ixy=(best.mx, best.my),
        cluster=best.cluster,
        approach_method=best.method,
        obstacle_clearance=best.obstacle_clearance,
        unknown_clearance=best.unknown_clearance,
        score=best.score,
        cluster_id=best.cluster_id,
    )
    return SafeGoalPickResult(
        goal=vg,
        safe_candidates=all_safe,
        rejected_unsafe=all_rejected,
        clearance_relaxed=relaxed,
        clearance_used_m=clearance_used,
    )


def format_goal_candidates_log(candidates: Sequence[ScoredGoalCandidate], limit: int = 5) -> str:
    lines = ["GOAL_CANDIDATES:"]
    for c in candidates[:limit]:
        lines.append(
            f"id={c.cluster_id} world=({c.wx:.2f},{c.wy:.2f}) dist={c.dist_robot:.2f} "
            f"obstacle_clearance={c.obstacle_clearance:.2f} score={c.score:.2f}"
        )
    return "\n".join(lines)


def build_goal_reach_mask(
    passable: Sequence[bool],
    reach_bfs: Sequence[bool],
    require_strict: bool,
) -> List[bool]:
    """V1: bruk BFS-reach når strict-maske er tom eller strict reach ikke kreves."""
    n = min(len(passable), len(reach_bfs))
    reach_strict_count = sum(
        1 for i in range(n) if passable[i] and reach_bfs[i]
    )
    reach_bfs_count = sum(1 for x in reach_bfs if x)
    if not require_strict or (reach_strict_count == 0 and reach_bfs_count > 0):
        return [bool(reach_bfs[i]) for i in range(n)]
    return [bool(passable[i] and reach_bfs[i]) for i in range(n)]


def _occupied_clearance_ok(
    gix: int,
    giy: int,
    data: Sequence[int],
    w: int,
    h: int,
    occ_th: int,
    clearance_m: float,
    res: float,
) -> bool:
    if clearance_m <= 0.0:
        return True
    r = int(math.ceil(clearance_m / max(res, 1e-9)))
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            cx, cy = gix + dx, giy + dy
            if cx < 0 or cy < 0 or cx >= w or cy >= h:
                continue
            if _is_occupied(int(data[_idx(cx, cy, w)]), occ_th):
                return False
    return True


def _validate_approach_candidate(
    gix: int,
    giy: int,
    data: Sequence[int],
    w: int,
    h: int,
    goal_reach_mask: Sequence[bool],
    passable: Optional[Sequence[bool]],
    require_passable: bool,
    occ_th: int,
    unknown_val: int,
    free_th: int,
    min_occupied_clearance_m: float,
    res: float,
) -> Optional[str]:
    if gix < 0 or giy < 0 or gix >= w or giy >= h:
        return "out_of_bounds"
    gi = _idx(gix, giy, w)
    v = int(data[gi])
    if _is_unknown(v, unknown_val):
        return "unknown"
    if _is_occupied(v, occ_th):
        return "occupied"
    if not _is_free(v, free_th, occ_th, unknown_val):
        return "not_free"
    if gi >= len(goal_reach_mask) or not goal_reach_mask[gi]:
        return "not_reachable"
    if require_passable and passable is not None:
        if gi >= len(passable) or not passable[gi]:
            return "not_passable"
    if not _occupied_clearance_ok(
        gix, giy, data, w, h, occ_th, min_occupied_clearance_m, res
    ):
        return "too_close_obstacle"
    _ = free_th
    return None


def _cells_in_annulus(
    cx: float,
    cy: float,
    w: int,
    h: int,
    ox: float,
    oy: float,
    res: float,
    radius_min_m: float,
    radius_max_m: float,
) -> List[Tuple[float, float, int, int]]:
    """Returnerer (avstand_til_sentroid_m, wx, wy, mx, my) for kartceller i ring."""
    r_min_c = int(math.floor(radius_min_m / max(res, 1e-9)))
    r_max_c = int(math.ceil(radius_max_m / max(res, 1e-9))) + 1
    mx0 = int(math.floor(cx))
    my0 = int(math.floor(cy))
    out: List[Tuple[float, float, int, int]] = []
    for my in range(max(0, my0 - r_max_c), min(h, my0 + r_max_c + 1)):
        for mx in range(max(0, mx0 - r_max_c), min(w, mx0 + r_max_c + 1)):
            wx = ox + (mx + 0.5) * res
            wy = oy + (my + 0.5) * res
            d = math.hypot(wx - (ox + (cx + 0.5) * res), wy - (oy + (cy + 0.5) * res))
            if d < radius_min_m - 1e-6 or d > radius_max_m + 1e-6:
                continue
            out.append((d, wx, wy, mx, my))
    return out


def pick_cluster_approach_cell(
    cl: FrontierCluster,
    data: Sequence[int],
    w: int,
    h: int,
    goal_reach_mask: Sequence[bool],
    passable: Optional[Sequence[bool]],
    robot_xy: Tuple[float, float],
    ox: float,
    oy: float,
    res: float,
    occ_th: int,
    free_th: int,
    unknown_val: int,
    blacklist: Sequence[Tuple[float, float, float]],
    blacklist_radius: float,
    min_robot_dist_m: float,
    max_robot_dist_m: float,
    approach_radius_min_m: float,
    approach_radius_max_m: float,
    min_occupied_clearance_m: float,
    require_passable_for_approach: bool,
) -> Tuple[Optional[Tuple[int, int]], str, ApproachPickStats]:
    """V1: søk kjent-frie reachbare celler i ring rundt klyngesentroid."""
    rx, ry = robot_xy
    ccx, ccy = cl.centroid_map
    stats = ApproachPickStats()
    annulus = _cells_in_annulus(
        ccx, ccy, w, h, ox, oy, res, approach_radius_min_m, approach_radius_max_m
    )
    stats.candidates_total = len(annulus)
    valid: List[Tuple[float, float, int, int]] = []
    for _dc, wx, wy, mx, my in annulus:
        rej = _validate_approach_candidate(
            mx,
            my,
            data,
            w,
            h,
            goal_reach_mask,
            passable,
            require_passable_for_approach,
            occ_th,
            unknown_val,
            free_th,
            min_occupied_clearance_m,
            res,
        )
        if rej is not None:
            if rej == "unknown":
                stats.rejected_unknown += 1
            elif rej == "occupied":
                stats.rejected_occupied += 1
            elif rej == "not_reachable":
                stats.rejected_not_reachable += 1
            elif rej == "not_passable":
                stats.rejected_not_passable += 1
            elif rej == "too_close_obstacle":
                stats.rejected_too_close_obstacle += 1
            elif rej == "out_of_bounds":
                stats.rejected_out_of_bounds += 1
            continue
        if _blacklisted(wx, wy, blacklist, blacklist_radius):
            stats.rejected_blacklist += 1
            continue
        dr = math.hypot(wx - rx, wy - ry)
        if dr < min_robot_dist_m or dr > max_robot_dist_m:
            stats.rejected_robot_distance += 1
            continue
        valid.append((_dc, wx, wy, mx, my))
    if not valid:
        return None, "no_cluster_approach", stats
    valid.sort(key=lambda t: t[0])
    _dc, _wx, _wy, mx, my = valid[0]
    return (mx, my), "centroid_annulus", stats


def pick_cluster_neighbor_fallback(
    cl: FrontierCluster,
    data: Sequence[int],
    w: int,
    h: int,
    goal_reach_mask: Sequence[bool],
    passable: Optional[Sequence[bool]],
    robot_xy: Tuple[float, float],
    ox: float,
    oy: float,
    res: float,
    occ_th: int,
    free_th: int,
    unknown_val: int,
    blacklist: Sequence[Tuple[float, float, float]],
    blacklist_radius: float,
    min_robot_dist_m: float,
    max_robot_dist_m: float,
    neighbor_radius_m: float,
    min_occupied_clearance_m: float,
    require_passable_for_approach: bool,
) -> Tuple[Optional[Tuple[int, int]], str, ApproachPickStats]:
    """Nærmeste reachbare kjent-frie celle innenfor neighbor_radius_m av en frontier-celle."""
    rx, ry = robot_xy
    stats = ApproachPickStats()
    r_cells = int(math.ceil(neighbor_radius_m / max(res, 1e-9))) + 1
    best: Optional[Tuple[float, float, int, int]] = None
    for fmx, fmy in cl.cells:
        for my in range(max(0, fmy - r_cells), min(h, fmy + r_cells + 1)):
            for mx in range(max(0, fmx - r_cells), min(w, fmx + r_cells + 1)):
                if (mx, my) in cl.cells:
                    continue
                stats.candidates_total += 1
                wx = ox + (mx + 0.5) * res
                wy = oy + (my + 0.5) * res
                if math.hypot(wx - (ox + (fmx + 0.5) * res), wy - (oy + (fmy + 0.5) * res)) > (
                    neighbor_radius_m + 1e-6
                ):
                    continue
                rej = _validate_approach_candidate(
                    mx,
                    my,
                    data,
                    w,
                    h,
                    goal_reach_mask,
                    passable,
                    require_passable_for_approach,
                    occ_th,
                    unknown_val,
                    free_th,
                    min_occupied_clearance_m,
                    res,
                )
                if rej is not None:
                    if rej == "unknown":
                        stats.rejected_unknown += 1
                    elif rej == "occupied":
                        stats.rejected_occupied += 1
                    elif rej == "not_reachable":
                        stats.rejected_not_reachable += 1
                    elif rej == "not_passable":
                        stats.rejected_not_passable += 1
                    elif rej == "too_close_obstacle":
                        stats.rejected_too_close_obstacle += 1
                    continue
                if _blacklisted(wx, wy, blacklist, blacklist_radius):
                    stats.rejected_blacklist += 1
                    continue
                dr = math.hypot(wx - rx, wy - ry)
                if dr < min_robot_dist_m or dr > max_robot_dist_m:
                    stats.rejected_robot_distance += 1
                    continue
                if best is None or dr < best[0]:
                    best = (dr, wx, wy, mx, my)
    if best is None:
        return None, "neighbor_fallback_failed", stats
    return (best[3], best[4]), "neighbor_fallback", stats


def validate_and_build_goal(
    cl: FrontierCluster,
    data: Sequence[int],
    w: int,
    h: int,
    robot_xy: Tuple[float, float],
    robot_ixy: Optional[Tuple[int, int]],
    ox: float,
    oy: float,
    res: float,
    passable: Sequence[bool],
    goal_reach_mask: Sequence[bool],
    occ_th: int,
    free_th: int,
    unknown_val: int,
    blacklist: Sequence[Tuple[float, float, float]],
    blacklist_radius: float,
    min_robot_dist_m: float,
    max_robot_dist_m: float,
    approach_radius_min_m: float,
    approach_radius_max_m: float,
    min_occupied_clearance_m: float,
    require_passable_for_approach: bool,
    neighbor_fallback_m: float,
    _fallback_radius_min_m: float,
    _fallback_radius_max_m: float,
    enable_staging_goal: bool,
    _staging_min_distance_m: float,
    _staging_max_distance_m: float,
    _staging_fan_angles_deg: Optional[Tuple[float, ...]],
    _staging_fan_distances_m: Optional[Tuple[float, ...]],
    _staging_require_free_value_zero: bool,
    passable_staging: Optional[List[bool]],
    reachable_staging_mask: Optional[List[bool]],
) -> Tuple[Optional[ValidatedGoal], Optional[RejectReason], str, ApproachPickStats]:
    _ = (
        robot_ixy,
        enable_staging_goal,
        _staging_fan_angles_deg,
        _staging_fan_distances_m,
        _staging_require_free_value_zero,
        passable_staging,
        reachable_staging_mask,
        _fallback_radius_min_m,
        _fallback_radius_max_m,
    )
    ccx, ccy = cl.centroid_map
    cwx = ox + (ccx + 0.5) * res
    cwy = oy + (ccy + 0.5) * res

    picked, method, stats = pick_cluster_approach_cell(
        cl,
        data,
        w,
        h,
        goal_reach_mask,
        passable,
        robot_xy,
        ox,
        oy,
        res,
        occ_th,
        free_th,
        unknown_val,
        blacklist,
        blacklist_radius,
        min_robot_dist_m,
        max_robot_dist_m,
        approach_radius_min_m,
        approach_radius_max_m,
        min_occupied_clearance_m,
        require_passable_for_approach,
    )
    if picked is None:
        picked, method, stats = pick_cluster_neighbor_fallback(
            cl,
            data,
            w,
            h,
            goal_reach_mask,
            passable,
            robot_xy,
            ox,
            oy,
            res,
            occ_th,
            free_th,
            unknown_val,
            blacklist,
            blacklist_radius,
            min_robot_dist_m,
            max_robot_dist_m,
            neighbor_fallback_m,
            min_occupied_clearance_m,
            require_passable_for_approach,
        )
        if picked is not None:
            method = "CLUSTER_APPROACH_FAILED: using nearest reachable free neighbor fallback"
    if picked is None:
        return None, RejectReason.NO_APPROACH_CELL, stats.summary_line(), stats
    gix, giy = picked
    wx = ox + (gix + 0.5) * res
    wy = oy + (giy + 0.5) * res

    yaw = math.atan2(cwy - wy, cwx - wx)
    return (
        ValidatedGoal(
            wx=wx,
            wy=wy,
            yaw=yaw,
            approach_ixy=(gix, giy),
            cluster=cl,
            approach_method=method,
        ),
        None,
        method,
        stats,
    )
