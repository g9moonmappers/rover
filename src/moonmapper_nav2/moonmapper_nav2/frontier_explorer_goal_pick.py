"""Frontier målvalg fra /map (BFS, klynger, trygg approach)."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Optional, Tuple

from moonmapper_nav2.frontier_exploration_memory import map_unknown_fraction
from moonmapper_nav2.frontier_explorer_debug_bridge import log_map_pick_debug, publish_debug_markers
from moonmapper_nav2.frontier_explorer_markers import publish_frontier_markers
from moonmapper_nav2.frontier_explorer_params import safe_goal_config_from_node
from moonmapper_nav2.frontier_grid import (
    ValidatedGoal,
    apply_robot_footprint_clearing,
    bridge_passable_bfs_to_known_free,
    build_goal_reach_mask,
    build_passable_mask,
    collect_frontier_clusters,
    count_raw_frontier_cells,
    format_goal_candidates_log,
    format_reachability_debug_line,
    format_strict_mask_debug,
    global_nearest_free_cell,
    resolve_bfs_seed_and_masks,
    score_cluster_distance,
    select_best_frontier_goal,
)
from moonmapper_nav2.frontier_utils import sanitize_occ_grid_data, world_to_map

if TYPE_CHECKING:
    from moonmapper_nav2.frontier_explorer import FrontierExplorer


def pick_frontier_goal(explorer: "FrontierExplorer") -> Tuple[Optional[ValidatedGoal], str]:
    if explorer._map is None:
        return None, "no_map"
    pose = explorer._pose_map()
    if pose is None:
        return None, "no_tf"
    rx, ry, _ = pose
    info = explorer._map.info
    w, h = int(info.width), int(info.height)
    if w <= 0 or h <= 0:
        return None, "empty"

    lg = explorer.get_logger()
    ox = float(info.origin.position.x)
    oy = float(info.origin.position.y)
    res = float(info.resolution)
    occ = int(explorer.get_parameter("occupied_threshold").value)
    free = int(explorer.get_parameter("free_threshold").value)
    unk = int(explorer.get_parameter("unknown_value").value)
    mins = int(explorer.get_parameter("min_frontier_cluster_size").value)

    data = sanitize_occ_grid_data(explorer._map.data)
    log_map_pick_debug(explorer, data, rx, ry, w, h, ox, oy, res, unk, free, occ)

    raw_frontier_cells = count_raw_frontier_cells(data, w, h, unk, free, occ)
    clusters = collect_frontier_clusters(data, w, h, unk, free, occ, mins)
    n_cl = len(clusters)

    unk_blocked = bool(explorer.get_parameter("unknown_as_blocked_in_planner_grid").value)
    goal_infl = float(explorer.get_parameter("goal_inflation_radius_m").value)
    bfs_infl = float(explorer.get_parameter("bfs_goal_inflation_radius_m").value)
    passable, _ = build_passable_mask(
        data, w, h, unk, occ, free, unk_blocked, goal_infl, res
    )
    pass_bfs, _ = build_passable_mask(
        data, w, h, unk, occ, free, unk_blocked, bfs_infl, res
    )
    p_st = passable
    pass_st_bfs = list(pass_bfs)
    if bool(explorer.get_parameter("enable_staging_goal").value):
        st_infl = goal_infl + float(explorer.get_parameter("staging_clearance_m").value)
        p_st, _ = build_passable_mask(
            data, w, h, unk, occ, free, unk_blocked, st_infl, res
        )
        pass_st_bfs, _ = build_passable_mask(
            data,
            w,
            h,
            unk,
            occ,
            free,
            unk_blocked,
            bfs_infl + float(explorer.get_parameter("staging_clearance_m").value),
            res,
        )
    used_clear = False
    if bool(explorer.get_parameter("allow_robot_seed_clearing").value):
        cr = float(explorer.get_parameter("robot_seed_clear_radius_m").value)
        pass_bfs = apply_robot_footprint_clearing(pass_bfs, rx, ry, ox, oy, res, w, h, cr)
        pass_st_bfs = apply_robot_footprint_clearing(pass_st_bfs, rx, ry, ox, oy, res, w, h, cr)
        used_clear = True
        lg.info(
            f"SEED_FALLBACK robot footprint clearing used radius={cr:.2f}m "
            "(local BFS mask only; /map not modified)"
        )

    bridge_r = float(explorer.get_parameter("bfs_bridge_known_free_radius_m").value)
    pass_bfs = bridge_passable_bfs_to_known_free(
        pass_bfs, data, w, h, rx, ry, ox, oy, res, unk, free, occ, bridge_r
    )
    if bool(explorer.get_parameter("enable_staging_goal").value):
        pass_st_bfs = bridge_passable_bfs_to_known_free(
            pass_st_bfs, data, w, h, rx, ry, ox, oy, res, unk, free, occ, bridge_r
        )

    robot_ixy = world_to_map(rx, ry, ox, oy, res)
    bfs_r = float(explorer.get_parameter("bfs_seed_search_radius_m").value)
    bfs_step = float(explorer.get_parameter("bfs_seed_search_step_m").value)
    min_reach = int(explorer.get_parameter("min_reachable_cells_for_bfs").value)
    seed, rm, rs, sd_m, smeth = resolve_bfs_seed_and_masks(
        robot_ixy,
        (rx, ry),
        data,
        w,
        h,
        pass_bfs,
        pass_st_bfs,
        ox,
        oy,
        res,
        unk,
        free,
        occ,
        bfs_r,
        bfs_step,
        min_reach,
    )
    reach_bfs = sum(1 for x in rm if x)
    reach_strict = sum(1 for i, p in enumerate(passable) if p and i < len(rm) and rm[i])
    lg.info(
        f"REACH_COUNTS reach_bfs={reach_bfs} reach_strict={reach_strict} "
        f"bfs_inflation_m={bfs_infl:.2f} goal_inflation_m={goal_infl:.2f}"
    )
    lg.info(format_strict_mask_debug(data, w, h, passable, pass_bfs, rm, unk, free, occ))
    require_strict = bool(explorer.get_parameter("require_strict_reachability_for_goal").value)
    goal_reach = build_goal_reach_mask(passable, rm, require_strict)
    if reach_strict == 0 and reach_bfs > 0 and not require_strict:
        lg.info("STRICT_REACH_EMPTY: falling back to BFS reachability for V1")
    if seed is not None and smeth in (
        "global_nearest_passable_known_free",
        "seed_reseed_low_reach",
    ):
        lg.info(
            f"SEED_FALLBACK {smeth} cell={seed} distance_m={sd_m:.2f}m "
            f"reach_bfs={reach_bfs}"
        )
    lg.info(
        format_reachability_debug_line(
            (rx, ry),
            robot_ixy,
            data,
            w,
            h,
            passable,
            p_st,
            ox,
            oy,
            res,
            occ,
            free,
            unk,
            seed,
            sd_m,
            smeth,
            rm,
            rs,
            1.0,
        )
    )

    if seed is None:
        lg.error("NO_BFS_SEED: no passable BFS seed within configured radii")
        free_c = sum(
            1 for vv in data if int(vv) != unk and int(vv) >= free and int(vv) < occ
        )
        if free_c > 0:
            gf, gd = global_nearest_free_cell(rx, ry, data, w, h, ox, oy, res, unk, free, occ)
            if gf is not None:
                lg.info(
                    f"nearest_free_global cell={gf} distance_m={gd:.2f} "
                    "(raw occupancy free per MAP_STATS; may still be blocked in passable "
                    "mask after inflation - use global_nearest_passable_bfs / REACHABILITY for planner graph)"
                )
        elif free_c == 0:
            lg.warning("no_free_cells: MAP_STATS free_count=0")

    if not clusters:
        reason = "no_raw_frontiers" if raw_frontier_cells == 0 else "no_clusters"
        lg.info(
            "FRONTIER_DEBUG "
            f"raw_cells={raw_frontier_cells} clusters={n_cl} after_distance=0 "
            f"after_reachable=0 after_blacklist=0 reason={reason}"
        )
        publish_frontier_markers(explorer, explorer._pub_mk, [], None, explorer._map)
        return None, "no_clusters"

    if seed is None:
        publish_frontier_markers(explorer, explorer._pub_mk, clusters, None, explorer._map)
        lg.info(
            "FRONTIER_DEBUG "
            f"raw_cells={raw_frontier_cells} clusters={n_cl} after_distance=0 "
            f"after_reachable=0 after_blacklist=0 reason=no_seed_near_robot"
        )
        return None, "no_seed"

    sw = float(explorer.get_parameter("size_weight").value)
    ranked = sorted(
        clusters,
        key=lambda cl: score_cluster_distance(cl, (rx, ry), ox, oy, res, sw),
    )
    max_cl_score = int(explorer.get_parameter("max_clusters_to_score").value)
    if max_cl_score > 0:
        ranked = ranked[:max_cl_score]
    cluster_cd = float(explorer.get_parameter("goal_reselect_cooldown_sec").value)
    ranked_avail = [
        cl
        for cid, cl in enumerate(ranked)
        if not explorer._cluster_recently_sent(cid, cluster_cd)
    ]
    if ranked_avail:
        ranked = ranked_avail
    explorer._prune_bl()
    br = float(explorer.get_parameter("blacklist_radius_m").value)
    if br <= 0.0:
        br = float(explorer.get_parameter("blacklist_radius").value)
    relax_after = int(explorer.get_parameter("allow_relaxed_goal_after_attempts").value)
    min_clr_override = explorer._clearance_override_m
    if min_clr_override is None and explorer._search_fail_attempts >= relax_after:
        min_clr_override = float(
            explorer.get_parameter("min_obstacle_clearance_floor_m").value
        )
    explorer._last_n_clusters = n_cl
    unk_frac, _uk, _fr, _oc = map_unknown_fraction(data, unk, free, occ)
    explorer._last_unknown_fraction = unk_frac
    safe_cfg = safe_goal_config_from_node(explorer)
    explored_fn = explorer._explored_cell_fn(w, h)
    pick = select_best_frontier_goal(
        ranked,
        data,
        w,
        h,
        (rx, ry),
        ox,
        oy,
        res,
        occ,
        free,
        unk,
        passable,
        goal_reach,
        explorer._blacklist + explorer._blacklist_failed,
        br,
        safe_cfg,
        min_clearance_override_m=min_clr_override,
        cluster_blacklist=explorer._blacklist_clusters,
        explored_explored_fn=explored_fn,
    )
    explorer._clearance_override_m = None
    if explorer._explored_memory is not None:
        lg.info(
            f"EXPLORED_MEMORY fraction={explorer._explored_memory.explored_fraction():.3f} "
            f"unknown_fraction={unk_frac:.3f}"
        )
    if pick.clearance_relaxed:
        lg.warn(
            "SAFE_GOAL_FALLBACK clearance relaxed from "
            f"{safe_cfg.min_obstacle_clearance_m:.2f}m to {pick.clearance_used_m:.2f}m"
        )
    if pick.goal is not None and pick.goal.obstacle_clearance < safe_cfg.min_obstacle_clearance_m:
        lg.warn(
            "GOAL_LOW_CLEARANCE selected_clearance="
            f"{pick.goal.obstacle_clearance:.2f}m (min={safe_cfg.min_obstacle_clearance_m:.2f}m)"
        )

    publish_debug_markers(
        explorer, clusters, pick.goal, pick.safe_candidates, pick.rejected_unsafe
    )

    if pick.goal is not None:
        vg = pick.goal
        if pick.safe_candidates:
            lg.info(format_goal_candidates_log(pick.safe_candidates, limit=5))
        pref = float(explorer.get_parameter("preferred_obstacle_clearance_m").value)
        if vg.obstacle_clearance < pref:
            lg.warn(f"GOAL_LOW_CLEARANCE selected_clearance={vg.obstacle_clearance:.2f}m")
        dist_robot = math.hypot(vg.wx - rx, vg.wy - ry)
        lg.info(
            f"GOAL_SELECTED cluster_id={vg.cluster_id} approach_cell={vg.approach_ixy} "
            f"world=({vg.wx:.2f},{vg.wy:.2f}) dist_robot={dist_robot:.2f}m "
            f"obstacle_clearance={vg.obstacle_clearance:.2f}m score={vg.score:.2f} "
            f"method={vg.approach_method}"
        )
        lg.info("SENDING_NAV2_GOAL")
        lg.info(
            "FRONTIER_DEBUG "
            f"raw_cells={raw_frontier_cells} clusters={n_cl} "
            f"candidates={len(pick.safe_candidates)} rejected_unsafe={len(pick.rejected_unsafe)} "
            f"reason=goal_selected used_seed_clearing={used_clear}"
        )
        publish_frontier_markers(explorer, explorer._pub_mk, clusters, vg, explorer._map)
        return vg, ""

    lg.warn(
        "NO_SAFE_GOAL: no approach met clearance/distance/corner rules "
        f"(rejected_unsafe={len(pick.rejected_unsafe)} clearance_used={pick.clearance_used_m:.2f}m)"
    )
    lg.info(
        "FRONTIER_DEBUG "
        f"raw_cells={raw_frontier_cells} clusters={n_cl} "
        f"candidates=0 rejected_unsafe={len(pick.rejected_unsafe)} reason=no_safe_goal"
    )
    publish_frontier_markers(explorer, explorer._pub_mk, clusters, None, explorer._map)
    return None, "no_valid"
