"""Hukommelse for besøkte celler og når utforskning er ferdig."""

from __future__ import annotations

from typing import Optional, Sequence, Tuple


class ExploredMemory:
    """Rutenett over celler roboten har besøkt varer for en utforskingskjøring."""

    def __init__(self, width: int, height: int) -> None:
        self._w = max(1, int(width))
        self._h = max(1, int(height))
        self._cells = bytearray(self._w * self._h)

    def resize(self, width: int, height: int) -> None:
        nw, nh = max(1, int(width)), max(1, int(height))
        if nw == self._w and nh == self._h:
            return
        self._w, self._h = nw, nh
        self._cells = bytearray(nw * nh)

    def mark_world(
        self,
        wx: float,
        wy: float,
        radius_m: float,
        ox: float,
        oy: float,
        res: float,
        w: int,
        h: int,
    ) -> None:
        if w != self._w or h != self._h:
            self.resize(w, h)
        res = max(res, 1e-9)
        mx = int((wx - ox) / res)
        my = int((wy - oy) / res)
        r = int(max(1, round(radius_m / res)))
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r * r:
                    continue
                cx, cy = mx + dx, my + dy
                if 0 <= cx < w and 0 <= cy < h:
                    self._cells[cy * w + cx] = 1

    def is_explored(self, mx: int, my: int, w: int, h: int) -> bool:
        if mx < 0 or my < 0 or mx >= w or my >= h:
            return False
        if w != self._w or h != self._h:
            return False
        return bool(self._cells[my * w + mx])

    def explored_fraction(self) -> float:
        if not self._cells:
            return 0.0
        return sum(self._cells) / float(len(self._cells))


def map_unknown_fraction(
    data: Sequence[int],
    unknown_value: int,
    free_threshold: int,
    occupied_threshold: int,
) -> Tuple[float, int, int, int]:
    """Returnerer (ukjent_andel, antall_ukjent, antall_fri, antall_okkupert)."""
    unk_c = free_c = occ_c = 0
    for v in data:
        iv = int(v)
        if iv == unknown_value:
            unk_c += 1
        elif iv >= occupied_threshold:
            occ_c += 1
        elif iv >= free_threshold:
            free_c += 1
    total = max(1, len(data))
    return float(unk_c) / float(total), unk_c, free_c, occ_c


def exploration_complete(
    n_frontier_clusters: int,
    unknown_fraction: float,
    max_unknown_fraction: float,
    min_frontier_clusters_to_continue: int,
) -> Tuple[bool, str]:
    """True når det ikke er meningsfulle frontiers igjen og kartet er stort sett kjent."""
    if n_frontier_clusters >= min_frontier_clusters_to_continue:
        return False, ""
    if unknown_fraction <= max_unknown_fraction:
        return True, f"unknown_fraction={unknown_fraction:.3f}"
    if n_frontier_clusters == 0:
        return True, "no_frontier_clusters"
    return False, ""
