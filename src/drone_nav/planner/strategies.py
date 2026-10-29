"""Scan sequencing strategies for warehouse drone flight planning."""

import math
from typing import Protocol

from drone_nav.world.models import Bin, World


class ScanOrderStrategy(Protocol):
    """Protocol for bin scan sequencing."""
    def order_bins(self, world: World, bins: list[Bin]) -> list[Bin]:
        ...


class NaiveStrategy:
    """Naive baseline: visits bins in raw alphanumeric ID order without spatial sequencing."""

    def order_bins(self, world: World, bins: list[Bin]) -> list[Bin]:
        return sorted(bins, key=lambda b: b.id)


class SerpentineStrategy:
    """Optimized serpentine strategy: visits aisles sequentially with smooth entry/exit.

    Respects one-way aisles, predominant warehouse orientation, and scans flanking racks in an organized sweep.
    """

    def order_bins(self, world: World, bins: list[Bin]) -> list[Bin]:
        # Group bins by aisle
        bins_by_aisle: dict[str, list[Bin]] = {}
        for b in bins:
            bins_by_aisle.setdefault(b.aisle_id, []).append(b)

        aisle_map = {a.id: a for a in world.aisles}

        # Check predominant orientation
        total_dx = sum(abs(a.x_end - a.x_start) for a in world.aisles)
        total_dy = sum(abs(a.y_end - a.y_start) for a in world.aisles)
        is_predominantly_x = total_dx >= total_dy

        if is_predominantly_x:
            # Sort aisles along Y (North to South / South to North)
            sorted_aisle_ids = sorted(
                bins_by_aisle.keys(),
                key=lambda a_id: (aisle_map[a_id].y_start + aisle_map[a_id].y_end) / 2.0 if a_id in aisle_map else 0.0,
            )
        else:
            # Sort aisles along X
            sorted_aisle_ids = sorted(
                bins_by_aisle.keys(),
                key=lambda a_id: (aisle_map[a_id].x_start + aisle_map[a_id].x_end) / 2.0 if a_id in aisle_map else 0.0,
            )

        ordered: list[Bin] = []
        for aisle_idx, a_id in enumerate(sorted_aisle_ids):
            a_bins = bins_by_aisle[a_id]
            aisle_obj = aisle_map.get(a_id)

            if is_predominantly_x:
                if aisle_obj and aisle_obj.one_way:
                    reverse_dir = (aisle_obj.direction == "backward")
                else:
                    reverse_dir = (aisle_idx % 2 == 1)

                # Sort bins primarily by X position (along longitudinal aisle), then rack side, then level
                a_bins.sort(
                    key=lambda b: (
                        -b.center[0] if reverse_dir else b.center[0],
                        b.rack_id,
                        b.level_index,
                    )
                )
            else:
                if aisle_obj and aisle_obj.one_way:
                    reverse_dir = (aisle_obj.direction == "backward")
                else:
                    reverse_dir = (aisle_idx % 2 == 1)

                # Sort bins primarily by Y position (along aisle), then rack side, then level
                a_bins.sort(
                    key=lambda b: (
                        -b.center[1] if reverse_dir else b.center[1],
                        b.rack_id,
                        b.level_index,
                    )
                )

            ordered.extend(a_bins)

        return ordered


class NearestNeighborStrategy:
    """Greedy Nearest-Neighbor heuristic: iteratively visits closest remaining bin."""

    def order_bins(self, world: World, bins: list[Bin]) -> list[Bin]:
        if not bins:
            return []

        # Start at first pad position or first bin
        curr_pos = (
            world.pads[0].position if world.pads else bins[0].recommended_scan_pos
        )

        if len(bins) > 500:
            import numpy as np
            from scipy.spatial import cKDTree

            coords = np.array([b.recommended_scan_pos for b in bins], dtype=float)
            tree = cKDTree(coords)
            visited = np.zeros(len(bins), dtype=bool)
            ordered: list[Bin] = []
            curr_pt = np.array(curr_pos, dtype=float)

            for _ in range(len(bins)):
                k_val = min(64, len(bins))
                _, indices = tree.query(curr_pt, k=k_val)
                if np.isscalar(indices):
                    indices = [indices]
                found = False
                for idx in indices:
                    if not visited[idx]:
                        visited[idx] = True
                        ordered.append(bins[idx])
                        curr_pt = coords[idx]
                        found = True
                        break
                if not found:
                    unvisited = np.where(~visited)[0]
                    diffs = coords[unvisited] - curr_pt
                    dists_sq = np.sum(diffs ** 2, axis=1)
                    idx = unvisited[int(np.argmin(dists_sq))]
                    visited[idx] = True
                    ordered.append(bins[idx])
                    curr_pt = coords[idx]
            return ordered

        remaining = list(bins)
        ordered = []
        while remaining:
            closest_idx = 0
            best_dist = float("inf")
            for i, candidate in enumerate(remaining):
                d = math.dist(curr_pos, candidate.recommended_scan_pos)
                if d < best_dist:
                    best_dist = d
                    closest_idx = i

            next_bin = remaining.pop(closest_idx)
            ordered.append(next_bin)
            curr_pos = next_bin.recommended_scan_pos

        return ordered


class MinimumTimeStrategy:
    """Minimum-time strategy: clusters bay levels and accounts for 3D climb and turn costs."""

    def order_bins(self, world: World, bins: list[Bin]) -> list[Bin]:
        bins_by_aisle: dict[str, list[Bin]] = {}
        for b in bins:
            bins_by_aisle.setdefault(b.aisle_id, []).append(b)

        aisle_map = {a.id: a for a in world.aisles}

        total_dx = sum(abs(a.x_end - a.x_start) for a in world.aisles)
        total_dy = sum(abs(a.y_end - a.y_start) for a in world.aisles)
        is_predominantly_x = total_dx >= total_dy

        if is_predominantly_x:
            sorted_aisle_ids = sorted(
                bins_by_aisle.keys(),
                key=lambda a_id: (aisle_map[a_id].y_start + aisle_map[a_id].y_end) / 2.0 if a_id in aisle_map else 0.0,
            )
        else:
            sorted_aisle_ids = sorted(
                bins_by_aisle.keys(),
                key=lambda a_id: (aisle_map[a_id].x_start + aisle_map[a_id].x_end) / 2.0 if a_id in aisle_map else 0.0,
            )

        ordered: list[Bin] = []
        for aisle_idx, a_id in enumerate(sorted_aisle_ids):
            a_bins = bins_by_aisle[a_id]
            aisle_obj = aisle_map.get(a_id)

            reverse = (aisle_obj.direction == "backward") if (aisle_obj and aisle_obj.one_way) else (aisle_idx % 2 == 1)

            bins_by_bay: dict[int, list[Bin]] = {}
            for b in a_bins:
                bins_by_bay.setdefault(b.bay_index, []).append(b)

            sorted_bays = sorted(bins_by_bay.keys(), reverse=reverse)
            for bay_idx, b_idx in enumerate(sorted_bays):
                bay_bins = bins_by_bay[b_idx]
                climb_up = (bay_idx % 2 == 0)
                bay_bins.sort(key=lambda b: b.level_index if climb_up else -b.level_index)
                ordered.extend(bay_bins)

        return ordered
