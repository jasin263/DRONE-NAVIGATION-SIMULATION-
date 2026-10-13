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

    Respects one-way aisles and scans left/right flanking racks in an organized sweep.
    """

    def order_bins(self, world: World, bins: list[Bin]) -> list[Bin]:
        # Group bins by aisle
        bins_by_aisle: dict[str, list[Bin]] = {}
        for b in bins:
            bins_by_aisle.setdefault(b.aisle_id, []).append(b)

        # Sort aisles by physical X coordinate
        aisle_map = {a.id: a for a in world.aisles}
        sorted_aisle_ids = sorted(
            bins_by_aisle.keys(),
            key=lambda a_id: aisle_map[a_id].x_start if a_id in aisle_map else 0.0,
        )

        ordered: list[Bin] = []
        for aisle_idx, a_id in enumerate(sorted_aisle_ids):
            a_bins = bins_by_aisle[a_id]
            aisle_obj = aisle_map.get(a_id)

            # Determine traversal direction along Y
            # If one_way forward -> South to North (increasing Y)
            # If one_way backward -> North to South (decreasing Y)
            # If bidirectional -> alternate direction (serpentine snake: even index S->N, odd index N->S)
            if aisle_obj and aisle_obj.one_way:
                reverse_y = (aisle_obj.direction == "backward")
            else:
                reverse_y = (aisle_idx % 2 == 1)

            # Sort bins primarily by Y position (along aisle), secondarily by rack side, then level
            a_bins.sort(
                key=lambda b: (
                    -b.center[1] if reverse_y else b.center[1],
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

        remaining = list(bins)
        # Start at first pad position or first bin
        curr_pos = (
            world.pads[0].position if world.pads else remaining[0].recommended_scan_pos
        )
        ordered: list[Bin] = []

        while remaining:
            # Find closest unvisited bin to current position
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
        # Group by aisle, then order each rack face in a vertical serpentine (up one bay, down the next)
        bins_by_aisle: dict[str, list[Bin]] = {}
        for b in bins:
            bins_by_aisle.setdefault(b.aisle_id, []).append(b)

        aisle_map = {a.id: a for a in world.aisles}
        sorted_aisle_ids = sorted(
            bins_by_aisle.keys(),
            key=lambda a_id: aisle_map[a_id].x_start if a_id in aisle_map else 0.0,
        )

        ordered: list[Bin] = []
        for aisle_idx, a_id in enumerate(sorted_aisle_ids):
            a_bins = bins_by_aisle[a_id]
            aisle_obj = aisle_map.get(a_id)

            reverse_y = (aisle_obj.direction == "backward") if (aisle_obj and aisle_obj.one_way) else (aisle_idx % 2 == 1)

            # Group bins by bay
            bins_by_bay: dict[int, list[Bin]] = {}
            for b in a_bins:
                bins_by_bay.setdefault(b.bay_index, []).append(b)

            sorted_bays = sorted(bins_by_bay.keys(), reverse=reverse_y)
            for bay_idx, b_idx in enumerate(sorted_bays):
                bay_bins = bins_by_bay[b_idx]
                # Alternate climb up and descent down to save vertical energy and time
                climb_up = (bay_idx % 2 == 0)
                bay_bins.sort(key=lambda b: b.level_index if climb_up else -b.level_index)
                ordered.extend(bay_bins)

        return ordered
