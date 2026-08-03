#!/usr/bin/env python3
"""Where the robot's body can go, as opposed to where a point can.

FaRe plans as a point robot. `find_frontier_cells` only checks a square
neighbourhood, and the offline path search treats every free cell as drivable.
move_base does not: it inflates obstacles by the footprint's inscribed radius
plus footprint_padding, so a gap the offline planner routes straight through can
be impassable at runtime. That mismatch is why waypoints whose own clearance is
fine still fail -- the pinch is on the way there, not at the goal.

Two things are reproduced here, both from launch/costmap_override.yaml:

1. The hard limit. Cells with less than robot_radius + footprint_padding of
   clearance are not drivable at all.
2. The inflation gradient. Masking alone is not enough: the shortest path on a
   binary mask hugs the boundary of what is barely legal. Measured on this map,
   routing that way gives a 0.18 m bottleneck on segments the robot then failed
   to drive -- the same 0.18 m it gets on segments it drove fine, so the binary
   model predicts nothing. NavfnROS does not plan that way; it plans on the cost
   field, which pushes paths toward the middle of corridors. Weighting edges by
   the same decay makes the offline path resemble the one actually driven.

Routing is Dijkstra rather than the A* the paper names. On a uniform grid both
return the same optimal path; Dijkstra is used because one run from a waypoint
yields that waypoint's distance to *every* other one, which is what the GRASP
cost matrix needs. Reported lengths stay geometric metres -- the cost weighting
decides which way to go, not what the trip is then said to have cost.
"""

import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

from config import config

SQRT2 = float(np.sqrt(2))

# Half the 8-connected offsets; the graph is undirected so csgraph mirrors them.
_OFFSETS = ((0, 1, 1.0), (1, 0, 1.0), (1, 1, SQRT2), (1, -1, SQRT2))

# costmap_2d's inflation layer scales to this just outside the inscribed radius,
# and navfn turns a cell cost into a traversal price as COST_NEUTRAL + 0.8 * cost.
_INSCRIBED_COST = 253.0
_COST_NEUTRAL = 50.0
_COST_FACTOR = 0.8


def clearance_grid(grid_map, resolution):
    """Distance in metres from each free cell to the nearest non-navigable cell.

    Unknown space (205) counts as a barrier alongside obstacles (0): the robot
    cannot be driven through unmapped area either.
    """
    return distance_transform_edt(grid_map == config['unexplored_value']) * resolution


def required_clearance():
    """Metres of clearance a cell needs before the robot's footprint fits."""
    return config['robot_radius'] + config['footprint_padding']


def traversable_mask(grid_map, resolution):
    """Boolean grid, True where the robot's footprint fits."""
    return clearance_grid(grid_map, resolution) >= required_clearance()


def inflation_cost(clearance):
    """costmap_2d's inflation cost per cell, from its clearance in metres."""
    inscribed = required_clearance()
    cost = (_INSCRIBED_COST - 1.0) * np.exp(
        -config['cost_scaling_factor'] * (clearance - inscribed)
    )
    cost[clearance <= inscribed] = _INSCRIBED_COST
    cost[clearance > config['inflation_radius']] = 0.0
    return cost


def _neighbour_pairs(index, dr, dc):
    """Node-id pairs for every in-bounds (cell, cell+offset) that are both traversable."""
    rows, cols = index.shape
    r0, r1 = max(0, -dr), rows - max(0, dr)
    c0, c1 = max(0, -dc), cols - max(0, dc)

    here = index[r0:r1, c0:c1]
    there = index[r0 + dr:r1 + dr, c0 + dc:c1 + dc]
    both = (here >= 0) & (there >= 0)
    return here[both], there[both]


class TraversabilityGraph:
    """Shortest-path queries restricted to cells where the robot's footprint fits."""

    def __init__(self, grid_map, resolution):
        self.resolution = resolution
        self.clearance = clearance_grid(grid_map, resolution)
        self.mask = self.clearance >= required_clearance()
        self.cells = np.argwhere(self.mask)

        self.index = -np.ones(self.mask.shape, dtype=np.int64)
        self.index[self.mask] = np.arange(len(self.cells))

        # Price per cell, relative to open floor: 1.0 well away from obstacles,
        # rising to about 5x right against the inflation boundary.
        price = (_COST_NEUTRAL + _COST_FACTOR * inflation_cost(self.clearance)) / _COST_NEUTRAL
        node_price = price[self.mask]

        src, dst, weight = [], [], []
        for dr, dc, step in _OFFSETS:
            a, b = _neighbour_pairs(self.index, dr, dc)
            src.append(a)
            dst.append(b)
            # An edge costs its geometric length times the mean price of its ends.
            weight.append(step * 0.5 * (node_price[a] + node_price[b]))

        n = len(self.cells)
        self.graph = coo_matrix(
            (np.concatenate(weight), (np.concatenate(src), np.concatenate(dst))),
            shape=(n, n),
        ).tocsr()

        self._nearest = None

    def snap(self, cell):
        """Nearest traversable cell to `cell`, or `cell` itself if already traversable.

        FaRe places waypoints without regard to the footprint, so some can land in
        space the robot cannot occupy. Snapping keeps those waypoints in the tour
        instead of dropping them; `snap_distance` reports how far they moved.
        """
        row, col = int(cell[0]), int(cell[1])
        if self.mask[row, col]:
            return (row, col)
        if self._nearest is None:
            self._nearest = distance_transform_edt(~self.mask, return_indices=True)[1]
        return (int(self._nearest[0][row, col]), int(self._nearest[1][row, col]))

    def snap_distance(self, cell):
        """Metres `cell` had to move to become traversable. 0.0 when it already was."""
        row, col = self.snap(cell)
        return float(np.hypot(row - cell[0], col - cell[1])) * self.resolution

    def _walk_back(self, predecessors, source_id, target_id):
        """Cells from source to target, or [] when unreachable."""
        if target_id == source_id:
            return [(int(self.cells[source_id][0]), int(self.cells[source_id][1]))]
        if predecessors[target_id] < 0:
            return []

        path = []
        node = target_id
        while node != source_id:
            path.append((int(self.cells[node][0]), int(self.cells[node][1])))
            node = predecessors[node]
            if node < 0:
                return []
        path.append((int(self.cells[source_id][0]), int(self.cells[source_id][1])))
        return path[::-1]

    def _length(self, path):
        """Geometric length of a cell path, in metres."""
        total = 0.0
        for (r0, c0), (r1, c1) in zip(path, path[1:]):
            total += SQRT2 if (r0 != r1 and c0 != c1) else 1.0
        return total * self.resolution

    def bottleneck(self, path):
        """Tightest clearance in metres anywhere along a cell path."""
        if not path:
            return float('nan')
        return float(min(self.clearance[r, c] for r, c in path))

    def route(self, start, goal):
        """Drivable path between two cells: (list of (row, col), metres).

        Returns ([], inf) when no such path exists.
        """
        source = self.index[self.snap(start)]
        target = self.index[self.snap(goal)]

        _, predecessors = dijkstra(
            self.graph, directed=False, indices=source, return_predecessors=True
        )
        path = self._walk_back(predecessors, source, target)
        return (path, self._length(path)) if path else ([], float('inf'))

    def distance_matrix(self, waypoints):
        """Metres of drivable travel between every pair of waypoints.

        Unreachable pairs come back as numpy.inf, which is the signal that the
        patrol will stall between them at runtime.
        """
        sources = [self.index[self.snap(wp)] for wp in waypoints]
        _, predecessors = dijkstra(
            self.graph, directed=False, indices=sources, return_predecessors=True
        )

        n = len(waypoints)
        distances = np.full((n, n), np.inf)
        for i, source in enumerate(sources):
            for j, target in enumerate(sources):
                path = self._walk_back(predecessors[i], source, target)
                if path:
                    distances[i, j] = self._length(path)
        return distances
