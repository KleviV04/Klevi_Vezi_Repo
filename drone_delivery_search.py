#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Drone Delivery with Battery Limits
Done by: Klevi Vezi

This program simulates a delivery drone that has limited battery capacity.
The drone moves on a grid world, facing wind resistance and needing to recharge.
It must deliver all packages using the least total cost (energy/time).

Search algorithms implemented:
- Uniform Cost Search (UCS)
- A* Search with a simple admissible heuristic
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Tuple, List, Dict, Optional, Iterable, Set
import heapq
import math
import time
import itertools
import random


# →Domain Model

# A coordinate (x, y) on the grid
Coord = Tuple[int, int]

@dataclass(frozen=True)
class ProblemConfig:
    """
    Configuration of the environment:
      - grid size (width, height)
      - start position and delivery targets
      - battery capacity and weight per package
      - wind field (adds cost)
      - recharge stations with different recharge times
      - blocked cells (no-fly zones)
    """
    width: int
    height: int
    start: Coord
    deliveries: Tuple[Coord, ...]
    battery_capacity: int
    weight_per_package: float
    wind: Tuple[Tuple[int, ...], ...]
    recharge_stations: Dict[Coord, int]
    blocked: Set[Coord]

    # Check if a position is inside the grid
    def in_bounds(self, c: Coord) -> bool:
        x, y = c
        return 0 <= x < self.width and 0 <= y < self.height

    # Check if a cell is not blocked
    def passable(self, c: Coord) -> bool:
        return c not in self.blocked

    # Return wind level of a given cell
    def wind_cost(self, c: Coord) -> int:
        x, y = c
        return self.wind[y][x]

@dataclass(frozen=True)
class State:
    """
    Represents one state in the search space.
    It keeps track of:
      - drone position
      - current battery level
      - which deliveries are done (bitmask)
    """
    pos: Coord
    battery: int
    delivered_mask: int

    # Goal test: all deliveries are completed
    def is_goal(self, n_deliveries: int) -> bool:
        return self.delivered_mask == (1 << n_deliveries) - 1

@dataclass
class SearchStats:
    """Used to store statistics after each run."""
    expanded: int = 0
    generated: int = 0
    max_frontier: int = 0
    runtime_sec: float = 0.0

# Cost and Successor functions

BASE_MOVE_COST = 1.0     # basic energy for one move
DELIVER_COST = 0.0       # delivering a package is free here

def remaining_payload_weight(mask: int, n: int, weight_per_pkg: float) -> float:
    """Returns total weight of packages that are still undelivered."""
    remaining = n - bin(mask).count("1")
    return remaining * weight_per_pkg

def step_energy(cfg: ProblemConfig, s: State, to_cell: Coord) -> float:
    """
    Calculates how much energy a move will cost.
    Formula:
        energy = base cost + wind + (0.05 * remaining payload)
    """
    k = 0.05
    w = cfg.wind_cost(to_cell)
    payload = remaining_payload_weight(s.delivered_mask, len(cfg.deliveries), cfg.weight_per_package)
    return BASE_MOVE_COST + w + k * payload

def neighbors4(c: Coord) -> Iterable[Coord]:
    """Return 4-connected neighbor cells (up, down, left, right)."""
    x, y = c
    yield (x+1, y)
    yield (x-1, y)
    yield (x, y+1)
    yield (x, y-1)

def successors(cfg: ProblemConfig, s: State) -> Iterable[Tuple[str, State, float]]:
    """
    Generate all possible actions from the current state.
    Each action returns: (action_name, new_state, cost)
    """
    # 1) Movement actions
    for nxt in neighbors4(s.pos):
        if not cfg.in_bounds(nxt) or not cfg.passable(nxt):
            continue
        energy = step_energy(cfg, s, nxt)
        if s.battery < energy:
            continue
        yield ("MOVE", State(nxt, int(s.battery - math.ceil(energy)), s.delivered_mask), energy)

    # 2) Deliver action (if at a delivery spot not yet delivered)
    if s.pos in cfg.deliveries:
        i = cfg.deliveries.index(s.pos)
        if (s.delivered_mask >> i) & 1 == 0:
            yield ("DELIVER", State(s.pos, s.battery, s.delivered_mask | (1 << i)), DELIVER_COST)

    # 3) Recharge action (if standing on a recharge station)
    if s.pos in cfg.recharge_stations and s.battery < cfg.battery_capacity:
        recharge_time = cfg.recharge_stations[s.pos]
        yield ("RECHARGE", State(s.pos, cfg.battery_capacity, s.delivered_mask), float(recharge_time))

# Heuristic for A* Search

def manhattan(a: Coord, b: Coord) -> int:
    """Simple Manhattan distance between two cells."""
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def mst_length(points: List[Coord]) -> int:
    """
    Compute minimum spanning tree (MST) using Manhattan distance.
    Used as a lower bound for remaining delivery connections.
    """
    if not points:
        return 0
    n = len(points)
    in_tree = [False]*n
    d = [math.inf]*n
    d[0] = 0
    total = 0
    for _ in range(n):
        j = min((i for i in range(n) if not in_tree[i]), key=lambda i: d[i])
        in_tree[j] = True
        total += 0 if d[j] is math.inf else d[j]
        for k in range(n):
            if not in_tree[k]:
                w = manhattan(points[j], points[k])
                if w < d[k]:
                    d[k] = w
    return int(total)

def heuristic(cfg: ProblemConfig, s: State) -> float:
    """
    A* heuristic:
      h = distance from current position to nearest remaining + MST over remaining deliveries
    (ignores wind/recharge times → admissible lower bound)
    """
    remaining: List[Coord] = [p for i, p in enumerate(cfg.deliveries) if ((s.delivered_mask >> i) & 1) == 0]
    if not remaining:
        return 0.0
    d0 = min(manhattan(s.pos, p) for p in remaining)
    mst = mst_length(remaining)
    return float(d0 + mst)

# Generic Best-First Search (works for UCS or A*)

class PQItem:
    """Item stored in priority queue (frontier)."""
    __slots__ = ("priority", "count", "state", "g")

    def __init__(self, priority: float, count: int, state: State, g: float):
        self.priority = priority  # f = g + h
        self.count = count        # tie-breaker
        self.state = state
        self.g = g                # path cost so far

    def __lt__(self, other: "PQItem"):
        if self.priority != other.priority:
            return self.priority < other.priority
        return self.count < other.count

def search(cfg: ProblemConfig, astar_mode: bool) -> Tuple[Optional[State], Dict[State, Tuple[Optional[State], str, float]], SearchStats]:
    """Unified UCS/A* search."""
    start = State(cfg.start, cfg.battery_capacity, 0)
    t0 = time.perf_counter()

    frontier: List[PQItem] = []
    counter = 0
    start_h = heuristic(cfg, start) if astar_mode else 0.0
    heapq.heappush(frontier, PQItem(start_h, counter, start, 0.0))
    counter += 1

    best_g: Dict[State, float] = {start: 0.0}
    parent: Dict[State, Tuple[Optional[State], str, float]] = {start: (None, "START", 0.0)}
    stats = SearchStats(expanded=0, generated=1, max_frontier=1)

    while frontier:
        stats.max_frontier = max(stats.max_frontier, len(frontier))
        item = heapq.heappop(frontier)
        s = item.state
        g = item.g
        stats.expanded += 1

        if s.is_goal(len(cfg.deliveries)):
            stats.runtime_sec = time.perf_counter() - t0
            return s, parent, stats

        for action, s2, cost in successors(cfg, s):
            g2 = g + cost
            if s2 not in best_g or g2 < best_g[s2] - 1e-9:
                best_g[s2] = g2
                parent[s2] = (s, action, cost)
                h2 = heuristic(cfg, s2) if astar_mode else 0.0
                f2 = g2 + h2
                heapq.heappush(frontier, PQItem(f2, counter, s2, g2))
                counter += 1
                stats.generated += 1

    stats.runtime_sec = time.perf_counter() - t0
    return None, parent, stats

def reconstruct_path(parent: Dict[State, Tuple[Optional[State], str, float]], goal: State) -> List[Tuple[str, State, float]]:
    """Rebuild the path (actions) from the parent dictionary."""
    out = []
    s = goal
    while True:
        prev, action, cost = parent[s]
        if prev is None:
            break
        out.append((action, s, cost))
        s = prev
    out.reverse()
    return out


# Experiments


def make_demo_config(seed: int = 0) -> ProblemConfig:
    """Creates a demo environment for testing."""
    rng = random.Random(seed)
    width, height = 8, 6
    start = (0, 0)
    deliveries = ((6, 1), (4, 5), (7, 4))  # 3 packages
    battery_capacity = 20
    weight_per_package = 5.0

    # Random wind field (values 0–2)
    wind = tuple(tuple(rng.randint(0, 2) for _ in range(width)) for _ in range(height))

    # Recharge stations with different recharge times
    recharge_stations = {(0, 0): 2, (3, 2): 4, (6, 5): 1}

    # No-fly zones
    blocked = {(2, 1), (2, 2), (2, 3), (5, 3)}

    return ProblemConfig(
        width=width, height=height, start=start,
        deliveries=deliveries, battery_capacity=battery_capacity,
        weight_per_package=weight_per_package, wind=wind,
        recharge_stations=recharge_stations, blocked=blocked
    )

def print_plan(cfg: ProblemConfig, plan: List[Tuple[str, State, float]], total_cost: float) -> None:
    """Prints the plan in a readable format."""
    print("Plan:")
    for i, (act, s, c) in enumerate(plan, 1):
        x, y = s.pos
        print(f"{i:02d}. {act:8s} -> pos=({x},{y}) bat={s.battery:3d} delivered_mask={bin(s.delivered_mask)}  step_cost={c:.1f}")
    print(f"TOTAL COST: {total_cost:.1f}")

def run_once(astar_mode: bool) -> None:
    """Runs one test with either UCS or A*."""
    cfg = make_demo_config(seed=42)
    goal, parent, stats = search(cfg, astar_mode=astar_mode)
    title = "A*" if astar_mode else "UCS"
    print("="*72)
    print(f"{title} RESULTS")
    if goal is None:
        print("No solution found.")
        print(f"Expanded: {stats.expanded} Generated: {stats.generated} MaxFrontier: {stats.max_frontier} Time: {stats.runtime_sec:.4f}s")
        return
    plan = reconstruct_path(parent, goal)
    total = sum(c for _,_,c in plan)
    print(f"Expanded: {stats.expanded} Generated: {stats.generated} MaxFrontier: {stats.max_frontier} Time: {stats.runtime_sec:.4f}s")
    print_plan(cfg, plan, total)

if __name__ == "__main__":
    # Algorithms comparison
    run_once(astar_mode=False)  # UCS
    run_once(astar_mode=True)   # A*
