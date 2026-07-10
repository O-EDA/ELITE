from __future__ import annotations

import contextlib
import io
import os
import sys
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Iterable


Point = tuple[int, int]
PathData = list[list[Point]]


@dataclass(frozen=True)
class LegacyContourDetourConfig:
    script_path: Path
    target_total_length_grid: int
    residual_lengths_grid: tuple[int, ...] | None = None
    expected_explicit_lengths_grid: tuple[int, ...] | None = None
    boundary_area_grid: tuple[int, int, int, int] | None = None
    boundary_top_padding_grid: int | None = None
    auto_region_search_area_grid: tuple[int, int, int, int] | None = None
    hard_boundary_area_grid: tuple[int, int, int, int] | None = None
    external_obstacle_paths_grid: PathData | None = None
    external_block_points_grid: tuple[Point, ...] | None = None
    detour_area_slack_delta: float | None = None
    spiral_placement_policy: str = "legacy_longest"
    spiral_preferred_endpoints_grid: tuple[Point, ...] | None = None
    detour_direction_priority: str = "fixed_original"


@dataclass(frozen=True)
class DetourResult:
    paths: PathData
    log: str
    metadata: dict[str, object]
    snapshots: list[dict[str, object]]


@dataclass(frozen=True)
class DirectBoundaryContext:
    config: LegacyContourDetourConfig
    source_paths: PathData
    residual_lengths_grid: tuple[int, ...]
    namespace: dict[str, object]
    external_obstacle_paths: PathData
    ori_path: PathData
    res_len: list[int]
    expected_lengths_by_endpoint: dict[tuple[Point, Point], int]
    expected_lengths_by_order: list[dict[str, object]]
    preferred_endpoint_by_endpoint: dict[tuple[Point, Point], Point]
    boundary_area: list[int]
    boundary_source: str
    boundary: list[Point]
    routing_boundary_area: list[int]
    routing_boundary_source: str
    routing_boundary: list[Point]
    hard_boundary_area: list[int] | None
    area_slack_check: dict[str, object]
    schedule_plan: dict[str, object]
    target_length: int
    target_lengths_by_endpoint: dict[tuple[Point, Point], int]


@dataclass(frozen=True)
class OrderedSideRegionStep:
    active_side: str
    candidate_order: int
    diffuse_from_order: int
    diffuse_direction: str
    spiral_direction: str
    pair_order: int
    diffuse_snapshot_title: str
    fixed_snapshot_title: str


def to_point(point: Iterable[int]) -> Point:
    x, y = point
    return int(x), int(y)


def orient_left_to_right(route: list[Point]) -> list[Point]:
    if route[0][0] > route[-1][0]:
        return list(reversed(route))
    return list(route)


def sort_routes(paths: PathData) -> PathData:
    return sorted((simplify_collinear([to_point(point) for point in route]) for route in paths), key=lambda p: (p[0][1], p[0][0], p[-1][1], p[-1][0]))


def endpoint_key(route: list[Iterable[int]]) -> tuple[Point, Point]:
    route = simplify_collinear([to_point(point) for point in route])
    if route[0] <= route[-1]:
        return route[0], route[-1]
    return route[-1], route[0]


def manhattan_length(route: list[Point]) -> int:
    route = simplify_collinear(route)
    return sum(abs(a[0] - b[0]) + abs(a[1] - b[1]) for a, b in zip(route, route[1:]))


def route_lengths(paths: PathData) -> list[int]:
    return [manhattan_length(route) for route in sort_routes(paths)]


def bend_count(route: list[Iterable[int]]) -> int:
    route = simplify_collinear([to_point(point) for point in route])
    bends = 0
    for a, b, c in zip(route, route[1:], route[2:]):
        v1 = (b[0] - a[0], b[1] - a[1])
        v2 = (c[0] - b[0], c[1] - b[1])
        if v1[0] * v2[1] - v1[1] * v2[0] != 0:
            bends += 1
    return bends


def clone_paths(paths: list[list[Iterable[int]]]) -> PathData:
    return [[to_point(point) for point in route] for route in paths]


def normalize_obstacle_paths(paths: list[list[Iterable[int]]] | None) -> PathData:
    normalized: PathData = []
    for route in paths or []:
        points = [to_point(point) for point in route]
        if not points:
            continue
        if len(points) == 1:
            points = [points[0], points[0]]
        normalized.append(points)
    return normalized


def block_points_to_obstacle_paths(points: tuple[Point, ...] | None) -> PathData:
    return [[to_point(point), to_point(point)] for point in points or ()]


def config_external_obstacle_paths(config: LegacyContourDetourConfig) -> PathData:
    return normalize_obstacle_paths(config.external_obstacle_paths_grid) + block_points_to_obstacle_paths(
        config.external_block_points_grid
    )


def swap_xy_point(point: Iterable[int]) -> Point:
    x, y = to_point(point)
    return y, x


def swap_xy_paths(paths: list[list[Iterable[int]]]) -> PathData:
    return [[swap_xy_point(point) for point in route] for route in paths]


def swap_xy_boundary_area(boundary_area: tuple[int, int, int, int] | None) -> tuple[int, int, int, int] | None:
    if boundary_area is None:
        return None
    x_min, x_max, y_min, y_max = boundary_area
    return y_min, y_max, x_min, x_max


def swap_xy_boundary_area_list(boundary_area: list[int] | tuple[int, int, int, int]) -> list[int]:
    x_min, x_max, y_min, y_max = boundary_area
    return [y_min, y_max, x_min, x_max]


def swap_xy_longest_boundary_schedule(schedule: dict[str, object]) -> dict[str, object]:
    swapped = dict(schedule)
    if swapped.get("orientation") == "horizontal":
        swapped["orientation"] = "vertical"
    elif swapped.get("orientation") == "vertical":
        swapped["orientation"] = "horizontal"
    if swapped.get("order_axis") == "y":
        swapped["order_axis"] = "x"
    elif swapped.get("order_axis") == "x":
        swapped["order_axis"] = "y"
    side_map = {"lower": "left", "upper": "right", "left": "lower", "right": "upper"}
    partition = swapped.get("partition")
    if isinstance(partition, dict):
        partition = dict(partition)
        parts = []
        for raw_part in partition.get("partitions", []):
            part = dict(raw_part)
            if "side" in part:
                part["side"] = side_map.get(str(part["side"]), part["side"])
            parts.append(part)
        partition["partitions"] = parts
        swapped["partition"] = partition
    return swapped


def swap_xy_auto_region_metadata(metadata: dict[str, object]) -> dict[str, object]:
    def swap_side(side: object) -> object:
        if side == "top":
            return "right"
        if side == "bottom":
            return "left"
        if side == "left":
            return "bottom"
        if side == "right":
            return "top"
        return side

    swapped = dict(metadata)
    for key in ("base_boundary_area", "selected_boundary_area", "requested_search_boundary_area", "search_boundary_area"):
        if key in swapped and swapped[key] is not None:
            swapped[key] = swap_xy_boundary_area_list(swapped[key])
    extension_options = []
    for raw_option in swapped.get("extension_options", []):
        option = dict(raw_option)
        if "boundary_area" in option:
            option["boundary_area"] = swap_xy_boundary_area_list(option["boundary_area"])
        if "extension_side" in option:
            option["extension_side"] = swap_side(option["extension_side"])
        extension_options.append(option)
    if extension_options:
        swapped["extension_options"] = extension_options
    candidates = []
    for raw_candidate in swapped.get("candidates", []):
        candidate = dict(raw_candidate)
        if "boundary_area" in candidate:
            candidate["boundary_area"] = swap_xy_boundary_area_list(candidate["boundary_area"])
        if "extension_side" in candidate:
            candidate["extension_side"] = swap_side(candidate["extension_side"])
        candidates.append(candidate)
    if candidates:
        swapped["candidates"] = candidates
    if swapped.get("orientation") == "horizontal":
        swapped["orientation"] = "vertical"
    elif swapped.get("orientation") == "vertical":
        swapped["orientation"] = "horizontal"
    swapped["extension_side"] = swap_side(swapped.get("extension_side"))
    if "longest_boundary_schedule" in swapped:
        swapped["longest_boundary_schedule"] = swap_xy_longest_boundary_schedule(swapped["longest_boundary_schedule"])
    return swapped


def swap_xy_snapshots(snapshots: list[dict[str, object]]) -> list[dict[str, object]]:
    swapped: list[dict[str, object]] = []
    for snapshot in snapshots:
        item = dict(snapshot)
        for key in ("fixed_paths", "candidate_paths", "source_paths", "external_obstacle_paths"):
            item[key] = swap_xy_paths(snapshot.get(key, []))
        swapped.append(item)
    return swapped


def sign(value: int) -> int:
    if value > 0:
        return 1
    if value < 0:
        return -1
    return 0


def route_contains(route: list[Iterable[int]], point: Point) -> bool:
    return [point[0], point[1]] in route or point in route


def walk_points(route: list[Iterable[int]]) -> list[Point]:
    points = [to_point(route[0])]
    for raw_a, raw_b in zip(route, route[1:]):
        a = to_point(raw_a)
        b = to_point(raw_b)
        dx = sign(b[0] - a[0])
        dy = sign(b[1] - a[1])
        if dx and dy:
            raise ValueError(f"Non-manhattan segment: {a} -> {b}")
        cur = a
        while cur != b:
            cur = (cur[0] + dx, cur[1] + dy)
            points.append(cur)
    return points


def unit_edges(route: list[Iterable[int]]) -> list[tuple[Point, Point]]:
    points = walk_points(route)
    edges = []
    for a, b in zip(points, points[1:]):
        edges.append(tuple(sorted((a, b))))
    return edges


def has_self_block(route: list[Iterable[int]]) -> bool:
    route = simplify_collinear([to_point(point) for point in route])
    edges = unit_edges(route)
    if len(edges) != len(set(edges)):
        return True
    points = walk_points(route)
    seen: dict[Point, int] = {}
    for idx, point in enumerate(points):
        if point in seen and idx - seen[point] > 1:
            return True
        seen[point] = idx
    return False


def blocked_sets(paths: list[list[Iterable[int]]]) -> tuple[set[Point], set[tuple[Point, Point]]]:
    points: set[Point] = set()
    edges: set[tuple[Point, Point]] = set()
    for route in paths:
        points.update(walk_points(route))
        edges.update(unit_edges(route))
    return points, edges


def collides_with_block(route: list[Iterable[int]], block_paths: list[list[Iterable[int]]]) -> bool:
    block_points, block_edges = blocked_sets(block_paths)
    route_points = set(walk_points(route))
    route_edges = set(unit_edges(route))
    return bool(route_points & block_points or route_edges & block_edges)


def route_touches_grid_block(route: list[Iterable[int]], grid, boundary_area: list[int]) -> bool:
    x_min, _x_max, y_min, _y_max = boundary_area
    for point in walk_points(route):
        gx = point[0] - x_min
        gy = point[1] - y_min
        if gy < 0 or gy >= grid.shape[0] or gx < 0 or gx >= grid.shape[1]:
            return True
        if grid[gy, gx] == 1:
            return True
    return False


def stepwise_route_violation(
    route: list[Iterable[int]],
    grid,
    boundary_area: list[int],
) -> tuple[str | None, Point | None]:
    x_min, _x_max, y_min, _y_max = boundary_area
    points = simplify_collinear([to_point(point) for point in route])
    if not points:
        return "empty_route", None

    seen_points: dict[Point, int] = {}
    seen_edges: set[tuple[Point, Point]] = set()
    walk_index = 0

    def check_point(point: Point) -> str | None:
        gx = point[0] - x_min
        gy = point[1] - y_min
        if gy < 0 or gy >= grid.shape[0] or gx < 0 or gx >= grid.shape[1]:
            return "stepwise_out_of_boundary"
        if grid[gy, gx] == 1:
            return "stepwise_grid_block"
        return None

    reason = check_point(points[0])
    if reason is not None:
        return reason, points[0]
    seen_points[points[0]] = 0

    for raw_a, raw_b in zip(points, points[1:]):
        a = to_point(raw_a)
        b = to_point(raw_b)
        dx = sign(b[0] - a[0])
        dy = sign(b[1] - a[1])
        if dx and dy:
            return "non_manhattan_step", a
        cur = a
        while cur != b:
            nxt = (cur[0] + dx, cur[1] + dy)
            edge = tuple(sorted((cur, nxt)))
            if edge in seen_edges:
                return "stepwise_self_edge_block", nxt
            seen_edges.add(edge)
            walk_index += 1
            reason = check_point(nxt)
            if reason is not None:
                return reason, nxt
            if nxt in seen_points and walk_index - seen_points[nxt] > 1:
                return "stepwise_self_point_block", nxt
            seen_points[nxt] = walk_index
            cur = nxt
    return None, None


def point_in_boundary(point: Point, boundary_area: list[int] | tuple[int, int, int, int]) -> bool:
    x_min, x_max, y_min, y_max = boundary_area
    return x_min <= point[0] <= x_max and y_min <= point[1] <= y_max


def route_in_boundary(route: list[Iterable[int]], boundary_area: list[int] | tuple[int, int, int, int]) -> bool:
    return all(point_in_boundary(point, boundary_area) for point in walk_points(route))


def clip_obstacle_paths_to_boundary(
    obstacle_paths: list[list[Iterable[int]]],
    boundary_area: list[int] | tuple[int, int, int, int],
) -> PathData:
    clipped: PathData = []
    for route in obstacle_paths:
        current: list[Point] = []
        for point in walk_points(route):
            if point_in_boundary(point, boundary_area):
                current.append(point)
                continue
            if current:
                clipped.append(simplify_collinear(current) if len(current) > 1 else [current[0], current[0]])
                current = []
        if current:
            clipped.append(simplify_collinear(current) if len(current) > 1 else [current[0], current[0]])
    return clipped


def apply_simple_spiral(
    route: list[Iterable[int]],
    ind: int,
    point_start: Point,
    point_end: Point,
    direct: str,
    length_dt: int,
) -> list[Point]:
    if direct not in {"up", "down", "left", "right"}:
        raise ValueError(f"Unknown detour direction: {direct}")
    unit_along = (sign(point_end[0] - point_start[0]), sign(point_end[1] - point_start[1]))
    if unit_along == (0, 0):
        raise ValueError(f"Zero-length insertion segment: {point_start} -> {point_end}")
    if unit_along[0] and unit_along[1]:
        raise ValueError(f"Expected manhattan insertion segment, got {point_start} -> {point_end}")
    side_vectors = {
        "up": (0, 1),
        "down": (0, -1),
        "right": (1, 0),
        "left": (-1, 0),
    }
    unit_side = side_vectors[direct]
    if unit_along[0] and unit_side[0]:
        raise ValueError(f"Detour direction {direct} is parallel to horizontal segment {point_start} -> {point_end}")
    if unit_along[1] and unit_side[1]:
        raise ValueError(f"Detour direction {direct} is parallel to vertical segment {point_start} -> {point_end}")

    def add(point: Point, along: int = 0, side: int = 0) -> Point:
        return (
            point[0] + along * unit_along[0] + side * unit_side[0],
            point[1] + along * unit_along[1] + side * unit_side[1],
        )

    route_new = [to_point(point) for point in route]
    if point_start not in route_new:
        route_new.insert(ind + 1, point_start)
        ind += 1
    if length_dt == 1:
        insert = [
            add(point_start, side=1),
            add(point_start, along=1, side=1),
            add(point_start, along=1),
        ]
    else:
        point_far = add(point_start, along=length_dt - 1)
        insert = [
            add(point_start, side=2),
            add(point_far, side=2),
            add(point_far, side=1),
            add(point_start, along=1, side=1),
            add(point_start, along=1),
        ]
    for offset, point in enumerate(insert, start=1):
        route_new.insert(ind + offset, point)
    return simplify_collinear(route_new)


def manhattan_segment_candidates(route: list[Iterable[int]]) -> list[tuple[int, int, Point, Point, str]]:
    candidates = []
    for idx, (raw_a, raw_b) in enumerate(zip(route, route[1:])):
        a = to_point(raw_a)
        b = to_point(raw_b)
        if a == b:
            continue
        if a[1] == b[1]:
            candidates.append((abs(b[0] - a[0]), idx, a, b, "horizontal"))
        elif a[0] == b[0]:
            candidates.append((abs(b[1] - a[1]), idx, a, b, "vertical"))
    return sorted(candidates, key=lambda item: (-item[0], item[1]))


def route_prefix_lengths(route: list[Iterable[int]]) -> list[int]:
    points = [to_point(point) for point in route]
    prefix = [0]
    for a, b in zip(points, points[1:]):
        prefix.append(prefix[-1] + abs(a[0] - b[0]) + abs(a[1] - b[1]))
    return prefix


def source_target_arc_distance(
    route: list[Iterable[int]],
    prefix_lengths: list[int],
    segment_index: int,
    point_on_segment: Point,
) -> tuple[int, int, int]:
    points = [to_point(point) for point in route]
    source_distance = prefix_lengths[segment_index] + abs(point_on_segment[0] - points[segment_index][0]) + abs(
        point_on_segment[1] - points[segment_index][1]
    )
    target_distance = prefix_lengths[-1] - source_distance
    return min(source_distance, target_distance), source_distance, target_distance


def opposite_detour_direction(direct: str) -> str:
    opposites = {"up": "down", "down": "up", "left": "right", "right": "left"}
    if direct not in opposites:
        raise ValueError(f"Unknown detour direction: {direct}")
    return opposites[direct]


def detour_direction_order(direct: str) -> list[str]:
    opposite = opposite_detour_direction(direct)
    return [direct, opposite]


def fixed_priority_directions_for_segment(orientation: str, priority: str) -> list[str]:
    if orientation == "horizontal":
        if priority == "fixed_original":
            return ["up", "down"]
        if priority == "fixed_original_swapped_xy":
            return ["down", "up"]
    if orientation == "vertical":
        if priority == "fixed_original":
            return ["left", "right"]
        if priority == "fixed_original_swapped_xy":
            return ["right", "left"]
    raise ValueError(f"Unknown segment orientation/priority: {orientation}, {priority}")


def uses_fixed_direction_priority(priority: str) -> bool:
    return priority in {"fixed_original", "fixed_original_swapped_xy"}


def uses_source_target_inner_priority(priority: str) -> bool:
    return priority in {"source_target_inner", "legacy_direct"}


def detour_directions_for_segment(orientation: str, direct: str, priority: str = "legacy_direct") -> list[str]:
    if uses_fixed_direction_priority(priority):
        return fixed_priority_directions_for_segment(orientation, priority)
    if not uses_source_target_inner_priority(priority):
        raise ValueError(f"Unknown detour direction priority: {priority}")
    if orientation == "horizontal":
        directions = ["up", "down"]
    elif orientation == "vertical":
        directions = ["right", "left"]
    else:
        raise ValueError(f"Unknown segment orientation: {orientation}")
    return [candidate for candidate in detour_direction_order(direct) if candidate in directions]


def detour_segment_orientation(direct: str) -> str:
    if direct in {"up", "down"}:
        return "horizontal"
    if direct in {"left", "right"}:
        return "vertical"
    raise ValueError(f"Unknown detour direction: {direct}")


def preferred_detour_directions(route: list[Iterable[int]], boundary_area: list[int]) -> list[str]:
    route = [to_point(point) for point in route]
    x_min, x_max, y_min, y_max = boundary_area
    if route[0][1] == route[-1][1]:
        y = route[0][1]
        up_space = y_max - y
        down_space = y - y_min
        return ["up", "down"] if up_space >= down_space else ["down", "up"]
    if route[0][0] == route[-1][0]:
        x = route[0][0]
        right_space = x_max - x
        left_space = x - x_min
        return ["right", "left"] if right_space >= left_space else ["left", "right"]
    return ["up", "down", "right", "left"]


def shorten_u_bends_to_length(
    route: list[Iterable[int]],
    target_length: int,
    fixed_paths: list[list[Iterable[int]]],
    grid,
    boundary_area: list[int],
    hard_boundary_area: list[int] | tuple[int, int, int, int] | None = None,
) -> tuple[list[Point], dict[str, object]]:
    current = simplify_collinear([to_point(point) for point in route])
    attempts: list[dict[str, object]] = []
    reductions: list[dict[str, object]] = []

    def one_reduction_candidates(path: list[Point], required_delta: int) -> list[tuple[tuple[int, int, int], list[Point], dict[str, object]]]:
        candidates: list[tuple[tuple[int, int, int], list[Point], dict[str, object]]] = []
        target_shrink = (required_delta + 1) // 2
        for idx in range(len(path) - 3):
            p0, p1, p2, p3 = path[idx : idx + 4]
            v01 = (p1[0] - p0[0], p1[1] - p0[1])
            v12 = (p2[0] - p1[0], p2[1] - p1[1])
            v23 = (p3[0] - p2[0], p3[1] - p2[1])
            len01 = abs(v01[0]) + abs(v01[1])
            len12 = abs(v12[0]) + abs(v12[1])
            len23 = abs(v23[0]) + abs(v23[1])
            attempt = {
                "idx": idx,
                "points": [p0, p1, p2, p3],
                "segment_lengths": [len01, len12, len23],
            }
            if len01 == 0 or len12 == 0 or len23 == 0:
                attempt["status"] = "zero_length_segment"
                attempts.append(attempt)
                continue
            if (v01[0] and v01[1]) or (v12[0] and v12[1]) or (v23[0] and v23[1]):
                attempt["status"] = "non_manhattan_window"
                attempts.append(attempt)
                continue
            dot_outer = v01[0] * v23[0] + v01[1] * v23[1]
            cross_outer = v01[0] * v23[1] - v01[1] * v23[0]
            dot_middle = v01[0] * v12[0] + v01[1] * v12[1]
            if cross_outer != 0 or dot_outer >= 0 or dot_middle != 0:
                attempt["status"] = "not_u_bend"
                attempts.append(attempt)
                continue
            shrink = min(target_shrink, len01, len23)
            if shrink <= 0:
                attempt["status"] = "no_shrink_capacity"
                attempts.append(attempt)
                continue
            unit01 = (sign(v01[0]), sign(v01[1]))
            shift = (-unit01[0] * shrink, -unit01[1] * shrink)
            candidate = list(path)
            candidate[idx + 1] = (p1[0] + shift[0], p1[1] + shift[1])
            candidate[idx + 2] = (p2[0] + shift[0], p2[1] + shift[1])
            candidate = simplify_collinear(candidate)
            candidate_length = manhattan_length(candidate)
            attempt.update(
                {
                    "status": "candidate",
                    "shrink": shrink,
                    "shift": shift,
                    "candidate_length": candidate_length,
                    "candidate_bends": bend_count(candidate),
                }
            )
            if candidate_length >= manhattan_length(path):
                attempt["status"] = "not_shorter"
                attempts.append(attempt)
                continue
            if not route_in_boundary(candidate, boundary_area):
                attempt["status"] = "out_of_boundary"
                attempts.append(attempt)
                continue
            if hard_boundary_area is not None and not route_in_boundary(candidate, hard_boundary_area):
                attempt["status"] = "out_of_hard_boundary"
                attempt["hard_boundary_area"] = list(hard_boundary_area)
                attempts.append(attempt)
                continue
            if has_self_block(candidate):
                attempt["status"] = "self_block"
                attempts.append(attempt)
                continue
            if collides_with_block(candidate, fixed_paths):
                attempt["status"] = "other_block"
                attempts.append(attempt)
                continue
            if route_touches_grid_block(candidate, grid, boundary_area):
                attempt["status"] = "grid_block"
                attempts.append(attempt)
                continue
            attempt["status"] = "valid"
            attempts.append(attempt)
            if candidate_length <= target_length:
                score = (0, target_length - candidate_length, bend_count(candidate), idx)
            else:
                score = (1, candidate_length - target_length, bend_count(candidate), idx)
            candidates.append((score, candidate, attempt))
        return candidates

    while manhattan_length(current) > target_length:
        before_length = manhattan_length(current)
        candidates = one_reduction_candidates(current, before_length - target_length)
        if not candidates:
            return current, {
                "status": "u_bend_reduction_failed",
                "route_length": before_length,
                "target_length": target_length,
                "attempts": attempts,
                "reductions": reductions,
            }
        _score, current, selected = min(candidates, key=lambda item: item[0])
        reductions.append(selected)
        if manhattan_length(current) >= before_length:
            return current, {
                "status": "u_bend_reduction_stalled",
                "route_length": before_length,
                "target_length": target_length,
                "attempts": attempts,
                "reductions": reductions,
            }

    return current, {
        "status": "u_bend_reduced",
        "route_length": manhattan_length(current),
        "target_length": target_length,
        "attempts": attempts,
        "reductions": reductions,
    }


def plan_block_aware_spiral(
    route: list[Iterable[int]],
    fixed_paths: list[list[Iterable[int]]],
    grid,
    boundary_area: list[int],
    hard_boundary_area: list[int] | tuple[int, int, int, int] | None,
    direct: str,
    direction_priority: str,
    target_length: int,
    cur_len: int,
    residual_len: int,
    expected_route_length_grid: int | None,
    placement_policy: str = "legacy_longest",
    preferred_endpoint: Point | None = None,
) -> tuple[list[Point], dict[str, object]]:
    def select_spiral_candidate(
        source_route: list[Iterable[int]],
        length_dt: int,
        expected_route_length_grid: int,
        direction_tier: str = "any",
    ) -> tuple[list[Point], dict[str, object], list[dict[str, object]]]:
        route_points = simplify_collinear([to_point(point) for point in source_route])
        total_route_length = manhattan_length(route_points)
        preferred_anchor: str | None = None
        preferred_point = to_point(preferred_endpoint) if preferred_endpoint is not None else None
        if preferred_point is not None:
            if preferred_point == route_points[0]:
                preferred_anchor = "source"
            elif preferred_point == route_points[-1]:
                preferred_anchor = "target"
        attempts: list[dict[str, object]] = []
        valid_candidates: list[tuple[tuple[int, int, int, int, int], list[Point], dict[str, object]]] = []
        if placement_policy == "nearest_endpoint":
            if preferred_anchor == "source":
                oriented_routes: list[tuple[str, list[Point], bool]] = [
                    ("source", route_points, False),
                ]
            else:
                oriented_routes = [
                    ("target", list(reversed(route_points)), True),
                ]
        else:
            oriented_routes = [("source", route_points, False)]

        required_orientation = detour_segment_orientation(direct)
        for anchor_order, (anchor, oriented_route, reverse_back) in enumerate(oriented_routes):
            prefix_lengths = route_prefix_lengths(oriented_route)
            segment_candidates = manhattan_segment_candidates(oriented_route)
            for segment_order, (seg_len, ind, seg_start, seg_end, orientation) in enumerate(segment_candidates):
                if orientation != required_orientation:
                    attempts.append(
                        {
                            "segment": [seg_start, seg_end],
                            "orientation": orientation,
                            "required_orientation": required_orientation,
                            "endpoint_anchor": anchor,
                            "direct": direct,
                            "status": "segment_orientation_mismatch",
                            "segment_length": seg_len,
                            "length_dt": length_dt,
                            "direction_priority": direction_priority,
                            "direction_tier": direction_tier,
                        }
                    )
                    continue
                seg_dir = (
                    sign(seg_end[0] - seg_start[0]),
                    sign(seg_end[1] - seg_start[1]),
                )
                max_offset = seg_len - 1
                candidate_directions = detour_directions_for_segment(orientation, direct, direction_priority)
                if direction_tier == "primary":
                    candidate_directions = candidate_directions[:1]
                elif direction_tier == "fallback":
                    candidate_directions = candidate_directions[1:]
                elif direction_tier != "any":
                    raise ValueError(f"Unknown direction_tier: {direction_tier}")
                if not candidate_directions:
                    direction_order = (
                        fixed_priority_directions_for_segment(orientation, direction_priority)
                        if uses_fixed_direction_priority(direction_priority)
                        else detour_direction_order(direct)
                    )
                    attempts.append(
                        {
                            "segment": [seg_start, seg_end],
                            "orientation": orientation,
                            "endpoint_anchor": anchor,
                            "direct": direct,
                            "status": "direction_mismatch",
                            "segment_length": seg_len,
                            "length_dt": length_dt,
                            "direction_priority": direction_priority,
                            "direction_order": direction_order,
                            "direction_tier": direction_tier,
                        }
                    )
                    continue

                for offset in range(max_offset + 1):
                    point_start = (
                        seg_start[0] + offset * seg_dir[0],
                        seg_start[1] + offset * seg_dir[1],
                    )
                    _nearest_oriented, oriented_source_distance, oriented_target_distance = source_target_arc_distance(
                        oriented_route,
                        prefix_lengths,
                        ind,
                        point_start,
                    )
                    if reverse_back:
                        target_distance = oriented_source_distance
                        source_distance = total_route_length - oriented_source_distance
                    else:
                        source_distance = oriented_source_distance
                        target_distance = oriented_target_distance
                    endpoint_distance = min(source_distance, target_distance)

                    for side_order, candidate_direct in enumerate(candidate_directions):
                        candidate = apply_simple_spiral(oriented_route, ind, point_start, seg_end, candidate_direct, length_dt)
                        if reverse_back:
                            candidate = list(reversed(candidate))
                        candidate_length = manhattan_length(candidate)
                        attempt = {
                            "segment": [seg_start, seg_end],
                            "orientation": orientation,
                            "endpoint_anchor": anchor,
                            "preferred_direct": direct,
                            "direct": candidate_direct,
                            "direction_priority": direction_priority,
                            "direction_order": candidate_directions,
                            "offset": offset,
                            "point_start": point_start,
                            "length_dt": length_dt,
                            "candidate_length": candidate_length,
                            "source_arc_distance": source_distance,
                            "target_arc_distance": target_distance,
                            "nearest_endpoint_distance": endpoint_distance,
                            "preferred_endpoint": preferred_point,
                            "preferred_anchor": preferred_anchor,
                            "placement_policy": placement_policy,
                            "direction_tier": direction_tier,
                        }
                        if candidate_length != expected_route_length_grid:
                            attempt["status"] = "expected_length_mismatch"
                            attempt["expected_route_length_grid"] = expected_route_length_grid
                            attempts.append(attempt)
                            continue
                        if hard_boundary_area is not None and not route_in_boundary(candidate, hard_boundary_area):
                            attempt["status"] = "out_of_hard_boundary"
                            attempt["hard_boundary_area"] = list(hard_boundary_area)
                            attempts.append(attempt)
                            continue
                        if collides_with_block(candidate, fixed_paths):
                            attempt["status"] = "other_block"
                            attempts.append(attempt)
                            continue
                        stepwise_status, stepwise_point = stepwise_route_violation(candidate, grid, boundary_area)
                        if stepwise_status is not None:
                            attempt["status"] = stepwise_status
                            attempt["blocked_point"] = stepwise_point
                            attempts.append(attempt)
                            continue
                        attempt["status"] = "valid"
                        attempt["bend_count"] = bend_count(candidate)
                        attempts.append(attempt)
                        if placement_policy == "nearest_endpoint":
                            if preferred_anchor == "source":
                                placement_distance = source_distance
                            elif preferred_anchor == "target":
                                placement_distance = target_distance
                            else:
                                placement_distance = endpoint_distance
                            attempt["placement_distance"] = placement_distance
                            score = (placement_distance, anchor_order, -seg_len, side_order, bend_count(candidate))
                        elif placement_policy == "legacy_longest":
                            score = (segment_order, offset, side_order, bend_count(candidate), endpoint_distance)
                        else:
                            raise ValueError(f"Unknown spiral_placement_policy: {placement_policy}")
                        valid_candidates.append((score, candidate, attempt))

        if valid_candidates:
            _score, candidate, selected = min(valid_candidates, key=lambda item: item[0])
            status = "selected_nearest_source_or_target" if placement_policy == "nearest_endpoint" else "selected_legacy_longest"
            return candidate, {"status": status, **selected}, attempts
        raise RuntimeError(f"No block-aware spiral insertion candidate found. Attempts: {attempts}")

    def iterative_spiral_to_length(
        source_route: list[Iterable[int]],
        target_route_length_grid: int,
    ) -> tuple[list[Point], dict[str, object]]:
        current = simplify_collinear([to_point(point) for point in source_route])
        requested_target_route_length_grid = target_route_length_grid
        steps: list[dict[str, object]] = []
        all_attempts: list[dict[str, object]] = []
        while manhattan_length(current) < target_route_length_grid:
            current_length = manhattan_length(current)
            remaining = target_route_length_grid - current_length
            max_length_dt = remaining // 2
            if max_length_dt <= 0:
                all_attempts.append(
                    {
                        "status": "no_non_exceeding_even_increment_available",
                        "current_length": current_length,
                        "target_route_length_grid": target_route_length_grid,
                        "remaining_grid": remaining,
                    }
                )
                break

            selected = None
            per_step_attempts: list[dict[str, object]] = []
            for direction_tier in ("primary", "fallback"):
                for length_dt_candidate in range(max_length_dt, 0, -1):
                    expected_after = current_length + 2 * length_dt_candidate
                    if expected_after > target_route_length_grid:
                        continue
                    try:
                        candidate, step_plan, attempts = select_spiral_candidate(
                            current,
                            length_dt_candidate,
                            expected_after,
                            direction_tier=direction_tier,
                        )
                        selected = (candidate, step_plan, attempts)
                        break
                    except RuntimeError as exc:
                        per_step_attempts.append(
                            {
                                "direction_tier": direction_tier,
                                "length_dt": length_dt_candidate,
                                "status": "failed",
                                "error": compact_error(exc),
                            }
                        )
                if selected is not None:
                    break
            if selected is None:
                all_attempts.extend(per_step_attempts)
                break

            current, step_plan, attempts = selected
            all_attempts.extend(attempts)
            steps.append(
                {
                    **step_plan,
                    "before_length": current_length,
                    "after_length": manhattan_length(current),
                    "target_route_length_grid": target_route_length_grid,
                    "iteration": len(steps),
                }
            )
        return current, {
            "status": "selected"
            if manhattan_length(current) == requested_target_route_length_grid
            else "closest_not_exceed_selected",
            "route_length": manhattan_length(current),
            "requested_target_route_length_grid": requested_target_route_length_grid,
            "target_route_length_grid": min(target_route_length_grid, requested_target_route_length_grid),
            "target_gap_grid": requested_target_route_length_grid - manhattan_length(current),
            "iterations": len(steps),
            "steps": steps,
            "attempts": all_attempts,
        }

    route_length = manhattan_length([to_point(point) for point in route])
    if expected_route_length_grid is not None:
        if route_length > expected_route_length_grid:
            reduced_route, reduction_plan = shorten_u_bends_to_length(
                route,
                expected_route_length_grid,
                fixed_paths,
                grid,
                boundary_area,
                hard_boundary_area,
            )
            reduced_length = manhattan_length(reduced_route)
            if reduced_length <= expected_route_length_grid:
                return reduced_route, {
                    "status": "u_bend_reduced",
                    "route_length": route_length,
                    "reduced_route_length": reduced_length,
                    "expected_route_length_grid": expected_route_length_grid,
                    "target_gap_grid": expected_route_length_grid - reduced_length,
                    "reduction_plan": reduction_plan,
                }
            raise RuntimeError(
                f"Route length {route_length} already exceeds expected route length {expected_route_length_grid}; "
                f"U-bend reduction failed: {reduction_plan}"
            )
        target_route_length_grid = expected_route_length_grid
    else:
        target_route_length_grid = target_length - 1 - residual_len
    if target_route_length_grid <= route_length:
        return simplify_collinear([to_point(point) for point in route]), {
            "status": "no_insert_needed",
            "route_length": route_length,
            "target_route_length_grid": target_route_length_grid,
            "expected_route_length_grid": expected_route_length_grid,
        }

    return iterative_spiral_to_length(route, target_route_length_grid)


def derive_target_residual_lengths(
    expected_explicit_lengths_grid: tuple[int, ...] | None,
    target_total_length_grid: int,
    net_count: int,
) -> tuple[int, ...]:
    if expected_explicit_lengths_grid is None:
        return tuple(0 for _ in range(net_count))
    if len(expected_explicit_lengths_grid) != net_count:
        raise ValueError(
            f"Expected {net_count} target lengths, got {len(expected_explicit_lengths_grid)}"
        )
    return tuple(target_total_length_grid - length for length in expected_explicit_lengths_grid)


def compact_error(exc: Exception, max_len: int = 220) -> str:
    message = f"{type(exc).__name__}: {exc}"
    if len(message) <= max_len:
        return message
    return message[: max_len - 3] + "..."


def paths_cross_or_touch(paths: PathData) -> bool:
    occupied: set[Point] = set()
    occupied_edges: set[tuple[Point, Point]] = set()
    for route in paths:
        points = walk_points(route)
        for point in points:
            if point in occupied:
                return True
            occupied.add(point)
        for edge in unit_edges(route):
            if edge in occupied_edges:
                return True
            occupied_edges.add(edge)
    return False


def infer_detour_region_orientation(paths: PathData) -> str:
    routes = sort_routes(paths)
    same_source_x = len({route[0][0] for route in routes}) == 1
    same_target_x = len({route[-1][0] for route in routes}) == 1
    if same_source_x and same_target_x:
        start_order = sorted(range(len(routes)), key=lambda idx: (routes[idx][0][1], routes[idx][0][0]))
        end_order = sorted(range(len(routes)), key=lambda idx: (routes[idx][-1][1], routes[idx][-1][0]))
        if start_order != end_order:
            raise ValueError("Horizontal detour region violates relative endpoint order.")
        return "horizontal"

    same_source_y = len({route[0][1] for route in routes}) == 1
    same_target_y = len({route[-1][1] for route in routes}) == 1
    if same_source_y and same_target_y:
        start_order = sorted(range(len(routes)), key=lambda idx: (routes[idx][0][0], routes[idx][0][1]))
        end_order = sorted(range(len(routes)), key=lambda idx: (routes[idx][-1][0], routes[idx][-1][1]))
        if start_order != end_order:
            raise ValueError("Vertical detour region violates relative endpoint order.")
        return "vertical"

    raise ValueError("Could not infer a detour region: paths do not enter and exit through the same two sides.")


def base_detour_boundary_area(paths: PathData) -> tuple[list[int], dict[str, object]]:
    if paths_cross_or_touch(paths):
        raise ValueError("Could not infer a detour region: initial paths cross or touch.")
    orientation = infer_detour_region_orientation(paths)
    xs = [point[0] for route in paths for point in route]
    ys = [point[1] for route in paths for point in route]
    boundary_area = [min(xs) - 1, max(xs) + 1, min(ys) - 1, max(ys) + 1]
    return boundary_area, {
        "orientation": orientation,
        "base_boundary_area": list(boundary_area),
        "source": "path_order_and_outermost_paths",
    }


def auto_region_extension_limit(
    source_paths: PathData,
    expected_explicit_lengths_grid: tuple[int, ...] | None,
    target_total_length_grid: int,
) -> int:
    current_lengths = route_lengths(source_paths)
    if expected_explicit_lengths_grid is not None:
        target_lengths = list(expected_explicit_lengths_grid)
    else:
        target_lengths = [target_total_length_grid for _ in current_lengths]
    max_delta = max([target - current for target, current in zip(target_lengths, current_lengths)] + [0])
    return max(1, max_delta // 2 + len(current_lengths))


def expand_boundary_area(boundary_area: list[int], side: str, amount: int) -> list[int]:
    x_min, x_max, y_min, y_max = boundary_area
    if side == "top":
        y_max += amount
    elif side == "bottom":
        y_min -= amount
    elif side == "right":
        x_max += amount
    elif side == "left":
        x_min -= amount
    else:
        raise ValueError(f"Unknown boundary side: {side}")
    return [x_min, x_max, y_min, y_max]


def contains_boundary_area(outer: tuple[int, int, int, int] | list[int], inner: tuple[int, int, int, int] | list[int]) -> bool:
    return outer[0] <= inner[0] and inner[1] <= outer[1] and outer[2] <= inner[2] and inner[3] <= outer[3]


def expand_search_area_to_contain_base(
    search_area: tuple[int, int, int, int] | list[int],
    base_boundary_area: list[int],
) -> list[int]:
    return [
        min(int(search_area[0]), int(base_boundary_area[0])),
        max(int(search_area[1]), int(base_boundary_area[1])),
        min(int(search_area[2]), int(base_boundary_area[2])),
        max(int(search_area[3]), int(base_boundary_area[3])),
    ]


def boundary_with_outer_halo(boundary_area: list[int] | tuple[int, int, int, int]) -> list[int]:
    return [int(boundary_area[0]) - 1, int(boundary_area[1]) + 1, int(boundary_area[2]) - 1, int(boundary_area[3]) + 1]


def max_empty_extension_grid(
    paths: PathData,
    base_boundary_area: list[int],
    search_area: tuple[int, int, int, int],
    side: str,
) -> int:
    x_min, x_max, y_min, y_max = base_boundary_area
    search_x_min, search_x_max, search_y_min, search_y_max = search_area
    occupied = {point for route in paths for point in walk_points(route)}

    def row_has_block(y: int) -> bool:
        return any((x, y) in occupied for x in range(x_min, x_max + 1))

    def column_has_block(x: int) -> bool:
        return any((x, y) in occupied for y in range(y_min, y_max + 1))

    amount = 0
    if side == "top":
        for y in range(y_max + 1, search_y_max + 1):
            if row_has_block(y):
                break
            amount += 1
    elif side == "bottom":
        for y in range(y_min - 1, search_y_min - 1, -1):
            if row_has_block(y):
                break
            amount += 1
    elif side == "right":
        for x in range(x_max + 1, search_x_max + 1):
            if column_has_block(x):
                break
            amount += 1
    elif side == "left":
        for x in range(x_min - 1, search_x_min - 1, -1):
            if column_has_block(x):
                break
            amount += 1
    else:
        raise ValueError(f"Unknown boundary side: {side}")
    return amount


def resource_extension_sides(orientation: str) -> tuple[str, str]:
    if orientation == "horizontal":
        return "top", "bottom"
    if orientation == "vertical":
        return "right", "left"
    raise ValueError(f"Unknown detour region orientation: {orientation}")


def boundary_edge_length_grid(boundary_area: list[int] | tuple[int, int, int, int], side: str) -> int:
    x_min, x_max, y_min, y_max = boundary_area
    if side in {"top", "bottom"}:
        return max(0, x_max - x_min)
    if side in {"right", "left"}:
        return max(0, y_max - y_min)
    raise ValueError(f"Unknown boundary side: {side}")


def boundary_extent_area_grid(boundary_area: list[int] | tuple[int, int, int, int]) -> int:
    x_min, x_max, y_min, y_max = boundary_area
    return max(0, x_max - x_min) * max(0, y_max - y_min)


def cell_in_boundary(cell: Point, boundary_area: list[int] | tuple[int, int, int, int]) -> bool:
    x_min, x_max, y_min, y_max = boundary_area
    return x_min <= cell[0] < x_max and y_min <= cell[1] < y_max


def occupied_grid_cells_for_route(
    route: list[Iterable[int]],
    boundary_area: list[int] | tuple[int, int, int, int],
) -> set[Point]:
    cells: set[Point] = set()
    points = [to_point(point) for point in route]
    if not points:
        return cells
    if len(points) == 1:
        if cell_in_boundary(points[0], boundary_area):
            cells.add(points[0])
        return cells
    for raw_a, raw_b in zip(points, points[1:]):
        a = to_point(raw_a)
        b = to_point(raw_b)
        dx = sign(b[0] - a[0])
        dy = sign(b[1] - a[1])
        if dx and dy:
            raise ValueError(f"Non-manhattan segment: {a} -> {b}")
        if a == b:
            if cell_in_boundary(a, boundary_area):
                cells.add(a)
            continue
        cur = a
        while cur != b:
            nxt = (cur[0] + dx, cur[1] + dy)
            if dx:
                cell = (min(cur[0], nxt[0]), cur[1])
            else:
                cell = (cur[0], min(cur[1], nxt[1]))
            if cell_in_boundary(cell, boundary_area):
                cells.add(cell)
            cur = nxt
    return cells


def occupied_grid_cells_in_boundary(paths: PathData, boundary_area: list[int] | tuple[int, int, int, int]) -> set[Point]:
    occupied: set[Point] = set()
    for route in paths:
        occupied.update(occupied_grid_cells_for_route(route, boundary_area))
    return occupied


def detour_area_required_grid(
    config: LegacyContourDetourConfig,
    source_paths: PathData,
) -> tuple[int, list[int], list[int], int]:
    current_lengths = route_lengths(source_paths)
    target_length = config.target_total_length_grid
    per_net = [max(0, target_length - current_length) for current_length in current_lengths]
    return sum(per_net), per_net, current_lengths, target_length


def unique_longest_index(lengths: list[int] | tuple[int, ...]) -> int | None:
    if not lengths:
        return None
    longest = max(lengths)
    indices = [idx for idx, length in enumerate(lengths) if length == longest]
    return indices[0] if len(indices) == 1 else None


def choose_partition_longest_index(
    lengths: list[int] | tuple[int, ...],
    current_lengths: list[int],
    target_lengths: list[int],
) -> int | None:
    if not lengths:
        return None
    longest = max(lengths)
    candidates = [
        idx
        for idx, length in enumerate(lengths)
        if length == longest
        and idx not in {0, len(lengths) - 1}
        and current_lengths[idx] >= target_lengths[idx]
    ]
    if not candidates:
        return None
    center = (len(lengths) - 1) / 2
    return min(candidates, key=lambda idx: (abs(idx - center), idx))


def longest_boundary_partition_plan(
    source_paths: PathData,
    config: LegacyContourDetourConfig,
    orientation: str,
) -> dict[str, object]:
    ordered_paths = sort_routes(source_paths)
    net_count = len(ordered_paths)
    current_lengths = route_lengths(ordered_paths)
    analyses: list[dict[str, object]] = []
    fixed_target_lengths = (
        list(config.expected_explicit_lengths_grid)
        if config.expected_explicit_lengths_grid is not None
        else [config.target_total_length_grid for _ in range(net_count)]
    )

    def add_analysis(source: str, lengths: list[int]) -> None:
        tied_longest_indices = [
            idx for idx, length in enumerate(lengths) if lengths and length == max(lengths)
        ]
        idx = choose_partition_longest_index(lengths, current_lengths, fixed_target_lengths)
        outer = idx in {0, net_count - 1} if idx is not None else None
        fixed_reaches_target = (
            bool(current_lengths[idx] >= fixed_target_lengths[idx])
            if idx is not None and idx < len(fixed_target_lengths)
            else None
        )
        analysis: dict[str, object] = {
            "source": source,
            "lengths_grid": lengths,
            "longest_indices": tied_longest_indices,
            "selected_longest_index": idx,
            "unique_longest_index": idx,
            "longest_is_outer_boundary": outer,
            "fixed_candidate_target_length_grid": fixed_target_lengths[idx] if idx is not None else None,
            "fixed_candidate_current_length_grid": current_lengths[idx] if idx is not None else None,
            "fixed_candidate_reaches_target": fixed_reaches_target,
            "partition_required": bool(idx is not None),
        }
        if idx is not None:
            analysis["longest_length_grid"] = lengths[idx]
        analyses.append(analysis)

    add_analysis("current_path_lengths", current_lengths)
    if config.expected_explicit_lengths_grid is not None:
        if len(config.expected_explicit_lengths_grid) != net_count:
            raise ValueError(
                f"Expected {net_count} explicit lengths for longest-boundary analysis, "
                f"got {len(config.expected_explicit_lengths_grid)}"
            )
        add_analysis("expected_explicit_lengths", list(config.expected_explicit_lengths_grid))

    partition_analysis = next(
        (
            analysis
            for analysis in analyses
            if analysis["source"] == "current_path_lengths" and analysis["partition_required"]
        ),
        None,
    )
    partition_index = partition_analysis["unique_longest_index"] if partition_analysis is not None else None
    side_names = ("lower", "upper") if orientation == "horizontal" else ("left", "right")
    partition: dict[str, object] | None = None
    if isinstance(partition_index, int):
        partition = {
            "fixed_longest_index": partition_index,
            "fixed_longest_source": partition_analysis["source"],
            "fixed_longest_length_grid": partition_analysis.get("longest_length_grid"),
            "partitions": [
                {
                    "side": side_names[0],
                    "path_indices": list(range(0, partition_index + 1)),
                    "shared_fixed_boundary_index": partition_index,
                },
                {
                    "side": side_names[1],
                    "path_indices": list(range(partition_index, net_count)),
                    "shared_fixed_boundary_index": partition_index,
                },
            ],
        }

    return {
        "policy": "longest_path_must_be_outer_boundary_or_partition_region",
        "orientation": orientation,
        "ordered_path_count": net_count,
        "order_axis": "y" if orientation == "horizontal" else "x",
        "analyses": analyses,
        "partition_required": partition is not None,
        "partition": partition,
    }


def detour_area_slack_check(
    boundary_area: list[int] | tuple[int, int, int, int],
    source_paths: PathData,
    external_obstacle_paths: PathData,
    config: LegacyContourDetourConfig,
) -> dict[str, object]:
    if config.detour_area_slack_delta is None:
        return {"enabled": False}
    delta = float(config.detour_area_slack_delta)
    required_area, required_per_net, current_lengths, target_length = detour_area_required_grid(
        config,
        source_paths,
    )
    total_area = boundary_extent_area_grid(boundary_area)
    source_cells = occupied_grid_cells_in_boundary(source_paths, boundary_area)
    external_cells = occupied_grid_cells_in_boundary(external_obstacle_paths, boundary_area)
    occupied_cells = source_cells | external_cells
    available_area = max(0, total_area - len(occupied_cells))
    required_with_slack = required_area * (1.0 + delta)
    return {
        "enabled": True,
        "delta": delta,
        "boundary_area": list(boundary_area),
        "area_model": "unused_grid_cells_in_detour_region",
        "required_area_model": "sum(max(0, longest_length_grid - current_path_length_grid))",
        "total_area_grid": total_area,
        "source_occupied_area_grid": len(source_cells),
        "external_blocked_area_grid": len(external_cells),
        "occupied_area_grid": len(occupied_cells),
        "overlap_occupied_area_grid": len(source_cells) + len(external_cells) - len(occupied_cells),
        "available_area_grid": available_area,
        "target_longest_length_grid": target_length,
        "current_lengths_grid": current_lengths,
        "required_area_grid": required_area,
        "required_area_per_net_grid": required_per_net,
        "required_area_with_slack_grid": required_with_slack,
        "passes": available_area + 1e-9 >= required_with_slack,
    }


def auto_region_candidates_from_search_area(
    base_boundary_area: list[int],
    orientation: str,
    search_area: tuple[int, int, int, int],
    source_paths: PathData,
    external_obstacle_paths: PathData,
    config: LegacyContourDetourConfig,
) -> tuple[list[dict[str, object]], dict[str, object]]:
    requested_search_area = list(search_area)
    if not contains_boundary_area(search_area, base_boundary_area):
        search_area = tuple(expand_search_area_to_contain_base(search_area, base_boundary_area))

    sides = resource_extension_sides(orientation)
    side_order = {side: order for order, side in enumerate(sides)}
    options = []
    occupancy_paths = source_paths + external_obstacle_paths
    for side in sides:
        clearance = max_empty_extension_grid(occupancy_paths, base_boundary_area, search_area, side)
        options.append(
            {
                "extension_side": side,
                "extension_clearance_grid": clearance,
                "boundary_edge_length_grid": boundary_edge_length_grid(base_boundary_area, side),
            }
        )
    options.sort(
        key=lambda item: (
            -int(item["boundary_edge_length_grid"]),
            -int(item["extension_clearance_grid"]),
            side_order[str(item["extension_side"])],
        )
    )
    candidates: list[dict[str, object]] = [
        {
            "extension_side": "none",
            "extension_grid": 0,
            "boundary_area": list(base_boundary_area),
            "boundary_edge_length_grid": None,
            "candidate_policy": "base_detour_region_before_extension",
        }
    ]
    base_area_check = detour_area_slack_check(
        base_boundary_area,
        source_paths,
        external_obstacle_paths,
        config,
    )
    if not base_area_check["enabled"]:
        for option in options:
            clearance = int(option["extension_clearance_grid"])
            if clearance <= 0:
                continue
            side = str(option["extension_side"])
            candidates.append(
                {
                    "extension_side": side,
                    "extension_grid": clearance,
                    "boundary_area": expand_boundary_area(base_boundary_area, side, clearance),
                    "boundary_edge_length_grid": option["boundary_edge_length_grid"],
                    "candidate_policy": "max_clearance_without_area_slack",
                }
            )
    elif not bool(base_area_check["passes"]):
        for option in options:
            side = str(option["extension_side"])
            clearance = int(option["extension_clearance_grid"])
            minimum_passing_amount: int | None = None
            minimum_passing_check: dict[str, object] | None = None
            for amount in range(1, clearance + 1):
                area_check = detour_area_slack_check(
                    expand_boundary_area(base_boundary_area, side, amount),
                    source_paths,
                    external_obstacle_paths,
                    config,
                )
                if bool(area_check["passes"]):
                    minimum_passing_amount = amount
                    minimum_passing_check = area_check
                    break
            option["minimum_area_passing_extension_grid"] = minimum_passing_amount
            if minimum_passing_check is not None:
                option["minimum_area_passing_available_area_grid"] = minimum_passing_check["available_area_grid"]
                option["minimum_area_passing_required_with_slack_grid"] = minimum_passing_check[
                    "required_area_with_slack_grid"
                ]
            if minimum_passing_amount is None:
                continue
            for amount in range(minimum_passing_amount, clearance + 1):
                candidates.append(
                    {
                        "extension_side": side,
                        "extension_grid": amount,
                        "boundary_area": expand_boundary_area(base_boundary_area, side, amount),
                        "boundary_edge_length_grid": option["boundary_edge_length_grid"],
                        "candidate_policy": (
                            "minimum_area_passing_extension"
                            if amount == minimum_passing_amount
                            else "post_area_pass_detour_fallback"
                        ),
                    }
                )
    return candidates, {
        "selection_mode": "paper_area_direct_longest_edge",
        "expansion_policy": "test_base_region_then_compute_minimum_longest_edge_extension_from_available_area",
        "requested_search_boundary_area": requested_search_area,
        "search_boundary_area": list(search_area),
        "search_boundary_area_adjusted_to_contain_base": list(search_area) != requested_search_area,
        "allowed_extension_sides": list(sides),
        "base_area_slack_check": base_area_check,
        "extension_options": options,
    }


def auto_region_candidates(base_boundary_area: list[int], orientation: str, extension_limit: int) -> list[dict[str, object]]:
    candidates: list[dict[str, object]] = [
        {
            "extension_side": "none",
            "extension_grid": 0,
            "boundary_area": list(base_boundary_area),
            "boundary_edge_length_grid": None,
            "candidate_policy": "base_detour_region_before_extension",
        }
    ]
    sides = resource_extension_sides(orientation)
    side_order = {side: order for order, side in enumerate(sides)}
    options = [
        {
            "extension_side": side,
            "extension_clearance_grid": extension_limit,
            "boundary_edge_length_grid": boundary_edge_length_grid(base_boundary_area, side),
        }
        for side in sides
    ]
    options.sort(
        key=lambda item: (
            -int(item["boundary_edge_length_grid"]),
            -int(item["extension_clearance_grid"]),
            side_order[str(item["extension_side"])],
        )
    )
    for amount in range(1, extension_limit + 1):
        for option in options:
            side = str(option["extension_side"])
            candidates.append(
                {
                    "extension_side": side,
                    "extension_grid": amount,
                    "boundary_area": expand_boundary_area(base_boundary_area, side, amount),
                    "boundary_edge_length_grid": option["boundary_edge_length_grid"],
                    "candidate_policy": "increment_longest_expandable_edge",
                }
            )
    return candidates


def auto_region_result_matches(
    result: DetourResult,
    expected_explicit_lengths_grid: tuple[int, ...] | None,
) -> tuple[bool, list[int]]:
    lengths = route_lengths(result.paths)
    if expected_explicit_lengths_grid is None:
        return True, lengths
    if len(lengths) != len(expected_explicit_lengths_grid):
        return False, lengths
    return all(length <= expected for length, expected in zip(lengths, expected_explicit_lengths_grid)), lengths


def non_exceed_gap_score(
    lengths: list[int],
    expected_explicit_lengths_grid: tuple[int, ...] | None,
) -> tuple[int, int, int]:
    if expected_explicit_lengths_grid is None:
        return (0, 0, 0)
    if len(lengths) != len(expected_explicit_lengths_grid):
        return (1, 10**9, 10**9)
    over = [max(0, length - expected) for length, expected in zip(lengths, expected_explicit_lengths_grid)]
    under = [max(0, expected - length) for length, expected in zip(lengths, expected_explicit_lengths_grid)]
    return (sum(over), sum(under), max(under, default=0))


def subset_tuple(values: tuple[int, ...] | None, indices: list[int]) -> tuple[int, ...] | None:
    if values is None:
        return None
    return tuple(values[index] for index in indices)


def subset_point_tuple(values: tuple[Point, ...] | None, indices: list[int]) -> tuple[Point, ...] | None:
    if values is None:
        return None
    return tuple(values[index] for index in indices)


def endpoint_path_map(paths: PathData) -> dict[tuple[Point, Point], list[Point]]:
    return {endpoint_key(route): simplify_collinear([to_point(point) for point in route]) for route in paths}


def partition_boundary_area(
    boundary_area: list[int] | tuple[int, int, int, int],
    fixed_path: list[Iterable[int]],
    orientation: str,
    side: str,
) -> list[int]:
    x_min, x_max, y_min, y_max = boundary_area
    fixed = [to_point(point) for point in fixed_path]
    fixed_x_min = min(point[0] for point in fixed)
    fixed_x_max = max(point[0] for point in fixed)
    fixed_y_min = min(point[1] for point in fixed)
    fixed_y_max = max(point[1] for point in fixed)
    if orientation == "horizontal":
        if side == "lower":
            return [x_min, x_max, y_min, fixed_y_max + 1]
        if side == "upper":
            return [x_min, x_max, fixed_y_min - 1, y_max]
    elif orientation == "vertical":
        if side == "left":
            return [x_min, fixed_x_max + 1, y_min, y_max]
        if side == "right":
            return [fixed_x_min - 1, x_max, y_min, y_max]
    raise ValueError(f"Cannot split boundary_area={boundary_area} for orientation={orientation}, side={side}")


def with_partition_snapshot_prefix(snapshots: list[dict[str, object]], prefix: str) -> list[dict[str, object]]:
    prefixed: list[dict[str, object]] = []
    for snapshot in snapshots:
        item = dict(snapshot)
        item["title"] = f"{prefix}: {snapshot.get('title', '')}"
        item["partition"] = prefix
        prefixed.append(item)
    return prefixed


def run_partitioned_candidate_detour(
    astar_paths: PathData,
    config: LegacyContourDetourConfig,
    source_paths: PathData,
    boundary_area: list[int] | tuple[int, int, int, int],
    schedule_plan: dict[str, object],
) -> DetourResult:
    partition = schedule_plan.get("partition")
    if not isinstance(partition, dict):
        raise ValueError(f"Missing partition plan: {schedule_plan}")
    fixed_index = int(partition["fixed_longest_index"])
    orientation = str(schedule_plan["orientation"])
    ordered_astar = sort_routes(astar_paths)
    ordered_source = sort_routes(source_paths)
    fixed_astar = ordered_astar[fixed_index]
    fixed_source = ordered_source[fixed_index]

    merged_with: dict[int, list[Point]] = {fixed_index: simplify_collinear([to_point(point) for point in fixed_astar])}
    merged_source: dict[int, list[Point]] = {fixed_index: simplify_collinear([to_point(point) for point in fixed_source])}
    subresults: list[dict[str, object]] = []
    snapshots: list[dict[str, object]] = []
    logs: list[str] = []

    base_external_paths = normalize_obstacle_paths(config.external_obstacle_paths_grid)
    fixed_obstacle_paths = base_external_paths + [simplify_collinear([to_point(point) for point in fixed_source])]

    for raw_part in partition.get("partitions", []):
        part = dict(raw_part)
        side = str(part["side"])
        indices = [int(index) for index in part["path_indices"] if int(index) != fixed_index]
        if not indices:
            subresults.append({"side": side, "active_indices": indices, "status": "empty"})
            continue
        side_boundary = partition_boundary_area(boundary_area, fixed_source, orientation, side)
        sub_config = replace(
            config,
            expected_explicit_lengths_grid=subset_tuple(config.expected_explicit_lengths_grid, indices),
            residual_lengths_grid=subset_tuple(config.residual_lengths_grid, indices),
            spiral_preferred_endpoints_grid=subset_point_tuple(config.spiral_preferred_endpoints_grid, indices),
            boundary_area_grid=tuple(side_boundary),
            boundary_top_padding_grid=None,
            auto_region_search_area_grid=None,
            external_obstacle_paths_grid=fixed_obstacle_paths,
        )
        sub_astar = [ordered_astar[index] for index in indices]
        sub_result = run_legacy_contour_detour(sub_astar, sub_config)
        paths_by_endpoint = endpoint_path_map(sub_result.paths)
        for index in indices:
            astar_key = endpoint_key(ordered_astar[index])
            source_key = endpoint_key(ordered_source[index])
            if astar_key not in paths_by_endpoint:
                raise RuntimeError(f"Partition {side} did not return path for endpoint {astar_key}")
            if source_key not in paths_by_endpoint:
                raise RuntimeError(f"Partition {side} did not return source path for endpoint {source_key}")
            merged_with[index] = paths_by_endpoint[astar_key]
            merged_source[index] = paths_by_endpoint[source_key]
        logs.append(f"partition {side} indices={indices}\n{sub_result.log}")
        snapshots.extend(with_partition_snapshot_prefix(sub_result.snapshots, f"partition {side}"))
        subresults.append(
            {
                "side": side,
                "active_indices": indices,
                "boundary_area": side_boundary,
                "status": "routed",
                "lengths_grid": route_lengths(sub_result.paths),
                "metadata": sub_result.metadata,
            }
        )

    missing = [index for index in range(len(ordered_astar)) if index not in merged_with or index not in merged_source]
    if missing:
        raise RuntimeError(f"Partitioned detour did not produce all paths; missing indices {missing}")
    merged_with_paths = [merged_with[index] for index in range(len(ordered_astar))]
    metadata = {
        "source": "partitioned_contour_detour",
        "boundary_area": list(boundary_area),
        "longest_boundary_schedule": schedule_plan,
        "fixed_longest_index": fixed_index,
        "subresults": subresults,
    }
    return DetourResult(
        paths=merged_with_paths,
        log="\n".join(logs),
        metadata=metadata,
        snapshots=snapshots,
    )


def run_auto_region_detour(
    astar_paths: PathData,
    config: LegacyContourDetourConfig,
    source_paths: PathData,
) -> DetourResult:
    base_boundary_area, region_metadata = base_detour_boundary_area(source_paths)
    schedule_plan = longest_boundary_partition_plan(
        source_paths,
        config,
        str(region_metadata["orientation"]),
    )
    external_obstacle_paths = config_external_obstacle_paths(config)
    extension_limit: int | None = None
    if config.auto_region_search_area_grid is not None:
        candidates, selection_metadata = auto_region_candidates_from_search_area(
            base_boundary_area,
            str(region_metadata["orientation"]),
            config.auto_region_search_area_grid,
            source_paths,
            external_obstacle_paths,
            config,
        )
    else:
        extension_limit = auto_region_extension_limit(
            source_paths,
            config.expected_explicit_lengths_grid,
            config.target_total_length_grid,
        )
        candidates = auto_region_candidates(base_boundary_area, str(region_metadata["orientation"]), extension_limit)
        selection_metadata = {
            "selection_mode": "paper_longest_edge_area_slack_fallback",
            "expansion_policy": "test_base_region_then_increment_longest_expandable_edge_until_area_slack_passes",
            "extension_limit_grid": extension_limit,
            "allowed_extension_sides": list(resource_extension_sides(str(region_metadata["orientation"]))),
        }
    candidate_metadata: list[dict[str, object]] = []
    best_result: DetourResult | None = None
    best_record: dict[str, object] | None = None
    best_score: tuple[int, int, int] | None = None

    def finish_result(result: DetourResult, selected_record: dict[str, object], validation: str) -> DetourResult:
        result.metadata["boundary_area_source"] = "auto_detour_region"
        result.metadata["auto_detour_region"] = {
            **region_metadata,
            **selection_metadata,
            "longest_boundary_schedule": schedule_plan,
            "selected_boundary_area": list(selected_record["boundary_area"]),
            "extension_side": selected_record["extension_side"],
            "extension_grid": selected_record["extension_grid"],
            "validation": validation,
            "candidates": candidate_metadata,
        }
        return result

    def remember_best(result: DetourResult, record: dict[str, object], lengths: list[int]) -> bool:
        nonlocal best_result, best_record, best_score
        score = non_exceed_gap_score(lengths, config.expected_explicit_lengths_grid)
        record["gap_score"] = {
            "total_over_target_grid": score[0],
            "total_under_target_grid": score[1],
            "max_under_target_grid": score[2],
        }
        if score[0] > 0:
            return False
        if best_score is None or score < best_score:
            best_result = result
            best_record = record
            best_score = score
        return score[1] == 0

    for candidate in candidates:
        area_check = detour_area_slack_check(
            candidate["boundary_area"],
            source_paths,
            external_obstacle_paths,
            config,
        )
        candidate_config = replace(
            config,
            boundary_area_grid=tuple(candidate["boundary_area"]),
            boundary_top_padding_grid=None,
        )
        record = dict(candidate)
        if area_check["enabled"]:
            record["area_slack_check"] = area_check
            if not area_check["passes"]:
                record["status"] = "area_slack_insufficient"
                candidate_metadata.append(record)
                continue
        try:
            if schedule_plan.get("partition_required"):
                    result = run_partitioned_candidate_detour(
                        astar_paths,
                        candidate_config,
                        source_paths,
                        candidate["boundary_area"],
                        schedule_plan,
                    )
            else:
                result = run_legacy_contour_detour(astar_paths, candidate_config)
            matches, lengths = auto_region_result_matches(
                result,
                config.expected_explicit_lengths_grid,
            )
            record["lengths_grid"] = lengths
            target_matched = remember_best(result, record, lengths) if matches else False
            record["status"] = "selected_target" if target_matched else ("under_target_candidate" if matches else "length_over_target")
            candidate_metadata.append(record)
            if config.expected_explicit_lengths_grid is None:
                return finish_result(result, record, "first_feasible_detour")
            if target_matched:
                return finish_result(result, record, "closest_not_exceed_target")
            if config.spiral_placement_policy == "nearest_endpoint" and not matches:
                fallback_record = {
                    "extension_side": candidate["extension_side"],
                    "extension_grid": candidate["extension_grid"],
                    "boundary_area": list(candidate["boundary_area"]),
                    "candidate_policy": "legacy_longest_fallback_after_nearest_endpoint_mismatch",
                }
                try:
                    fallback_config = replace(candidate_config, spiral_placement_policy="legacy_longest")
                    if schedule_plan.get("partition_required"):
                        fallback_result = run_partitioned_candidate_detour(
                            astar_paths,
                            fallback_config,
                            source_paths,
                            candidate["boundary_area"],
                            schedule_plan,
                        )
                    else:
                        fallback_result = run_legacy_contour_detour(astar_paths, fallback_config)
                    fallback_matches, fallback_lengths = auto_region_result_matches(
                        fallback_result,
                        config.expected_explicit_lengths_grid,
                    )
                    fallback_record["lengths_grid"] = fallback_lengths
                    fallback_target_matched = remember_best(fallback_result, fallback_record, fallback_lengths) if fallback_matches else False
                    fallback_record["status"] = (
                        "selected_target" if fallback_target_matched else ("under_target_candidate" if fallback_matches else "length_over_target")
                    )
                    candidate_metadata.append(fallback_record)
                    if fallback_matches and fallback_result is best_result:
                        fallback_result.metadata["nearest_endpoint_fallback"] = {
                            "reason": "legacy-longest placement is closer without exceeding target",
                            "fallback_policy": "legacy_longest",
                        }
                    if fallback_target_matched:
                        return finish_result(fallback_result, fallback_record, "closest_not_exceed_target")
                except Exception as fallback_exc:
                    fallback_record["status"] = "failed"
                    fallback_record["error"] = compact_error(fallback_exc)
                    candidate_metadata.append(fallback_record)
        except Exception as exc:
            record["status"] = "failed"
            record["error"] = compact_error(exc)
            candidate_metadata.append(record)

    if best_result is not None and best_record is not None:
        return finish_result(best_result, best_record, "closest_not_exceed_target")

    raise RuntimeError(
        "Auto detour region search failed. "
        f"Base={base_boundary_area}, selection={selection_metadata}, candidates={candidate_metadata}"
    )


def remove_duplicate_points(route: list[Point]) -> list[Point]:
    clean: list[Point] = []
    for point in route:
        if not clean or clean[-1] != point:
            clean.append(point)
    return clean


def simplify_collinear(route: list[Point]) -> list[Point]:
    route = remove_duplicate_points([to_point(point) for point in route])
    if len(route) <= 2:
        return route
    simplified = [route[0]]
    for i in range(1, len(route) - 1):
        prev = simplified[-1]
        cur = route[i]
        nxt = route[i + 1]
        if (prev[0] == cur[0] == nxt[0]) or (prev[1] == cur[1] == nxt[1]):
            continue
        simplified.append(cur)
    simplified.append(route[-1])
    return simplified


def first_walk_index_at_x(walked: list[Point], x: int) -> int:
    for idx, point in enumerate(walked):
        if point[0] == x:
            return idx
    raise ValueError(f"Route never reaches x={x}: {walked}")


def last_walk_index_at_x(walked: list[Point], x: int) -> int:
    for idx in range(len(walked) - 1, -1, -1):
        if walked[idx][0] == x:
            return idx
    raise ValueError(f"Route never reaches x={x}: {walked}")


def first_walk_index_at_y(walked: list[Point], y: int) -> int:
    for idx, point in enumerate(walked):
        if point[1] == y:
            return idx
    raise ValueError(f"Route never reaches y={y}: {walked}")


def last_walk_index_at_y(walked: list[Point], y: int) -> int:
    for idx in range(len(walked) - 1, -1, -1):
        if walked[idx][1] == y:
            return idx
    raise ValueError(f"Route never reaches y={y}: {walked}")


def normalize_region_paths(
    astar_paths: PathData,
    region_orientation: str | None = None,
) -> PathData:
    region_paths: PathData = []
    for route in sort_routes(astar_paths):
        route = simplify_collinear(route)
        walked = walk_points(route)
        if region_orientation is None:
            if walked[0][0] != walked[-1][0]:
                body_orientation = "horizontal"
            elif walked[0][1] != walked[-1][1]:
                body_orientation = "vertical"
            else:
                raise ValueError(f"Route endpoints are identical after simplification: {route}")
        else:
            body_orientation = region_orientation

        if body_orientation == "horizontal":
            body_orientation = "horizontal"
            if walked[0][0] == walked[-1][0]:
                raise ValueError(f"Horizontal detour region needs different endpoint x values: {route}")
        elif body_orientation == "vertical":
            body_orientation = "vertical"
            if walked[0][1] == walked[-1][1]:
                raise ValueError(f"Vertical detour region needs different endpoint y values: {route}")
        else:
            raise ValueError(f"Unknown detour region orientation: {body_orientation}")
        region_paths.append(simplify_collinear(walked))
    return region_paths


def load_legacy_namespace(script_path: Path) -> dict[str, object]:
    detour_dir = script_path.parent
    namespace: dict[str, object] = {"__name__": "_legacy_contour_detour_lib_", "__file__": str(script_path)}
    source = script_path.read_text(encoding="utf-8", errors="ignore")
    old_cwd = Path.cwd()
    old_sys_path = list(sys.path)
    try:
        sys.path.insert(0, str(detour_dir))
        os.chdir(detour_dir)
        exec(compile(source, str(script_path), "exec"), namespace)
    finally:
        os.chdir(old_cwd)
        sys.path[:] = old_sys_path
    return namespace


def build_direct_boundary_context(
    source_paths: PathData,
    config: LegacyContourDetourConfig,
) -> DirectBoundaryContext:
    schedule_plan = longest_boundary_partition_plan(source_paths, config, "horizontal")
    residual_lengths_grid = config.residual_lengths_grid or derive_target_residual_lengths(
        config.expected_explicit_lengths_grid,
        config.target_total_length_grid,
        len(source_paths),
    )
    if len(source_paths) != len(residual_lengths_grid):
        raise ValueError(
            f"Expected {len(source_paths)} residual lengths, got {len(residual_lengths_grid)}"
        )

    namespace = load_legacy_namespace(config.script_path)
    external_obstacle_paths = config_external_obstacle_paths(config)
    ori_path = list(reversed(source_paths))
    res_len = list(reversed(residual_lengths_grid))
    namespace["ori_path"] = ori_path
    namespace["res_len"] = res_len

    expected_lengths_by_endpoint: dict[tuple[Point, Point], int] = {}
    expected_lengths_by_order: list[dict[str, object]] = []
    if config.expected_explicit_lengths_grid is not None:
        sorted_source = sort_routes(source_paths)
        for order, (route, explicit_length) in enumerate(zip(sorted_source, config.expected_explicit_lengths_grid)):
            start, end = endpoint_key(route)
            expected_lengths_by_endpoint[(start, end)] = explicit_length
            expected_lengths_by_order.append(
                {
                    "order": order,
                    "start": start,
                    "end": end,
                    "expected_explicit_length_grid": explicit_length,
                }
            )

    preferred_endpoint_by_endpoint: dict[tuple[Point, Point], Point] = {}
    if config.spiral_preferred_endpoints_grid is not None:
        sorted_source = sort_routes(source_paths)
        if len(config.spiral_preferred_endpoints_grid) != len(sorted_source):
            raise ValueError(
                f"Expected {len(sorted_source)} preferred spiral endpoints, "
                f"got {len(config.spiral_preferred_endpoints_grid)}"
            )
        for route, raw_preferred in zip(sorted_source, config.spiral_preferred_endpoints_grid):
            preferred = to_point(raw_preferred)
            if preferred != route[0] and preferred != route[-1]:
                raise ValueError(
                    f"Preferred spiral endpoint {preferred} is not an endpoint of route "
                    f"{route[0]}->{route[-1]}"
                )
            preferred_endpoint_by_endpoint[endpoint_key(route)] = preferred

    x_min = min(point[0] for route in ori_path for point in route)
    x_max = max(point[0] for route in ori_path for point in route)
    y_min = min(point[1] for route in ori_path for point in route)
    y_max = max(point[1] for route in ori_path for point in route)
    if config.boundary_area_grid is not None:
        boundary_area = list(config.boundary_area_grid)
        if not (boundary_area[0] <= x_min and x_max <= boundary_area[1] and boundary_area[2] <= y_min and y_max <= boundary_area[3]):
            raise ValueError(
                f"boundary_area_grid={boundary_area} does not contain source paths x={x_min}..{x_max}, y={y_min}..{y_max}"
            )
        boundary_source = "bbox"
    else:
        y_max = y_max + config.boundary_top_padding_grid
        boundary_area = [x_min - 1, x_max + 1, y_min - 1, y_max + 1]
        boundary_source = "explicit_top_padding"

    boundary = [
        (boundary_area[0], boundary_area[2]),
        (boundary_area[0], boundary_area[3]),
        (boundary_area[1], boundary_area[3]),
        (boundary_area[1], boundary_area[2]),
        (boundary_area[0], boundary_area[2]),
    ]
    if config.auto_region_search_area_grid is not None:
        routing_boundary_area = expand_search_area_to_contain_base(config.auto_region_search_area_grid, boundary_area)
        routing_boundary_source = "global_auto_region_search_area"
    else:
        routing_boundary_area = list(boundary_area)
        routing_boundary_source = boundary_source
    routing_boundary = [
        (routing_boundary_area[0], routing_boundary_area[2]),
        (routing_boundary_area[0], routing_boundary_area[3]),
        (routing_boundary_area[1], routing_boundary_area[3]),
        (routing_boundary_area[1], routing_boundary_area[2]),
        (routing_boundary_area[0], routing_boundary_area[2]),
    ]
    area_slack_check = detour_area_slack_check(
        boundary_area,
        source_paths,
        external_obstacle_paths,
        config,
    )
    if area_slack_check["enabled"] and not area_slack_check["passes"]:
        raise ValueError(f"Detour region area slack check failed: {area_slack_check}")

    target_length = config.target_total_length_grid
    if target_length <= 0:
        raise ValueError(f"Invalid target_total_length_grid={config.target_total_length_grid}")
    target_lengths_by_endpoint = {
        endpoint_key(route): config.target_total_length_grid
        for route in source_paths
    }
    return DirectBoundaryContext(
        config=config,
        source_paths=source_paths,
        residual_lengths_grid=residual_lengths_grid,
        namespace=namespace,
        external_obstacle_paths=external_obstacle_paths,
        ori_path=ori_path,
        res_len=res_len,
        expected_lengths_by_endpoint=expected_lengths_by_endpoint,
        expected_lengths_by_order=expected_lengths_by_order,
        preferred_endpoint_by_endpoint=preferred_endpoint_by_endpoint,
        boundary_area=boundary_area,
        boundary_source=boundary_source,
        boundary=boundary,
        routing_boundary_area=routing_boundary_area,
        routing_boundary_source=routing_boundary_source,
        routing_boundary=routing_boundary,
        hard_boundary_area=list(config.hard_boundary_area_grid) if config.hard_boundary_area_grid is not None else None,
        area_slack_check=area_slack_check,
        schedule_plan=schedule_plan,
        target_length=target_length,
        target_lengths_by_endpoint=target_lengths_by_endpoint,
    )


def next_ordered_side_region_step(fixed_count: int, path_count: int) -> OrderedSideRegionStep:
    if fixed_count >= path_count:
        raise ValueError(f"All paths are already fixed: fixed_count={fixed_count}, path_count={path_count}")
    pair_order = fixed_count // 2
    if fixed_count % 2 == 0:
        return OrderedSideRegionStep(
            active_side="lower",
            candidate_order=path_count - 1 - pair_order,
            diffuse_from_order=pair_order,
            diffuse_direction="up",
            spiral_direction="up",
            pair_order=pair_order,
            diffuse_snapshot_title="diffuse upper-side active paths",
            fixed_snapshot_title="fixed lower candidate after upward spiral",
        )
    return OrderedSideRegionStep(
        active_side="upper",
        candidate_order=pair_order,
        diffuse_from_order=path_count - 1 - (pair_order + 1),
        diffuse_direction="down",
        spiral_direction="down",
        pair_order=pair_order,
        diffuse_snapshot_title="diffuse lower-side active paths",
        fixed_snapshot_title="fixed upper candidate after downward spiral",
    )


def run_vertical_contour_detour(astar_paths: PathData, config: LegacyContourDetourConfig) -> DetourResult:
    if config.boundary_top_padding_grid is not None and config.boundary_area_grid is None:
        raise NotImplementedError("Vertical-body detour requires boundary_area_grid; top-padding-only mode is horizontal-specific.")
    transformed_config = replace(
        config,
        boundary_area_grid=swap_xy_boundary_area(config.boundary_area_grid),
        auto_region_search_area_grid=swap_xy_boundary_area(config.auto_region_search_area_grid),
        hard_boundary_area_grid=swap_xy_boundary_area(config.hard_boundary_area_grid),
        external_obstacle_paths_grid=swap_xy_paths(config.external_obstacle_paths_grid or []),
        external_block_points_grid=tuple(swap_xy_point(point) for point in config.external_block_points_grid or ()),
        spiral_preferred_endpoints_grid=(
            tuple(swap_xy_point(point) for point in config.spiral_preferred_endpoints_grid)
            if config.spiral_preferred_endpoints_grid is not None
            else None
        ),
        detour_direction_priority=(
            "fixed_original_swapped_xy"
            if config.detour_direction_priority == "fixed_original"
            else config.detour_direction_priority
        ),
    )
    transformed_result = run_legacy_contour_detour(swap_xy_paths(astar_paths), transformed_config)
    metadata = dict(transformed_result.metadata)
    metadata["coordinate_frame_transform"] = "swap_xy_for_vertical_body"
    metadata["original_body_orientation"] = "vertical"
    if config.boundary_area_grid is not None:
        metadata["boundary_area"] = list(config.boundary_area_grid)
    elif "boundary_area" in metadata and metadata["boundary_area"] is not None:
        metadata["boundary_area"] = swap_xy_boundary_area_list(metadata["boundary_area"])
    if "routing_boundary_area" in metadata and metadata["routing_boundary_area"] is not None:
        metadata["routing_boundary_area"] = swap_xy_boundary_area_list(metadata["routing_boundary_area"])
    if "hard_boundary_area" in metadata and metadata["hard_boundary_area"] is not None:
        metadata["hard_boundary_area"] = swap_xy_boundary_area_list(metadata["hard_boundary_area"])
    if "auto_detour_region" in metadata:
        metadata["auto_detour_region"] = swap_xy_auto_region_metadata(metadata["auto_detour_region"])
    if "longest_boundary_schedule" in metadata:
        metadata["longest_boundary_schedule"] = swap_xy_longest_boundary_schedule(metadata["longest_boundary_schedule"])
    return DetourResult(
        paths=swap_xy_paths(transformed_result.paths),
        log="vertical body transformed with swap_xy frame\n" + transformed_result.log,
        metadata=metadata,
        snapshots=swap_xy_snapshots(transformed_result.snapshots),
    )


def run_legacy_multiline_side_region_contour_detour(ctx: DirectBoundaryContext) -> DetourResult:
    config = ctx.config
    source_paths = ctx.source_paths
    namespace = ctx.namespace
    cal_path_length = namespace["cal_path_length"]
    initialize_grid = namespace["initialize_grid"]
    diffuse = namespace["diffuse"]
    external_obstacle_paths = ctx.external_obstacle_paths
    ori_path = ctx.ori_path
    res_len = ctx.res_len
    residual_lengths_grid = ctx.residual_lengths_grid
    expected_lengths_by_endpoint = ctx.expected_lengths_by_endpoint
    expected_lengths_by_order = ctx.expected_lengths_by_order
    boundary_area = ctx.boundary_area
    boundary_source = ctx.boundary_source
    routing_boundary_area = ctx.routing_boundary_area
    routing_boundary_source = ctx.routing_boundary_source
    hard_boundary_area = ctx.hard_boundary_area
    boundary = ctx.boundary
    area_slack_check = ctx.area_slack_check
    schedule_plan = ctx.schedule_plan
    target_length = ctx.target_length
    target_lengths_by_endpoint = ctx.target_lengths_by_endpoint
    x_min, x_max, y_min, y_max = boundary_area
    namespace["x_min"] = x_min
    namespace["x_max"] = x_max
    namespace["y_min"] = y_min
    namespace["y_max"] = y_max
    namespace["boundary"] = boundary
    namespace["boundary_area"] = boundary_area
    fixed_path: PathData = []
    paths: list[list[Point]] = []
    snapshots: list[dict[str, object]] = []

    def snapshot(title: str, fixed: list[list[Iterable[int]]] | None = None, candidates: list[list[Iterable[int]]] | None = None) -> None:
        snapshots.append(
            {
                "step": len(snapshots),
                "title": title,
                "fixed_paths": clone_paths(fixed or []),
                "candidate_paths": clone_paths(candidates or []),
                "source_paths": clone_paths(source_paths),
                "external_obstacle_paths": clone_paths(external_obstacle_paths),
            }
        )

    def grid_obstacles_for_boundary() -> PathData:
        return fixed_path + clip_obstacle_paths_to_boundary(external_obstacle_paths, boundary_area)

    def current_candidate_route(step: OrderedSideRegionStep) -> list[Point]:
        if fixed_path:
            return paths[0]
        return [[point[0], point[1]] for point in ori_path[step.candidate_order]]

    def candidate_length_context(detour_path: list[Iterable[int]], candidate_order: int) -> tuple[int, int, int, int | None]:
        route_key = endpoint_key(detour_path)
        route_target_length = target_lengths_by_endpoint.get(route_key, target_length)
        residual_len = res_len[candidate_order]
        cur_len = cal_path_length(detour_path) + residual_len
        expected_length = expected_lengths_by_endpoint.get(route_key)
        if expected_length is None:
            expected_length = route_target_length - residual_len
        length_dt = round((route_target_length - cur_len) / 2)
        return route_target_length, residual_len, length_dt, expected_length

    def preferred_endpoint_for_route(detour_path: list[Iterable[int]]) -> Point | None:
        return ctx.preferred_endpoint_by_endpoint.get(endpoint_key(detour_path))

    def direct_detour_precheck() -> tuple[PathData | None, dict[str, object]]:
        max_direct_gap_grid = 2

        def plan_with_boundary(
            detour_path: list[Point],
            candidate_order: int,
            route_target_length: int,
            residual_len: int,
            expected_length: int | None,
            step: OrderedSideRegionStep,
            plan_boundary_area: list[int],
        ) -> tuple[list[Point], dict[str, object]]:
            plan_boundary = [
                (plan_boundary_area[0], plan_boundary_area[2]),
                (plan_boundary_area[0], plan_boundary_area[3]),
                (plan_boundary_area[1], plan_boundary_area[3]),
                (plan_boundary_area[1], plan_boundary_area[2]),
                (plan_boundary_area[0], plan_boundary_area[2]),
            ]
            grid_obstacles = [
                route
                for index, route in enumerate(current)
                if index != candidate_order
            ] + clip_obstacle_paths_to_boundary(external_obstacle_paths, plan_boundary_area)
            grid = initialize_grid(plan_boundary, grid_obstacles, plan_boundary_area)
            return plan_block_aware_spiral(
                detour_path,
                grid_obstacles,
                grid,
                plan_boundary_area,
                hard_boundary_area,
                step.spiral_direction,
                config.detour_direction_priority,
                route_target_length,
                cal_path_length(detour_path) + residual_len,
                residual_len,
                expected_length,
                config.spiral_placement_policy,
                preferred_endpoint_for_route(detour_path),
            )

        current = [simplify_collinear([to_point(point) for point in route]) for route in ori_path]
        active_plan_boundary_area = boundary_with_outer_halo(routing_boundary_area)
        checks: list[dict[str, object]] = []
        for fixed_count in range(len(ori_path)):
            step = next_ordered_side_region_step(fixed_count, len(ori_path))
            candidate_order = step.candidate_order
            detour_path = current[candidate_order]
            route_target_length, residual_len, length_dt, expected_length = candidate_length_context(
                detour_path,
                candidate_order,
            )
            check: dict[str, object] = {
                "candidate_order": candidate_order,
                "active_side": step.active_side,
                "spiral_direction": step.spiral_direction,
                "route_length_grid": manhattan_length(detour_path),
                "target_length_grid": route_target_length,
                "expected_length_grid": expected_length,
                "length_dt": length_dt,
            }
            if expected_length is not None and manhattan_length(detour_path) >= expected_length:
                check["status"] = "no_direct_detour_needed"
                check["after_length_grid"] = manhattan_length(detour_path)
                check["target_gap_grid"] = max(0, expected_length - manhattan_length(detour_path))
                checks.append(check)
                continue

            try:
                detoured, plan = plan_with_boundary(
                    detour_path,
                    candidate_order,
                    route_target_length,
                    residual_len,
                    expected_length,
                    step,
                    active_plan_boundary_area,
                )
            except RuntimeError as exc:
                check["status"] = "directional_area_insufficient"
                check["error"] = compact_error(exc)
                check["plan_boundary_area"] = list(active_plan_boundary_area)
                checks.append(check)
                return None, {
                    "status": "requires_diffuse_before_detour",
                    "reason": "at least one selected path lacks direct detour capacity",
                    "failed_candidate_order": candidate_order,
                    "checks": checks,
                }

            check["status"] = "direct_detour_feasible"
            check["after_length_grid"] = manhattan_length(detoured)
            check["target_gap_grid"] = max(0, (expected_length or 0) - manhattan_length(detoured))
            check["plan_boundary_area"] = list(active_plan_boundary_area)
            check["plan"] = plan
            checks.append(check)
            if check["target_gap_grid"] > max_direct_gap_grid:
                check["status"] = "direct_detour_gap_too_large"
                check["max_direct_gap_grid"] = max_direct_gap_grid
                return None, {
                    "status": "requires_diffuse_before_detour",
                    "reason": "direct detour leaves a selected path too far below target length",
                    "failed_candidate_order": candidate_order,
                    "max_direct_gap_grid": max_direct_gap_grid,
                    "checks": checks,
                }
            current[candidate_order] = simplify_collinear(detoured)

        return current, {
            "status": "all_paths_direct_detour_feasible",
            "policy": "commit direct detours only after every selected path passes and stays within max direct length gap",
            "max_direct_gap_grid": max_direct_gap_grid,
            "checks": checks,
        }

    stdout = io.StringIO()
    time_start = time.time()
    direct_precheck_metadata: dict[str, object] = {"status": "not_run"}
    with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stdout):
        print("legacy contour detour adapter")
        print("source_paths", source_paths)
        print("residual_lengths_grid", list(residual_lengths_grid))
        print("target_total_length_grid", config.target_total_length_grid)
        print("target_length", target_length)
        print("legacy_target_vector", [cal_path_length(ori_path[i]) + res_len[i] for i in range(len(ori_path))])
        print("boundary_area", boundary_area)
        print("routing_boundary_area", routing_boundary_area)
        print("routing_boundary_source", routing_boundary_source)
        print("hard_boundary_area", hard_boundary_area)
        print("boundary", boundary)
        print("area_slack_check", area_slack_check)
        snapshot("00 region input", fixed=[], candidates=source_paths)

        direct_paths, direct_precheck_metadata = direct_detour_precheck()
        print("direct_detour_precheck", direct_precheck_metadata)
        if direct_paths is not None:
            fixed_path = direct_paths
            paths = direct_paths
            snapshot("01 direct detour without diffuse", fixed=fixed_path, candidates=paths)

        while direct_paths is None and len(fixed_path) < len(ori_path):
            step = next_ordered_side_region_step(len(fixed_path), len(ori_path))
            detour_path = current_candidate_route(step)
            if step.active_side == "upper":
                print("==============")
                print(detour_path)

            grid_obstacles = grid_obstacles_for_boundary()
            grid = initialize_grid(boundary, grid_obstacles, boundary_area)
            print(
                "side_region_step",
                {
                    "active_side": step.active_side,
                    "candidate_order": step.candidate_order,
                    "diffuse_from_order": step.diffuse_from_order,
                    "diffuse_direction": step.diffuse_direction,
                    "spiral_direction": step.spiral_direction,
                    "pair_order": step.pair_order,
                },
            )

            paths = diffuse(step.diffuse_from_order, grid, step.candidate_order, step.diffuse_direction)
            snapshot(f"{len(snapshots):02d} {step.diffuse_snapshot_title}", fixed=fixed_path, candidates=paths)
            if step.spiral_direction == "up":
                for route in fixed_path:
                    paths.append(route)
            else:
                paths.append(detour_path)
                paths.append(fixed_path[0])

            route_target_length, residual_len, length_dt, expected_length = candidate_length_context(
                detour_path,
                step.candidate_order,
            )
            cur_len = cal_path_length(detour_path) + residual_len
            if expected_length is not None:
                detour_path, plan = plan_block_aware_spiral(
                    detour_path,
                    grid_obstacles,
                    grid,
                    boundary_area,
                    hard_boundary_area,
                    step.spiral_direction,
                    config.detour_direction_priority,
                    route_target_length,
                    cur_len,
                    residual_len,
                    expected_length,
                    config.spiral_placement_policy,
                    preferred_endpoint_for_route(detour_path),
                )
                print("block-aware plan:", plan)
            elif length_dt > 0:
                detour_path, plan = plan_block_aware_spiral(
                    detour_path,
                    grid_obstacles,
                    grid,
                    boundary_area,
                    hard_boundary_area,
                    step.spiral_direction,
                    config.detour_direction_priority,
                    route_target_length,
                    cur_len,
                    residual_len,
                    None,
                    config.spiral_placement_policy,
                    preferred_endpoint_for_route(detour_path),
                )
                print("block-aware plan:", plan)
            else:
                print("block-aware plan:", {"status": "no_spiral_needed", "length_dt": length_dt, "current_length": cur_len})

            if step.spiral_direction == "down":
                print(fixed_path)
                for route in fixed_path:
                    paths.append(route)

            paths.append(detour_path)
            fixed_path.append(paths[-1])
            snapshot(f"{len(snapshots):02d} {step.fixed_snapshot_title}", fixed=fixed_path, candidates=[detour_path])
            if step.spiral_direction == "down":
                print(detour_path)

        print("Total Time:", time.time() - time_start)
        print(paths)
        for route in paths:
            print("number of bends:", len(route) - 2)

    result_paths = [simplify_collinear([to_point(point) for point in route]) for route in fixed_path]
    metadata = {
        "source": "side_region_contour_detour",
        "side_region_engine": "ordered_side_region_loop",
        "source_paths": source_paths,
        "residual_lengths_grid": list(residual_lengths_grid),
        "residual_lengths_source": "config" if config.residual_lengths_grid is not None else "target_length_delta",
        "target_total_length_grid": config.target_total_length_grid,
        "expected_lengths_by_order": expected_lengths_by_order,
        "boundary_top_padding_grid": config.boundary_top_padding_grid,
        "boundary_area_source": boundary_source,
        "routing_boundary_area_source": routing_boundary_source,
        "longest_boundary_schedule": schedule_plan,
        "spiral_placement_policy": config.spiral_placement_policy,
        "detour_direction_priority": config.detour_direction_priority,
        "detour_area_slack_check": area_slack_check,
        "direct_detour_precheck": direct_precheck_metadata,
        "boundary_area": boundary_area,
        "routing_boundary_area": routing_boundary_area,
        "hard_boundary_area": hard_boundary_area,
        "external_obstacle_paths_grid": external_obstacle_paths,
        "external_obstacle_count": len(external_obstacle_paths),
        "target_length": target_length,
        "side_region_active_count": len(source_paths),
        "target_lengths_by_order": [
            {
                "order": order,
                "start": start,
                "end": end,
                "target_length": target_lengths_by_endpoint[(start, end)],
            }
            for order, route in enumerate(sort_routes(source_paths))
            for start, end in [endpoint_key(route)]
        ],
    }
    return DetourResult(
        paths=result_paths,
        log=stdout.getvalue(),
        metadata=metadata,
        snapshots=snapshots,
    )


def run_direct_side_region_contour_detour(ctx: DirectBoundaryContext) -> DetourResult:
    return run_legacy_multiline_side_region_contour_detour(ctx)


def run_legacy_contour_detour(astar_paths: PathData, config: LegacyContourDetourConfig) -> DetourResult:
    if not config.script_path.exists():
        raise FileNotFoundError(config.script_path)

    region_orientation = infer_detour_region_orientation(astar_paths)
    source_paths = normalize_region_paths(astar_paths, region_orientation)
    if region_orientation == "vertical":
        return run_vertical_contour_detour(astar_paths, config)
    if region_orientation != "horizontal":
        raise NotImplementedError(f"Unsupported detour region orientation: {region_orientation}")

    if config.boundary_area_grid is None and config.boundary_top_padding_grid is None:
        return run_auto_region_detour(astar_paths, config, source_paths)

    ctx = build_direct_boundary_context(source_paths, config)
    if ctx.schedule_plan.get("partition_required"):
        result = run_partitioned_candidate_detour(astar_paths, config, source_paths, ctx.boundary_area, ctx.schedule_plan)
        result.metadata["boundary_area_source"] = ctx.boundary_source
        result.metadata["detour_area_slack_check"] = ctx.area_slack_check
        return result
    return run_direct_side_region_contour_detour(ctx)
