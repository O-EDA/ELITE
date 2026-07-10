from __future__ import annotations

import ast
import copy
import csv
import json
import math
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import yaml

from engine.bend_insertion import blocked_grid_points, insert_bends_to_target
from engine.detour_adapter import LegacyContourDetourConfig, run_legacy_contour_detour
from engine.native import lut_optimizer, route as native_route


ROOT = Path(__file__).resolve().parents[1]
ENGINE = ROOT / "engine"
DETOUR_KERNEL = ENGINE / "legacy_detour_kernel.py"
LUT_TABLE = ENGINE / "lut" / "optimizer_lut_60_60_100_20_final"

PITCH_UM = 10.0
WAVEGUIDE_LOSS_PER_UM = 1.5e-4
BEND_LOSS = 0.03
CROSS_LOSS = 0.52
LOSS_EPS = 1e-9
GENERAL_OVERFLOW_PENALTY = 99999.0
DEFAULT_BEND_LOSS_BY_ANGLE = {
    30: 0.01,
    45: 0.015,
    60: 0.02,
    90: 0.03,
}
ASTAR_DIRECTION_ANGLES = {
    "1": 180,
    "2": 120,
    "3": 90,
    "4": 60,
    "5": 0,
    "6": 300,
    "7": 270,
    "8": 240,
}

Coord = int | float
Point = tuple[Coord, Coord]
PathData = list[list[Point]]


@dataclass(frozen=True)
class SegmentEntry:
    group: str
    net: int
    original_route: list[Point]
    prefix: list[Point]
    segment: list[Point]
    suffix: list[Point]
    outside_length: int
    target_full_length: int
    target_region_length: int


@dataclass(frozen=True)
class CandidateSegment:
    net: int
    route: list[Point]
    index: int
    orientation: str
    sign: int
    fixed_coord: int
    lo: int
    hi: int
    length: int


@dataclass(frozen=True)
class RegionSplit:
    net: int
    route: list[Point]
    prefix: list[Point]
    segment: list[Point]
    suffix: list[Point]
    orientation: str
    sign: int
    entry_axis: int
    exit_axis: int
    entry_orth: int
    exit_orth: int


@dataclass(frozen=True)
class RegionCandidate:
    group: str
    orientation: str
    sign: int
    splits: list[RegionSplit]
    lo: int
    hi: int
    area: int
    span: int
    overlap: int


def mkdir(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path


def write_text(path: Path, text: str) -> None:
    mkdir(path.parent)
    path.write_text(text, encoding="utf-8")


def clean_number(value: float | int) -> float | int:
    if isinstance(value, float) and value.is_integer():
        return int(value)
    return value


def pitch_um(case: dict[str, object]) -> float:
    return float(case.get("pitch_um", PITCH_UM))


def loss_config(case: dict[str, object]) -> dict[str, object]:
    return case.get("loss", {}) or {}


def propagation_loss_per_um(case: dict[str, object]) -> float:
    return float(loss_config(case).get("propagation_loss_per_um", WAVEGUIDE_LOSS_PER_UM))


def astar_path_loss_per_step(case: dict[str, object]) -> float:
    return propagation_loss_per_um(case) * pitch_um(case)


def min_bend_radius_grid(case: dict[str, object], section_cfg: dict[str, object] | None = None, default_grid: float = 0.0) -> float:
    section_cfg = section_cfg or {}
    cfg = loss_config(case)
    if "min_bend_radius_um" in cfg:
        return float(cfg["min_bend_radius_um"]) / pitch_um(case)
    if "min_bend_radius_um" in section_cfg:
        return float(section_cfg["min_bend_radius_um"]) / pitch_um(case)
    return default_grid


def crossing_loss_value(case: dict[str, object]) -> float:
    return float(loss_config(case).get("crossing_loss", CROSS_LOSS))


def bend_loss_by_angle(case: dict[str, object]) -> dict[int, float]:
    raw = loss_config(case).get("bend_loss_by_angle", {}) or {}
    losses = dict(DEFAULT_BEND_LOSS_BY_ANGLE)
    for angle, value in dict(raw).items():
        losses[int(angle)] = float(value)
    if "bend_loss_90" in loss_config(case):
        losses[90] = float(loss_config(case)["bend_loss_90"])
    return losses


def bend_loss_for_angle(case: dict[str, object], angle: int) -> float:
    if angle <= 0:
        return 0.0
    losses = bend_loss_by_angle(case)
    return float(losses.get(angle, losses.get(90, BEND_LOSS)))


def astar_loss_config(case: dict[str, object]) -> dict[str, float]:
    losses = bend_loss_by_angle(case)
    return {
        "path_loss": astar_path_loss_per_step(case),
        "crossing_loss": crossing_loss_value(case),
        **{f"bend_loss_{angle}": float(losses.get(angle, 0.0)) for angle in (30, 45, 60, 90)},
    }


def lut_max_level(case: dict[str, object]) -> int:
    return int((case.get("lut", {}) or {}).get("max_level", 10))


def to_coord(value: object) -> Coord:
    number = float(value)
    return int(number) if number.is_integer() else number


def to_point(point: Iterable[object]) -> Point:
    x, y = point
    return to_coord(x), to_coord(y)


def remove_duplicate_points(route: Sequence[Point]) -> list[Point]:
    clean: list[Point] = []
    for point in route:
        if not clean or clean[-1] != point:
            clean.append(point)
    return clean


def simplify_collinear(route: Sequence[Point]) -> list[Point]:
    route = remove_duplicate_points(route)
    if len(route) <= 2:
        return list(route)
    simplified = [route[0]]
    for idx in range(1, len(route) - 1):
        prev = simplified[-1]
        cur = route[idx]
        nxt = route[idx + 1]
        v1 = (cur[0] - prev[0], cur[1] - prev[1])
        v2 = (nxt[0] - cur[0], nxt[1] - cur[1])
        if (prev[0] == cur[0] == nxt[0]) or (prev[1] == cur[1] == nxt[1]):
            continue
        if (
            v1[0]
            and v1[1]
            and v2[0]
            and v2[1]
            and v1[0] * v2[1] == v1[1] * v2[0]
            and v1[0] * v2[0] + v1[1] * v2[1] >= 0
        ):
            continue
        simplified.append(cur)
    simplified.append(route[-1])
    return simplified


def orient_left_to_right(route: Sequence[Point]) -> list[Point]:
    route = list(route)
    if route[0][0] > route[-1][0]:
        return list(reversed(route))
    return route


def canonical_route(route: Sequence[Point]) -> list[Point]:
    return simplify_collinear(route)


def sort_routes(paths: Sequence[Sequence[Point]]) -> PathData:
    return sorted((canonical_route(route) for route in paths), key=lambda p: (p[0][1], p[0][0], p[-1][1], p[-1][0]))


def manhattan_length(route: Sequence[Point]) -> float | int:
    route = simplify_collinear(route)
    return clean_number(sum(abs(a[0] - b[0]) + abs(a[1] - b[1]) for a, b in zip(route, route[1:])))


def sign(value: int) -> int:
    if value > 0:
        return 1
    if value < 0:
        return -1
    return 0


def walk_points(route: Sequence[Point]) -> list[Point]:
    route = simplify_collinear(route)
    if not route:
        return []
    points = [route[0]]
    for a, b in zip(route, route[1:]):
        dx_raw = b[0] - a[0]
        dy_raw = b[1] - a[1]
        if int(dx_raw) != dx_raw or int(dy_raw) != dy_raw:
            raise ValueError(f"Non-integer grid segment in route {route}: {a}->{b}")
        if dx_raw and dy_raw and abs(dx_raw) != abs(dy_raw):
            raise ValueError(f"Non-45-degree segment in route {route}: {a}->{b}")
        dx = sign(int(dx_raw))
        dy = sign(int(dy_raw))
        steps = max(abs(int(dx_raw)), abs(int(dy_raw)))
        cur = a
        for _ in range(steps):
            cur = (cur[0] + dx, cur[1] + dy)
            points.append(cur)
    return points


def unit_segments(route: Sequence[Point]) -> list[tuple[Point, Point]]:
    points = walk_points(route)
    return [(a, b) for a, b in zip(points, points[1:]) if a != b]


def straight_segments(route: Sequence[Point]) -> list[tuple[Point, Point]]:
    route = simplify_collinear(route)
    segments: list[tuple[Point, Point]] = []
    for a, b in zip(route, route[1:]):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        if dx and dy and abs(dx) != abs(dy):
            raise ValueError(f"Unsupported segment direction: {a}->{b}")
        if a != b:
            segments.append((a, b))
    return segments


def segment_kind(a: Point, b: Point) -> str:
    dx = float(b[0] - a[0])
    dy = float(b[1] - a[1])
    if dy == 0:
        return "horizontal"
    if dx == 0:
        return "vertical"
    if abs(dx) == abs(dy):
        return "diag_pos" if dx == dy else "diag_neg"
    raise ValueError(f"Unsupported segment direction: {a}->{b}")


def astar_direction_code(a: Point, b: Point) -> str:
    dx = sign(float(b[0] - a[0]))
    dy = sign(float(b[1] - a[1]))
    mapping = {
        (-1, 0): "1",
        (-1, 1): "2",
        (0, 1): "3",
        (1, 1): "4",
        (1, 0): "5",
        (1, -1): "6",
        (0, -1): "7",
        (-1, -1): "8",
    }
    key = (dx, dy)
    if key not in mapping:
        raise ValueError(f"Unsupported segment direction: {a}->{b}")
    return mapping[key]


def astar_bend_angle(prev: Point, cur: Point, nxt: Point) -> int:
    in_code = astar_direction_code(cur, nxt)
    out_code = astar_direction_code(cur, prev)
    input_angle = ASTAR_DIRECTION_ANGLES[in_code]
    output_angle = ASTAR_DIRECTION_ANGLES[out_code]
    bend_angle = abs(output_angle - input_angle) % 360
    if bend_angle > 180:
        bend_angle = 360 - bend_angle
    return int(180 - bend_angle)


def bend_loss_at(prev: Point, cur: Point, nxt: Point, case: dict[str, object]) -> float:
    return bend_loss_for_angle(case, astar_bend_angle(prev, cur, nxt))


def route_bend_loss(route: Sequence[Point], case: dict[str, object]) -> float:
    simplified = simplify_collinear(route)
    total = 0.0
    for prev, cur, nxt in zip(simplified, simplified[1:], simplified[2:]):
        total += bend_loss_at(prev, cur, nxt, case)
    return total


def legal_crossing_type(left_kind: str, right_kind: str) -> str | None:
    kinds = {left_kind, right_kind}
    if kinds == {"horizontal", "vertical"}:
        return "orthogonal_90"
    if kinds == {"diag_pos", "diag_neg"}:
        return "diagonal_45"
    return None


def segment_intersection_key(a: Point, b: Point, c: Point, d: Point) -> tuple[object, ...] | None:
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])
    cx, cy = float(c[0]), float(c[1])
    dx, dy = float(d[0]), float(d[1])
    r = (bx - ax, by - ay)
    s = (dx - cx, dy - cy)
    denom = r[0] * s[1] - r[1] * s[0]
    qmp = (cx - ax, cy - ay)
    if abs(denom) <= LOSS_EPS:
        if abs(qmp[0] * r[1] - qmp[1] * r[0]) > LOSS_EPS:
            return None
        a_key = tuple(sorted((a, b)))
        c_key = tuple(sorted((c, d)))
        if a_key == c_key:
            return ("overlap", a_key)
        shared = set(a_key) & set(c_key)
        if shared:
            point = next(iter(shared))
            return (
                "illegal_touch",
                segment_kind(a, b),
                segment_kind(c, d),
                clean_number(float(point[0])),
                clean_number(float(point[1])),
            )
        return None
    t = (qmp[0] * s[1] - qmp[1] * s[0]) / denom
    u = (qmp[0] * r[1] - qmp[1] * r[0]) / denom
    if -LOSS_EPS <= t <= 1.0 + LOSS_EPS and -LOSS_EPS <= u <= 1.0 + LOSS_EPS:
        x = ax + t * r[0]
        y = ay + t * r[1]
        left_kind = segment_kind(a, b)
        right_kind = segment_kind(c, d)
        crossing_type = legal_crossing_type(left_kind, right_kind)
        if crossing_type is None:
            return (
                "illegal_crossing",
                left_kind,
                right_kind,
                clean_number(round(x, 9)),
                clean_number(round(y, 9)),
            )
        return ("legal_crossing", crossing_type, clean_number(round(x, 9)), clean_number(round(y, 9)))
    return None


def unit_edges(route: Sequence[Point]) -> list[tuple[Point, Point]]:
    points = walk_points(route)
    return [tuple(sorted((a, b))) for a, b in zip(points, points[1:])]


def axis_value(point: Point, orientation: str) -> Coord:
    return point[0] if orientation == "horizontal" else point[1]


def orthogonal_value(point: Point, orientation: str) -> Coord:
    return point[1] if orientation == "horizontal" else point[0]


def bend_count(route: Sequence[Point]) -> int:
    route = simplify_collinear(route)
    bends = 0
    for a, b, c in zip(route, route[1:], route[2:]):
        v1 = (b[0] - a[0], b[1] - a[1])
        v2 = (c[0] - b[0], c[1] - b[1])
        if v1[0] * v2[1] - v1[1] * v2[0] != 0:
            bends += 1
    return bends


def trim_route_start(route: Sequence[Point], distance: int) -> list[Point]:
    points = simplify_collinear(route)
    if distance <= 0 or len(points) < 2:
        return list(points)
    first, second = points[0], points[1]
    seg_len = abs(second[0] - first[0]) + abs(second[1] - first[1])
    if seg_len <= 0 or distance >= manhattan_length(points):
        return list(points)
    step = (sign(second[0] - first[0]), sign(second[1] - first[1]))
    if distance < seg_len:
        points[0] = (first[0] + step[0] * distance, first[1] + step[1] * distance)
    else:
        points = points[1:]
    return simplify_collinear(points)


def trim_route_end(route: Sequence[Point], distance: int) -> list[Point]:
    points = simplify_collinear(route)
    if distance <= 0 or len(points) < 2:
        return list(points)
    prev, last = points[-2], points[-1]
    seg_len = abs(last[0] - prev[0]) + abs(last[1] - prev[1])
    if seg_len <= 0 or distance >= manhattan_length(points):
        return list(points)
    step = (sign(prev[0] - last[0]), sign(prev[1] - last[1]))
    if distance < seg_len:
        points[-1] = (last[0] + step[0] * distance, last[1] + step[1] * distance)
    else:
        points = points[:-1]
    return simplify_collinear(points)


def measurement_route(route: Sequence[Point], case: dict[str, object]) -> list[Point]:
    metric_cfg = case.get("metric", {}) or {}
    access_grid = int(metric_cfg.get("exclude_terminal_access_grid", 0))
    if access_grid <= 0:
        return simplify_collinear(route)
    side = str(metric_cfg.get("terminal_access_side", "auto"))
    if side == "source":
        return trim_route_start(route, access_grid)
    if side == "target":
        return trim_route_end(route, access_grid)
    if side != "auto":
        raise ValueError(f"Unknown metric.terminal_access_side: {side}")
    candidates = [
        ("source", trim_route_start(route, access_grid)),
        ("target", trim_route_end(route, access_grid)),
    ]
    return min(candidates, key=lambda item: (bend_count(item[1]), manhattan_length(item[1]), item[0]))[1]


def measurement_length(route: Sequence[Point], case: dict[str, object]) -> int:
    return manhattan_length(measurement_route(route, case))


def metric_external_lengths(case: dict[str, object], count: int) -> list[int]:
    metric_cfg = case.get("metric", {}) or {}
    return scalar_or_list(metric_cfg.get("external_length_grid", 0), count, "metric.external_length_grid")


def measurement_bend_count(route: Sequence[Point], case: dict[str, object]) -> int:
    return bend_count(route)


def endpoint_key(route: Sequence[Point]) -> tuple[Point, Point]:
    route = simplify_collinear(route)
    if route[0] <= route[-1]:
        return route[0], route[-1]
    return route[-1], route[0]


def write_case_literal(path: Path, paths: PathData) -> None:
    payload = [[[(x, y) for x, y in route] for route in paths]]
    write_text(path, repr(payload) + "\n")


def load_case_literal(path: Path) -> PathData:
    value = ast.literal_eval(path.read_text(encoding="utf-8"))
    while isinstance(value, list) and len(value) == 1 and isinstance(value[0], list) and value[0] and isinstance(value[0][0], list):
        value = value[0]
    return [[to_point(point) for point in route] for route in value]


def orient_paths_to_case_endpoints(paths: PathData, case: dict[str, object]) -> PathData:
    remaining = [simplify_collinear(route) for route in paths]
    oriented: PathData = []
    for net in case["endpoints"]:
        source = to_point(net["source"])
        target = to_point(net["target"])
        match_index = None
        reverse = False
        for idx, route in enumerate(remaining):
            if route[0] == source and route[-1] == target:
                match_index = idx
                reverse = False
                break
            if route[0] == target and route[-1] == source:
                match_index = idx
                reverse = True
                break
        if match_index is None:
            raise ValueError(f"A* did not return a path for endpoint pair {source}->{target}: {remaining}")
        route = remaining.pop(match_index)
        oriented.append(list(reversed(route)) if reverse else route)
    return oriented


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    mkdir(path.parent)
    fieldnames = list(rows[0].keys()) if rows else []
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def scalar_or_list(value: object, count: int, name: str) -> list[int]:
    if isinstance(value, list):
        if len(value) != count:
            raise ValueError(f"{name} needs {count} entries, got {len(value)}")
        return [int(item) for item in value]
    return [int(value)] * count


def obstacle_rectangles(case: dict[str, object]) -> list[tuple[int, int, int, int]]:
    rectangles: list[tuple[int, int, int, int]] = []
    for raw in case.get("obstacles", []) or []:
        if not isinstance(raw, list) or len(raw) != 4:
            raise ValueError("Each obstacle must be a rectangle [xmin, xmax, ymin, ymax].")
        x0, x1, y0, y1 = (int(value) for value in raw)
        if x0 > x1 or y0 > y1:
            raise ValueError(f"Invalid obstacle rectangle: {raw}")
        rectangles.append((x0, x1, y0, y1))
    return rectangles


def obstacle_points(case: dict[str, object]) -> list[Point]:
    points: list[Point] = []
    for x0, x1, y0, y1 in obstacle_rectangles(case):
        for x in range(x0, x1 + 1):
            for y in range(y0, y1 + 1):
                points.append((x, y))
    return points


def add_scaled(point: Point, direction: Point, distance: float) -> Point:
    return clean_number(point[0] + direction[0] * distance), clean_number(point[1] + direction[1] * distance)


def unit_direction(a: Point, b: Point) -> Point:
    dx, dy = b[0] - a[0], b[1] - a[1]
    if dx and dy:
        raise ValueError(f"Non-manhattan segment: {a} -> {b}")
    if dx == 0 and dy == 0:
        raise ValueError(f"Zero-length segment: {a} -> {b}")
    return sign(dx), sign(dy)


def dot(a: Point, b: Point) -> float | int:
    return clean_number(a[0] * b[0] + a[1] * b[1])


def neg(point: Point) -> Point:
    return -point[0], -point[1]


def axis_segments(route: Sequence[Point]) -> list[tuple[Point, Point]]:
    route = simplify_collinear(route)
    segments: list[tuple[Point, Point]] = []
    for a, b in zip(route, route[1:]):
        if a[0] != b[0] and a[1] != b[1]:
            raise ValueError(f"Non-manhattan segment: {a} -> {b}")
        if a != b:
            segments.append((a, b))
    return segments


def intervals_overlap(a0: Coord, a1: Coord, b0: Coord, b1: Coord) -> bool:
    alo, ahi = sorted((a0, a1))
    blo, bhi = sorted((b0, b1))
    return max(alo, blo) <= min(ahi, bhi) + LOSS_EPS


def axis_segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool:
    a_vertical = a[0] == b[0]
    c_vertical = c[0] == d[0]
    if a_vertical and c_vertical:
        return a[0] == c[0] and intervals_overlap(a[1], b[1], c[1], d[1])
    if not a_vertical and not c_vertical:
        return a[1] == c[1] and intervals_overlap(a[0], b[0], c[0], d[0])
    vertical_a, vertical_b, horizontal_a, horizontal_b = (a, b, c, d) if a_vertical else (c, d, a, b)
    vx = vertical_a[0]
    hy = horizontal_a[1]
    return intervals_overlap(vx, vx, horizontal_a[0], horizontal_b[0]) and intervals_overlap(
        hy, hy, vertical_a[1], vertical_b[1]
    )


def crossing_counts(paths: Sequence[Sequence[Point]]) -> list[int]:
    routes = [simplify_collinear(route) for route in paths]
    counts = [0 for _ in routes]
    seen: set[tuple[object, ...]] = set()
    for left_idx, left_route in enumerate(routes):
        for right_idx in range(left_idx + 1, len(routes)):
            right_route = routes[right_idx]
            for a, b in straight_segments(left_route):
                for c, d in straight_segments(right_route):
                    hit = segment_intersection_key(a, b, c, d)
                    if hit is None or hit[0] != "legal_crossing":
                        continue
                    key = (left_idx, right_idx, hit)
                    if key in seen:
                        continue
                    seen.add(key)
                    counts[left_idx] += 1
                    counts[right_idx] += 1
    return counts


def route_self_intersects(route: Sequence[Point]) -> bool:
    segments = axis_segments(route)
    for idx, (a, b) in enumerate(segments):
        for other_idx in range(idx + 1, len(segments)):
            if other_idx == idx + 1:
                continue
            c, d = segments[other_idx]
            if axis_segments_intersect(a, b, c, d):
                return True
    return False


def route_intersects_routes(route: Sequence[Point], others: Sequence[Sequence[Point]]) -> bool:
    for a, b in axis_segments(route):
        for other in others:
            for c, d in axis_segments(other):
                if axis_segments_intersect(a, b, c, d):
                    return True
    return False


def route_intersects_grid_routes(route: Sequence[Point], others: Sequence[Sequence[Point]]) -> bool:
    for a, b in unit_segments(route):
        for other in others:
            for c, d in unit_segments(other):
                hit = segment_intersection_key(a, b, c, d)
                if hit is not None:
                    return True
    return False


def segment_intersects_rect(a: Point, b: Point, rect: tuple[int, int, int, int]) -> bool:
    x0, x1, y0, y1 = rect
    if a[0] == b[0]:
        return x0 - LOSS_EPS <= a[0] <= x1 + LOSS_EPS and intervals_overlap(a[1], b[1], y0, y1)
    if a[1] == b[1]:
        return y0 - LOSS_EPS <= a[1] <= y1 + LOSS_EPS and intervals_overlap(a[0], b[0], x0, x1)
    raise ValueError(f"Non-manhattan segment: {a} -> {b}")


def route_intersects_obstacles(route: Sequence[Point], rectangles: Sequence[tuple[int, int, int, int]]) -> bool:
    for a, b in axis_segments(route):
        if any(segment_intersects_rect(a, b, rect) for rect in rectangles):
            return True
    return False


def clipped_shifted_obstacle_points(case: dict[str, object], origin: Point) -> list[Point]:
    bbox = [int(v) for v in case["bbox"]]
    bx0, bx1, by0, by1 = bbox
    ox, oy = origin
    points: list[Point] = []
    for x0, x1, y0, y1 in obstacle_rectangles(case):
        cx0, cx1 = max(x0, bx0), min(x1, bx1)
        cy0, cy1 = max(y0, by0), min(y1, by1)
        if cx0 > cx1 or cy0 > cy1:
            continue
        for x in range(cx0, cx1 + 1):
            for y in range(cy0, cy1 + 1):
                points.append((x - ox, y - oy))
    return points


def load_case(path: Path) -> dict[str, object]:
    case = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(case, dict):
        raise ValueError(f"Invalid YAML case file: {path}")
    has_group_endpoints = any(
        isinstance(group, dict) and "endpoints" in group
        for key in ("matching_groups", "general_groups")
        for group in (case.get(key) or [])
    )
    if has_group_endpoints:
        if case.get("endpoints") is not None:
            raise ValueError("Use either top-level endpoints or group endpoints, not both")
        endpoints: list[dict[str, object]] = []
        matching_groups = []
        for raw_group in case.get("matching_groups") or []:
            group = dict(raw_group)
            group_endpoints = group.pop("endpoints", None)
            if not isinstance(group_endpoints, list) or not group_endpoints:
                raise ValueError("matching_groups entries with group endpoints must contain a non-empty endpoints list")
            start = len(endpoints)
            endpoints.extend(copy.deepcopy(group_endpoints))
            group["nets"] = list(range(start, len(endpoints)))
            matching_groups.append(group)
        general_groups = []
        general_nets = []
        for raw_group in case.get("general_groups") or []:
            group = dict(raw_group)
            group_endpoints = group.pop("endpoints", None)
            if not isinstance(group_endpoints, list) or not group_endpoints:
                raise ValueError("general_groups entries must contain a non-empty endpoints list")
            start = len(endpoints)
            endpoints.extend(copy.deepcopy(group_endpoints))
            group["nets"] = list(range(start, len(endpoints)))
            general_nets.extend(group["nets"])
            general_groups.append(group)
        case["endpoints"] = endpoints
        case["matching_groups"] = matching_groups
        case["general_groups"] = general_groups
        case["general_nets"] = general_nets
    endpoints = case.get("endpoints")
    if not isinstance(endpoints, list) or not endpoints:
        raise ValueError("path.yml must contain a non-empty endpoints list")
    general = [int(net) for net in case.get("general_nets", [])]
    if len(general) != len(set(general)):
        raise ValueError(f"general_nets contains duplicate nets: {general}")
    if any(idx < 0 or idx >= len(endpoints) for idx in general):
        raise ValueError(f"general_nets contains out-of-range nets: {general}")
    groups = case.get("matching_groups")
    if groups is None:
        groups = [{"name": "group1"}]
        case["matching_groups"] = groups
    matching: list[int] = []
    implicit_groups = [group for group in groups if "nets" not in group]
    if len(implicit_groups) > 1:
        raise ValueError("Only one matching group may omit nets")
    for group in groups:
        nets = group.get("nets")
        if nets is None:
            nets = [idx for idx in range(len(endpoints)) if idx not in set(general)]
            group["nets"] = nets
        for net in nets:
            idx = int(net)
            if idx < 0 or idx >= len(endpoints):
                raise ValueError(f"matching_groups contains out-of-range net {idx}")
            matching.append(idx)
    if len(matching) != len(set(matching)):
        raise ValueError(f"matching_groups contains duplicate nets: {matching}")
    overlap = sorted(set(matching) & set(general))
    if overlap:
        raise ValueError(f"Nets cannot be both matching and general: {overlap}")
    return case


def matching_net_indices(case: dict[str, object]) -> list[int]:
    groups = case.get("matching_groups") or []
    general = set(general_net_indices_without_matching(case))
    matching: set[int] = set()
    for group in groups:
        nets = group.get("nets")
        if nets is None:
            matching.update(idx for idx in range(len(case["endpoints"])) if idx not in general)
        else:
            matching.update(int(net) for net in nets)
    return sorted(matching)


def general_net_indices_without_matching(case: dict[str, object]) -> list[int]:
    endpoints = case["endpoints"]
    explicit = case.get("general_nets")
    if explicit is not None:
        return [int(net) for net in explicit]
    return [] if case.get("matching_groups") else list(range(len(endpoints)))


def general_net_indices(case: dict[str, object]) -> list[int]:
    endpoints = case["endpoints"]
    explicit = case.get("general_nets")
    if explicit is not None:
        return [int(net) for net in explicit]
    matching = set(matching_net_indices(case))
    return [idx for idx in range(len(endpoints)) if idx not in matching]


def subset_case(case: dict[str, object], nets: Sequence[int], matching_only: bool = True) -> dict[str, object]:
    net_list = list(nets)
    remap = {old: new for new, old in enumerate(net_list)}
    sub = copy.deepcopy(case)
    sub["endpoints"] = [copy.deepcopy(case["endpoints"][idx]) for idx in net_list]
    if "target_length" in case and isinstance(case["target_length"], list):
        sub["target_length"] = [case["target_length"][idx] for idx in net_list]
    metric_cfg = sub.get("metric")
    if isinstance(metric_cfg, dict) and isinstance(metric_cfg.get("external_length_grid"), list):
        metric_cfg["external_length_grid"] = [metric_cfg["external_length_grid"][idx] for idx in net_list]

    if matching_only:
        groups = []
        for group in case.get("matching_groups") or []:
            mapped = [remap[int(net)] for net in group.get("nets", []) if int(net) in remap]
            if mapped:
                groups.append({"name": group.get("name", f"group{len(groups) + 1}"), "nets": mapped})
        sub["matching_groups"] = groups or [{"name": "group1", "nets": list(range(len(net_list)))}]
        sub.pop("general_nets", None)
    else:
        sub["matching_groups"] = []
        sub["general_nets"] = list(range(len(net_list)))
    return sub


def astar_pin_direction(net: dict[str, object], key: str) -> int:
    if key not in {"source_direction", "target_direction"}:
        raise ValueError(f"Unknown A* pin direction key: {key}")
    if key not in net:
        raise ValueError(f"Each endpoint must explicitly set {key}: {net}")
    value = int(net[key])
    if value < -1 or value > 3:
        raise ValueError(f"{key} must be in [-1, 3], got {value}")
    return value


def shift_point(point: Point, origin: Point) -> Point:
    return point[0] - origin[0], point[1] - origin[1]


def unshift_paths(paths: PathData, origin: Point) -> PathData:
    ox, oy = origin
    return [[(x + ox, y + oy) for x, y in route] for route in paths]


def astar_memory_inputs(case: dict[str, object]) -> tuple[list[dict[str, object]], list[Point], Point]:
    bbox = [int(v) for v in case["bbox"]]
    origin = (bbox[0], bbox[2])
    nets: list[dict[str, object]] = []
    for index, net in enumerate(case["endpoints"]):
        source = shift_point(to_point(net["source"]), origin)
        target = shift_point(to_point(net["target"]), origin)
        if source[0] < 0 or source[1] < 0 or target[0] < 0 or target[1] < 0:
            raise ValueError(f"Endpoint is outside bbox after shifting by {origin}: {net}")
        nets.append(
            {
                "name": str(index),
                "source": source,
                "target": target,
                "source_direction": astar_pin_direction(net, "source_direction"),
                "target_direction": astar_pin_direction(net, "target_direction"),
            }
        )
    return nets, clipped_shifted_obstacle_points(case, origin), origin


def route_order_indices(case: dict[str, object], policy: str) -> list[int]:
    endpoints = case["endpoints"]
    indices = list(range(len(endpoints)))
    if policy in {"", "input"}:
        return indices
    if policy != "hpwl_short_first":
        raise ValueError(f"Unknown order policy: {policy}")

    def hpwl(idx: int) -> int:
        net = endpoints[idx]
        source = to_point(net["source"])
        target = to_point(net["target"])
        return int(abs(source[0] - target[0]) + abs(source[1] - target[1]))

    return sorted(indices, key=hpwl)


def reorder_case_endpoints_for_astar(case: dict[str, object], order: Sequence[int]) -> dict[str, object]:
    routed_case = copy.deepcopy(case)
    routed_case["endpoints"] = [case["endpoints"][idx] for idx in order]
    routed_case["matching_groups"] = [{"name": "astar_order", "nets": list(range(len(order)))}]
    routed_case.pop("general_nets", None)
    astar_cfg = dict(routed_case.get("astar", {}) or {})
    astar_cfg["order"] = "input"
    routed_case["astar"] = astar_cfg
    return routed_case


def run_astar(out_dir: Path, case: dict[str, object]) -> PathData:
    stage = mkdir(out_dir / "01_astar")
    pitch_um = float(case.get("pitch_um", PITCH_UM))
    astar_cfg = case.get("astar", {}) or {}
    order_policy = str(astar_cfg.get("order_policy", ""))
    order_indices = route_order_indices(case, order_policy)
    routed_case = reorder_case_endpoints_for_astar(case, order_indices) if order_policy else case
    nets, obstacles, origin = astar_memory_inputs(routed_case)
    bbox = [int(v) for v in case["bbox"]]
    config: dict[str, object] = {
        "pitch": pitch_um,
        "order": str((routed_case.get("astar", {}) or {}).get("order", "input")),
        "direction": int(astar_cfg.get("direction", 1)),
        "block_pins": True,
        "rudy_weight": float(astar_cfg.get("rudy_weight", 0.0)),
        "block_routed_paths": bool(astar_cfg.get("block_routed_paths", False)),
        "reserve_pin_stubs": True,
        "min_bend_radius_grid": min_bend_radius_grid(case, astar_cfg, 0.0),
        **astar_loss_config(case),
    }
    result = native_route(
        nets,
        bbox[1] - bbox[0],
        bbox[3] - bbox[2],
        obstacles,
        config,
    )
    native_paths = [[to_point(point) for point in route] for route in result["routes"]]
    raw_paths = unshift_paths(native_paths, origin)
    routed_paths = orient_paths_to_case_endpoints(raw_paths, routed_case)
    if order_policy:
        by_original_index = {old_idx: route for old_idx, route in zip(order_indices, routed_paths)}
        paths = [by_original_index[idx] for idx in range(len(case["endpoints"]))]
        write_text(
            stage / "astar_order.json",
            json.dumps({"policy": order_policy, "order": order_indices}, indent=2) + "\n",
        )
    else:
        paths = routed_paths
    write_case_literal(stage / "astar_paths.txt", paths)
    return paths


def route_segments(net: int, route: list[Point]) -> list[CandidateSegment]:
    route = simplify_collinear(route)
    segments: list[CandidateSegment] = []
    for idx, (a, b) in enumerate(zip(route, route[1:])):
        if a[1] == b[1]:
            lo, hi = sorted((a[0], b[0]))
            sign = 1 if b[0] > a[0] else -1
            segments.append(CandidateSegment(net, route, idx, "horizontal", sign, a[1], lo, hi, hi - lo))
        elif a[0] == b[0]:
            lo, hi = sorted((a[1], b[1]))
            sign = 1 if b[1] > a[1] else -1
            segments.append(CandidateSegment(net, route, idx, "vertical", sign, a[0], lo, hi, hi - lo))
        else:
            raise ValueError(f"Non-manhattan segment in route {route}: {a}->{b}")
    return segments


def segments_by_key(paths: PathData) -> dict[tuple[str, int], dict[int, list[CandidateSegment]]]:
    by_key: dict[tuple[str, int], dict[int, list[CandidateSegment]]] = {}
    for net, route in enumerate(sort_routes(paths)):
        for seg in route_segments(net, route):
            key = (seg.orientation, seg.sign)
            by_key.setdefault(key, {}).setdefault(net, []).append(seg)
    return by_key


def validate_paper_region_splits(
    splits: list[RegionSplit],
    orientation: str,
    sign_value: int,
    lo: int,
    hi: int,
) -> tuple[bool, str]:
    if len(splits) < 2:
        return False, "fewer than two paths"
    if len({split.net for split in splits}) != len(splits):
        return False, "a net appears more than once in the same region"
    if any(split.orientation != orientation for split in splits):
        return False, "inconsistent split orientation"
    if sign_value != 0 and any(split.sign != sign_value for split in splits):
        return False, "inconsistent split direction"
    low_side: list[tuple[int, int]] = []
    high_side: list[tuple[int, int]] = []
    for split in splits:
        endpoints = [split.segment[0], split.segment[-1]]
        endpoint_axes = {int(axis_value(point, orientation)) for point in endpoints}
        if endpoint_axes != {lo, hi}:
            return False, f"net{split.net} endpoints are not on the two common cut lines"
        for point in endpoints:
            axis = int(axis_value(point, orientation))
            orth = int(orthogonal_value(point, orientation))
            if axis == lo:
                low_side.append((split.net, orth))
            elif axis == hi:
                high_side.append((split.net, orth))
        for point in walk_points(split.segment):
            if axis_value(point, orientation) < lo or axis_value(point, orientation) > hi:
                return False, f"net{split.net} leaves the candidate side interval"

    low_order = [net for net, _orth in sorted(low_side, key=lambda item: (item[1], item[0]))]
    high_order = [net for net, _orth in sorted(high_side, key=lambda item: (item[1], item[0]))]
    if low_order != high_order:
        return False, "relative order changes between entry and exit sides"

    point_owner: dict[Point, int] = {}
    edge_owner: dict[tuple[Point, Point], int] = {}
    for split in splits:
        points = walk_points(split.segment)
        if len(points) != len(set(points)):
            return False, f"net{split.net} self-touches inside region"
        edges = unit_edges(split.segment)
        if len(edges) != len(set(edges)):
            return False, f"net{split.net} reuses an edge inside region"
        for point in points:
            owner = point_owner.get(point)
            if owner is not None and owner != split.net:
                return False, f"net{split.net} touches net{owner} inside region"
            point_owner[point] = split.net
        for edge in edges:
            owner = edge_owner.get(edge)
            if owner is not None and owner != split.net:
                return False, f"net{split.net} shares edge with net{owner} inside region"
            edge_owner[edge] = split.net
    return True, "ok"


def split_route_by_scan_points(route: list[Point], start: Point, end: Point) -> tuple[list[Point], list[Point], list[Point]]:
    walked = walk_points(route)
    for start_idx, point in enumerate(walked):
        if point != start:
            continue
        for end_idx in range(start_idx + 1, len(walked)):
            if walked[end_idx] == end:
                return (
                    simplify_collinear(walked[: start_idx + 1]),
                    simplify_collinear(walked[start_idx : end_idx + 1]),
                    simplify_collinear(walked[end_idx:]),
                )
    raise ValueError(f"Cannot split route between scan points {start}->{end}: {route}")


def one_grid_toward(a: Point, b: Point) -> Point:
    return a[0] + sign(b[0] - a[0]), a[1] + sign(b[1] - a[1])


def rebuild_region_split(
    split: RegionSplit,
    prefix: list[Point],
    segment: list[Point],
    suffix: list[Point],
) -> RegionSplit:
    return RegionSplit(
        net=split.net,
        route=split.route,
        prefix=prefix,
        segment=segment,
        suffix=suffix,
        orientation=split.orientation,
        sign=split.sign,
        entry_axis=int(axis_value(segment[0], split.orientation)),
        exit_axis=int(axis_value(segment[-1], split.orientation)),
        entry_orth=int(orthogonal_value(segment[0], split.orientation)),
        exit_orth=int(orthogonal_value(segment[-1], split.orientation)),
    )


def shrink_cut_boundaries_group(splits: list[RegionSplit]) -> list[RegionSplit]:
    if not splits:
        return splits
    orientations = {split.orientation for split in splits}
    signs = {split.sign for split in splits}
    if len(orientations) != 1 or len(signs) != 1:
        return splits
    orientation = next(iter(orientations))
    sign_value = next(iter(signs))
    axes = sorted(
        {
            int(axis_value(point, orientation))
            for split in splits
            for point in (split.segment[0], split.segment[-1])
        }
    )
    if len(axes) != 2:
        return splits
    shrink_axes = set(axes)

    for split in splits:
        for endpoint, neighbor in ((split.segment[0], split.segment[1]), (split.segment[-1], split.segment[-2])):
            endpoint_axis = int(axis_value(endpoint, orientation))
            if endpoint_axis in shrink_axes and int(axis_value(one_grid_toward(endpoint, neighbor), orientation)) == endpoint_axis:
                return splits

    trimmed: list[RegionSplit] = []
    for split in splits:
        route = simplify_collinear(split.route)
        start = split.segment[0]
        end = split.segment[-1]
        trimmed_start = one_grid_toward(start, split.segment[1]) if int(axis_value(start, orientation)) in shrink_axes else start
        trimmed_end = one_grid_toward(end, split.segment[-2]) if int(axis_value(end, orientation)) in shrink_axes else end
        try:
            prefix, segment, suffix = split_route_by_scan_points(route, trimmed_start, trimmed_end)
        except ValueError:
            return splits
        trimmed.append(rebuild_region_split(split, prefix, segment, suffix))
    trimmed_axes = sorted(
        {
            int(axis_value(point, orientation))
            for split in trimmed
            for point in (split.segment[0], split.segment[-1])
        }
    )
    if len(trimmed_axes) != 2:
        return splits
    valid, _reason = validate_paper_region_splits(trimmed, orientation, sign_value, trimmed_axes[0], trimmed_axes[1])
    return trimmed if valid else splits


def scanline_state_key(state: tuple[tuple[int, int, int], ...]) -> tuple[tuple[int, int], ...]:
    return tuple((net, sign_value) for net, _orth, sign_value in state)


def scanline_edge_states(paths: PathData, orientation: str) -> dict[int, tuple[tuple[int, int, int], ...] | None]:
    states_by_axis: dict[int, tuple[tuple[int, int, int], ...] | None] = {}
    entries_by_axis: dict[int, list[tuple[int, int, int]]] = {}
    duplicate_keys: set[tuple[int, int]] = set()
    seen_keys: set[tuple[int, int]] = set()
    for net, route in enumerate(paths):
        walked = walk_points(route)
        for a, b in zip(walked, walked[1:]):
            dx = int(b[0] - a[0])
            dy = int(b[1] - a[1])
            if orientation == "horizontal":
                if dy != 0 or dx == 0:
                    continue
                slab_axis = min(int(a[0]), int(b[0]))
                orth = int(a[1])
                sign_value = sign(dx)
            else:
                if dx != 0 or dy == 0:
                    continue
                slab_axis = min(int(a[1]), int(b[1]))
                orth = int(a[0])
                sign_value = sign(dy)
            duplicate_key = (slab_axis, net)
            if duplicate_key in seen_keys:
                duplicate_keys.add(duplicate_key)
                continue
            seen_keys.add(duplicate_key)
            entries_by_axis.setdefault(slab_axis, []).append((net, orth, sign_value))

    for slab_axis, entries in entries_by_axis.items():
        if any((slab_axis, net) in duplicate_keys for net, _orth, _sign_value in entries):
            states_by_axis[slab_axis] = None
            continue
        ordered = tuple(sorted(entries, key=lambda item: (item[1], item[0])))
        if len(ordered) < 2:
            continue
        states_by_axis[slab_axis] = ordered
    return states_by_axis


def scanline_edge_state_groups(paths: PathData, orientation: str) -> dict[int, dict[int, tuple[tuple[int, int, int], ...]] | None]:
    grouped_by_axis: dict[int, dict[int, tuple[tuple[int, int, int], ...]] | None] = {}
    entries_by_axis: dict[int, list[tuple[int, int, int]]] = {}
    duplicate_keys: set[tuple[int, int]] = set()
    seen_keys: set[tuple[int, int]] = set()
    for net, route in enumerate(paths):
        walked = walk_points(route)
        for a, b in zip(walked, walked[1:]):
            dx = int(b[0] - a[0])
            dy = int(b[1] - a[1])
            if orientation == "horizontal":
                if dy != 0 or dx == 0:
                    continue
                slab_axis = min(int(a[0]), int(b[0]))
                orth = int(a[1])
                sign_value = sign(dx)
            else:
                if dx != 0 or dy == 0:
                    continue
                slab_axis = min(int(a[1]), int(b[1]))
                orth = int(a[0])
                sign_value = sign(dy)
            duplicate_key = (slab_axis, net)
            if duplicate_key in seen_keys:
                duplicate_keys.add(duplicate_key)
                continue
            seen_keys.add(duplicate_key)
            entries_by_axis.setdefault(slab_axis, []).append((net, orth, sign_value))

    for slab_axis, entries in entries_by_axis.items():
        if any((slab_axis, net) in duplicate_keys for net, _orth, _sign_value in entries):
            grouped_by_axis[slab_axis] = None
            continue
        groups: dict[int, tuple[tuple[int, int, int], ...]] = {}
        for sign_value in sorted({item[2] for item in entries}):
            ordered = tuple(sorted((item for item in entries if item[2] == sign_value), key=lambda item: (item[1], item[0])))
            if len(ordered) >= 2:
                groups[sign_value] = ordered
        if groups:
            grouped_by_axis[slab_axis] = groups
    return grouped_by_axis


def scanline_region_candidate(
    paths: PathData,
    orientation: str,
    start_state: tuple[tuple[int, int, int], ...],
    end_state: tuple[tuple[int, int, int], ...],
    start_axis: int,
    end_axis: int,
) -> RegionCandidate | None:
    if end_axis - start_axis <= 1:
        return None
    lo, hi = sorted((start_axis, end_axis))
    splits: list[RegionSplit] = []
    if scanline_state_key(start_state) != scanline_state_key(end_state):
        return None
    end_by_net = {net: (orth, sign_value) for net, orth, sign_value in end_state}
    signs = {sign_value for _net, _orth, sign_value in start_state}
    if len(signs) != 1:
        return None
    candidate_sign = next(iter(signs))
    low_side_orths: list[int] = []
    high_side_orths: list[int] = []
    for net, start_orth, sign_value in start_state:
        end_orth, end_sign = end_by_net[net]
        if end_sign != sign_value:
            return None
        if orientation == "horizontal":
            start = (start_axis, start_orth) if sign_value > 0 else (end_axis, end_orth)
            end = (end_axis, end_orth) if sign_value > 0 else (start_axis, start_orth)
        else:
            start = (start_orth, start_axis) if sign_value > 0 else (end_orth, end_axis)
            end = (end_orth, end_axis) if sign_value > 0 else (start_orth, start_axis)
        low_side_orths.append(start_orth)
        high_side_orths.append(end_orth)
        prefix, segment, suffix = split_route_by_scan_points(paths[net], start, end)
        splits.append(
            RegionSplit(
                net=net,
                route=paths[net],
                prefix=prefix,
                segment=segment,
                suffix=suffix,
                orientation=orientation,
                sign=sign_value,
                entry_axis=int(axis_value(segment[0], orientation)),
                exit_axis=int(axis_value(segment[-1], orientation)),
                entry_orth=int(orthogonal_value(segment[0], orientation)),
                exit_orth=int(orthogonal_value(segment[-1], orientation)),
            )
        )
    valid, _reason = validate_paper_region_splits(splits, orientation, candidate_sign, lo, hi)
    if not valid:
        return None
    span = max(max(low_side_orths), max(high_side_orths)) - min(min(low_side_orths), min(high_side_orths))
    return RegionCandidate(
        group=f"{orientation}_{'pos' if candidate_sign > 0 else 'neg' if candidate_sign < 0 else 'mixed'}",
        orientation=orientation,
        sign=candidate_sign,
        splits=splits,
        lo=lo,
        hi=hi,
        area=(hi - lo) * max(1, span),
        span=span,
        overlap=hi - lo,
    )


def paper_region_candidates(paths: PathData) -> list[RegionCandidate]:
    sorted_paths = sort_routes(paths)
    candidates: list[RegionCandidate] = []
    for orientation in ("horizontal", "vertical"):
        state_groups_by_axis = scanline_edge_state_groups(sorted_paths, orientation)
        sign_values = sorted(
            {
                sign_value
                for groups in state_groups_by_axis.values()
                if groups is not None
                for sign_value in groups
            }
        )
        for sign_value in sign_values:
            active_key: tuple[tuple[int, int], ...] | None = None
            active_start_state: tuple[tuple[int, int, int], ...] | None = None
            active_end_state: tuple[tuple[int, int, int], ...] | None = None
            active_start: int | None = None
            active_end: int | None = None
            for axis in sorted(state_groups_by_axis):
                groups = state_groups_by_axis[axis]
                state = None if groups is None else groups.get(sign_value)
                state_key = scanline_state_key(state) if state is not None else None
                if state is not None and state_key == active_key and active_end == axis:
                    active_end_state = state
                    active_end = axis + 1
                    continue
                if (
                    active_start_state is not None
                    and active_end_state is not None
                    and active_start is not None
                    and active_end is not None
                ):
                    candidate = scanline_region_candidate(
                        sorted_paths, orientation, active_start_state, active_end_state, active_start, active_end
                    )
                    if candidate is not None:
                        candidates.append(candidate)
                if state is None:
                    active_key = None
                    active_start_state = None
                    active_end_state = None
                    active_start = None
                    active_end = None
                else:
                    active_key = state_key
                    active_start_state = state
                    active_end_state = state
                    active_start = axis
                    active_end = axis + 1
            if (
                active_start_state is not None
                and active_end_state is not None
                and active_start is not None
                and active_end is not None
            ):
                candidate = scanline_region_candidate(
                    sorted_paths, orientation, active_start_state, active_end_state, active_start, active_end
                )
                if candidate is not None:
                    candidates.append(candidate)
    return sorted(candidates, key=lambda item: (-item.overlap, -item.area, -item.span, item.group))


def region_gap_penalty(candidate: RegionCandidate, splits: list[RegionSplit], target_full_lengths: list[int]) -> int:
    penalty = 0
    for split in splits:
        segment_length = manhattan_length(split.segment)
        outside_length = manhattan_length(split.route) - segment_length
        target_region = target_full_lengths[split.net] - outside_length
        penalty += max(0, int(target_region - segment_length))
    return penalty


def select_overlap_groups(paths: PathData, target_full_lengths: list[int]) -> list[tuple[str, list[RegionSplit], int, int]]:
    candidates = paper_region_candidates(paths)
    if not candidates:
        raise ValueError("No paper-valid detour region was found.")
    all_nets = set(range(len(paths)))
    best_full: tuple[tuple[int, int, int], list[tuple[RegionCandidate, list[RegionSplit]]]] | None = None
    orientations = sorted({candidate.orientation for candidate in candidates})
    for orientation in orientations:
        orientation_candidates = [candidate for candidate in candidates if candidate.orientation == orientation]
        dp: dict[frozenset[int], tuple[tuple[int, int, int], list[tuple[RegionCandidate, list[RegionSplit]]]]] = {
            frozenset(): ((0, 0, 0), [])
        }
        for candidate in orientation_candidates:
            candidate_nets = {split.net for split in candidate.splits}
            updates = dict(dp)
            for covered, (score, selection) in dp.items():
                fresh_nets = candidate_nets - set(covered)
                if len(fresh_nets) < 2:
                    continue
                fresh = [split for split in candidate.splits if split.net in fresh_nets]
                valid, _reason = validate_paper_region_splits(
                    fresh, candidate.orientation, candidate.sign, candidate.lo, candidate.hi
                )
                if not valid:
                    continue
                next_covered = frozenset(set(covered) | fresh_nets)
                next_score = (
                    score[0] + len(fresh) * candidate.overlap,
                    score[1] + len(fresh) * candidate.area,
                    score[2] - 1,
                )
                if next_covered not in updates or next_score > updates[next_covered][0]:
                    updates[next_covered] = (next_score, selection + [(candidate, fresh)])
            dp = updates
        full = dp.get(frozenset(all_nets))
        if full is not None:
            if best_full is None or full[0] > best_full[0]:
                best_full = full
    if best_full is not None:
        selected = []
        for idx, (candidate, fresh) in enumerate(best_full[1]):
            selected.append((f"{candidate.group}_{idx}", fresh, candidate.lo, candidate.hi))
        return selected
    missing = sorted(all_nets - set().union(*(set(split.net for split in candidate.splits) for candidate in candidates)))
    raise ValueError(f"Paper-valid detour regions do not cover all nets: missing={missing}")


def split_on_overlap(seg: CandidateSegment, lo: int, hi: int) -> tuple[list[Point], list[Point], list[Point]]:
    route = seg.route
    a, b = route[seg.index], route[seg.index + 1]
    if seg.orientation == "horizontal":
        start = (lo, seg.fixed_coord) if seg.sign > 0 else (hi, seg.fixed_coord)
        end = (hi, seg.fixed_coord) if seg.sign > 0 else (lo, seg.fixed_coord)
    else:
        start = (seg.fixed_coord, lo) if seg.sign > 0 else (seg.fixed_coord, hi)
        end = (seg.fixed_coord, hi) if seg.sign > 0 else (seg.fixed_coord, lo)
    prefix = list(route[: seg.index + 1])
    if prefix[-1] != start:
        prefix.append(start)
    suffix = [end]
    if end != b:
        suffix.append(b)
    suffix.extend(route[seg.index + 2 :])
    return simplify_collinear(prefix), [start, end], simplify_collinear(suffix)


def select_region_segments(
    paths: PathData,
    target_full_lengths: list[int],
) -> list[SegmentEntry]:
    entries: list[SegmentEntry] = []
    sorted_paths = sort_routes(paths)
    groups = select_overlap_groups(sorted_paths, target_full_lengths)
    for group_name, splits, _lo, _hi in groups:
        splits = shrink_cut_boundaries_group(splits)
        for split in splits:
            route = simplify_collinear(split.route)
            target_full = target_full_lengths[split.net]
            prefix, segment, suffix = split.prefix, split.segment, split.suffix
            segment_length = manhattan_length(segment)
            outside_length = manhattan_length(route) - segment_length
            target_region = target_full - outside_length
            if target_region < segment_length:
                target_region = segment_length
                target_full = outside_length + segment_length
            entries.append(
                SegmentEntry(
                    group_name,
                    split.net,
                    route,
                    prefix,
                    segment,
                    suffix,
                    outside_length,
                    target_full,
                    target_region,
                )
            )
    entries.sort(key=lambda entry: entry.net)
    return entries


def splice_region(entry: SegmentEntry, detoured_segment: list[Point]) -> list[Point]:
    detoured_segment = simplify_collinear(detoured_segment)
    if detoured_segment[0] == entry.segment[-1] and detoured_segment[-1] == entry.segment[0]:
        detoured_segment = list(reversed(detoured_segment))
    if detoured_segment[0] != entry.segment[0] or detoured_segment[-1] != entry.segment[-1]:
        raise ValueError(f"Detoured segment endpoints do not match {entry.segment[0]}->{entry.segment[-1]}")
    return simplify_collinear(entry.prefix[:-1] + detoured_segment + entry.suffix[1:])


def region_static_block_points(entries: Sequence[SegmentEntry]) -> tuple[Point, ...]:
    region_endpoints = {entry.segment[0] for entry in entries} | {entry.segment[-1] for entry in entries}
    blocked = blocked_grid_points(
        route
        for entry in entries
        for route in (entry.prefix, entry.suffix)
        if len(route) > 1
    )
    blocked.difference_update(region_endpoints)
    return tuple(sorted(blocked))


def preferred_spiral_endpoint(entry: SegmentEntry) -> tuple[Point, str, int, int]:
    prefix_length = manhattan_length(entry.prefix)
    suffix_length = manhattan_length(entry.suffix)
    if prefix_length <= suffix_length:
        return entry.segment[0], "route_start_side", prefix_length, suffix_length
    return entry.segment[-1], "route_end_side", prefix_length, suffix_length


def run_detour(out_dir: Path, case: dict[str, object], astar_paths: PathData) -> PathData:
    stage = mkdir(out_dir / "02_detour")
    detour_cfg = case.get("detour", {}) or {}
    detour_target = detour_cfg.get("target_length", case["target_length"])
    detour_target_name = "detour.target_length" if "target_length" in detour_cfg else "target_length"
    target_lengths = scalar_or_list(detour_target, len(astar_paths), detour_target_name)
    detour_source_paths = sort_routes(astar_paths)
    entries = select_region_segments(
        detour_source_paths,
        target_lengths,
    )
    bbox = tuple(int(v) for v in case["bbox"])
    obstacles = tuple(obstacle_points(case))
    direction_priority = str(detour_cfg.get("direction_priority", "fixed_original"))
    placement = str(detour_cfg.get("placement", "legacy_longest"))

    write_text(
        stage / "detour_input.json",
        json.dumps(
            {
                "bbox": list(bbox),
                "obstacle_rectangles": [list(rect) for rect in obstacle_rectangles(case)],
                "target_lengths": target_lengths,
                "region_selection_policy": "paper_scanline_common_cut_axis_order_preserved_no_crossing",
                "region_engine": "legacy_contour_detour",
                "direction_priority": direction_priority,
                "placement": placement,
                "segment_entries": [
                    {
                        "net": entry.net,
                        "group": entry.group,
                        "segment": entry.segment,
                        "preferred_spiral_endpoint": preferred[0],
                        "preferred_spiral_endpoint_side": preferred[1],
                        "prefix_length_to_segment_start": preferred[2],
                        "suffix_length_from_segment_end": preferred[3],
                        "outside_length": entry.outside_length,
                        "target_region_length": entry.target_region_length,
                    }
                    for entry in entries
                    for preferred in (preferred_spiral_endpoint(entry),)
                ],
            },
            indent=2,
        )
        + "\n",
    )

    detoured_by_key: dict[tuple[Point, Point], list[Point]] = {}
    metadata: list[dict[str, object]] = []
    region_inputs: PathData = []
    region_outputs: PathData = []

    def run_region_entries(group_entries: list[SegmentEntry], label: str) -> None:
        region_paths = [entry.segment for entry in group_entries]
        target_by_key = {endpoint_key(entry.segment): entry.target_region_length for entry in group_entries}
        preferred_by_key = {endpoint_key(entry.segment): preferred_spiral_endpoint(entry)[0] for entry in group_entries}
        sorted_region_paths = sort_routes(region_paths)
        sorted_targets = tuple(target_by_key[endpoint_key(route)] for route in sorted_region_paths)
        sorted_preferred = tuple(preferred_by_key[endpoint_key(route)] for route in sorted_region_paths)
        static_block_points = tuple(sorted(set(obstacles) | set(region_static_block_points(group_entries))))
        if all(target == manhattan_length(route) for target, route in zip(sorted_targets, sorted_region_paths)):
            for route in sorted_region_paths:
                detoured_by_key[endpoint_key(route)] = simplify_collinear(route)
            region_inputs.extend(sorted_region_paths)
            region_outputs.extend(sorted_region_paths)
            metadata.append(
                {
                    "group": label,
                    "targets": list(sorted_targets),
                    "source": "noop",
                    "static_block_points_grid": len(static_block_points),
                }
            )
            return
        config = LegacyContourDetourConfig(
            script_path=DETOUR_KERNEL,
            target_total_length_grid=max(sorted_targets),
            expected_explicit_lengths_grid=sorted_targets,
            auto_region_search_area_grid=bbox,
            hard_boundary_area_grid=bbox,
            external_block_points_grid=static_block_points,
            detour_area_slack_delta=float(detour_cfg.get("area_slack_delta", 0.03)),
            spiral_placement_policy=placement,
            spiral_preferred_endpoints_grid=sorted_preferred if placement == "nearest_endpoint" else None,
            detour_direction_priority=direction_priority,
        )
        result = run_legacy_contour_detour(sorted_region_paths, config)
        for route in result.paths:
            detoured_by_key[endpoint_key(route)] = route
        region_inputs.extend(sorted_region_paths)
        region_outputs.extend(result.paths)
        metadata.append(
            {
                "group": label,
                "targets": list(sorted_targets),
                "static_block_points_grid": len(static_block_points),
                "detour_metadata": result.metadata,
            }
        )

    for group_name in sorted({entry.group for entry in entries}):
        run_region_entries([entry for entry in entries if entry.group == group_name], group_name)

    final_by_route_key: dict[tuple[Point, Point], list[Point]] = {}
    for entry in entries:
        final_by_route_key[endpoint_key(entry.original_route)] = splice_region(
            entry,
            detoured_by_key[endpoint_key(entry.segment)],
        )
    final_paths = [final_by_route_key.get(endpoint_key(route), simplify_collinear(route)) for route in astar_paths]
    write_case_literal(stage / "detour_region_segments.txt", region_inputs)
    write_case_literal(stage / "detour_region_output.txt", region_outputs)
    write_case_literal(stage / "detour_paths.txt", final_paths)
    write_text(stage / "detour_metadata.json", json.dumps(metadata, indent=2) + "\n")
    return final_paths


def stage_metrics(stage: str, paths: PathData, case: dict[str, object], *, preserve_order: bool = False) -> list[dict[str, object]]:
    ordered_paths = [list(route) for route in paths] if preserve_order else sort_routes(paths)
    measured_paths = [measurement_route(route, case) for route in ordered_paths]
    geometry_lengths = [manhattan_length(route) for route in measured_paths]
    external_lengths = metric_external_lengths(case, len(ordered_paths))
    lengths = [geometry + external for geometry, external in zip(geometry_lengths, external_lengths)]
    if "target_length" in case:
        target_lengths = scalar_or_list(case["target_length"], len(ordered_paths), "target_length")
        if len(case.get("endpoints", [])) == len(ordered_paths):
            for idx in general_net_indices(case):
                if idx < len(target_lengths):
                    target_lengths[idx] = None
    else:
        target_lengths = [None for _ in ordered_paths]
    bends = [measurement_bend_count(route, case) for route in ordered_paths]
    crossings = crossing_counts(ordered_paths)
    rows = []
    prop_loss = propagation_loss_per_um(case)
    pitch = pitch_um(case)
    cross_loss = crossing_loss_value(case)
    for net, (route, measured_route, geometry_length, external_length, length, target_length, bend, crossing) in enumerate(
        zip(ordered_paths, measured_paths, geometry_lengths, external_lengths, lengths, target_lengths, bends, crossings)
    ):
        target_error = "" if target_length is None else clean_number(float(length) - float(target_length))
        path_loss = round(length * pitch * prop_loss, 6)
        bend_loss = round(route_bend_loss(route, case), 6)
        crossing_loss = round(crossing * cross_loss, 6)
        rows.append(
            {
                "stage": stage,
                "net": net,
                "start": route[0],
                "end": route[-1],
                "measured_start": measured_route[0],
                "measured_end": measured_route[-1],
                "geometry_length_grid": geometry_length,
                "external_length_grid": external_length,
                "length_grid": length,
                "target_length_grid": "" if target_length is None else target_length,
                "target_length_error_grid": target_error,
                "bend_count": bend,
                "crossing_count": crossing,
                "path_loss": path_loss,
                "bend_loss": bend_loss,
                "crossing_loss": crossing_loss,
                "total_loss": round(path_loss + bend_loss + crossing_loss, 6),
            }
        )
    return rows


def max_total_loss(paths: PathData, case: dict[str, object]) -> float:
    return max(row["total_loss"] for row in stage_metrics("loss_check", paths, case))


def endpoint_signature(paths: PathData) -> list[tuple[Point, Point]]:
    return sorted(endpoint_key(route) for route in paths)


def run_lut(case: dict[str, object], detour_paths: PathData) -> PathData:
    optimized = lut_optimizer(LUT_TABLE, lut_max_level(case)).optimize(detour_paths)
    return [[to_point(point) for point in route] for route in optimized]


def adopt_lut_if_loss_safe(out_dir: Path, case: dict[str, object], detour_paths: PathData, lut_paths: PathData) -> tuple[PathData, dict[str, object]]:
    stage = mkdir(out_dir / "03_lut")
    detour_max = max_total_loss(detour_paths, case)
    lut_max = max_total_loss(lut_paths, case)
    accepted = (
        endpoint_signature(detour_paths) == endpoint_signature(lut_paths)
        and lut_max < detour_max - LOSS_EPS
    )
    reason = "lut_strictly_reduces_raw_max_loss"
    if endpoint_signature(detour_paths) != endpoint_signature(lut_paths):
        reason = "endpoint_signature_mismatch"
    elif lut_max >= detour_max - LOSS_EPS:
        reason = "lut_does_not_strictly_reduce_raw_max_loss"
    adopted = lut_paths if accepted else detour_paths
    write_case_literal(stage / "lut_paths.txt", adopted)
    gate = {
        "accepted": accepted,
        "reason": reason,
        "detour_max_loss": detour_max,
        "lut_raw_max_loss": lut_max,
        "adopted_source": "lut" if accepted else "detour",
    }
    write_text(stage / "lut_acceptance.json", json.dumps(gate, indent=2) + "\n")
    return adopted, gate


def apply_final_bend_insertion(out_dir: Path, case: dict[str, object], paths: PathData, obstacles: Sequence[Point]) -> PathData:
    stage = mkdir(out_dir / "04_final")
    final_cfg = case.get("final", {}) or {}
    ordered_paths = [canonical_route(route) for route in paths]
    if final_cfg.get("bend_insertion", True) is False:
        rows = [
            {
                "net": net,
                "base_length_grid": manhattan_length(route),
                "base_metric_length_grid": measurement_length(route, case),
                "base_bends": bend_count(route),
                "base_metric_bends": measurement_bend_count(route, case),
                "target_metric_bends": measurement_bend_count(route, case),
                "target_full_bends": bend_count(route),
                "final_length_grid": manhattan_length(route),
                "final_metric_length_grid": measurement_length(route, case),
                "final_bends": bend_count(route),
                "final_metric_bends": measurement_bend_count(route, case),
                "status": "skipped",
                "operators": "",
            }
            for net, route in enumerate(ordered_paths)
        ]
        write_csv(stage / "bend_insertion_ops.csv", rows)
        write_case_literal(stage / "final_paths.txt", ordered_paths)
        return [list(route) for route in ordered_paths]

    target_bend = max(measurement_bend_count(route, case) for route in ordered_paths) if ordered_paths else 0
    current: PathData = [list(route) for route in ordered_paths]
    rows: list[dict[str, object]] = []
    for net, route in enumerate(ordered_paths):
        other_routes = [item for idx, item in enumerate(current) if idx != net]
        blocked_points = blocked_grid_points(other_routes, obstacles)
        base_bends = bend_count(route)
        base_metric_bends = measurement_bend_count(route, case)
        target_full_bends = base_bends + max(0, target_bend - base_metric_bends)
        history = (
            insert_bends_to_target(route, target_full_bends, allow_partial=True, blocked_points=blocked_points)
            if base_metric_bends < target_bend
            else []
        )
        final_route = history[-1].path if history else route
        current[net] = final_route
        rows.append(
            {
                "net": net,
                "base_length_grid": manhattan_length(route),
                "base_metric_length_grid": measurement_length(route, case),
                "base_bends": base_bends,
                "base_metric_bends": base_metric_bends,
                "target_metric_bends": target_bend,
                "target_full_bends": target_full_bends,
                "final_length_grid": manhattan_length(final_route),
                "final_metric_length_grid": measurement_length(final_route, case),
                "final_bends": bend_count(final_route),
                "final_metric_bends": measurement_bend_count(final_route, case),
                "status": "ok" if measurement_bend_count(final_route, case) == target_bend else "partial",
                "operators": " -> ".join(f"{item.operator}+{item.added_bends}@{item.replaced_range}" for item in history),
            }
    )
    write_csv(stage / "bend_insertion_ops.csv", rows)
    if bool(final_cfg.get("length_padding", True)):
        current = apply_final_u_bend_length_padding(stage, case, current)
    write_case_literal(stage / "final_paths.txt", current)
    return current


def expand_u_bend_bridge(route: Sequence[Point], start_idx: int, delta: float) -> list[Point] | None:
    path = simplify_collinear(route)
    if start_idx + 3 >= len(path) or delta <= 0:
        return None
    p0, p1, p2, p3 = path[start_idx : start_idx + 4]
    try:
        leg_in = unit_direction(p0, p1)
        bridge = unit_direction(p1, p2)
        leg_out = unit_direction(p2, p3)
    except ValueError:
        return None
    if leg_in != neg(leg_out) or dot(leg_in, bridge) != 0:
        return None
    q1 = add_scaled(p1, leg_in, delta)
    q2 = add_scaled(p2, leg_in, delta)
    return simplify_collinear(list(path[: start_idx + 1]) + [q1, q2] + list(path[start_idx + 3 :]))


def is_valid_final_padding_candidate(
    route: Sequence[Point],
    other_routes: Sequence[Sequence[Point]],
    obstacle_rects: Sequence[tuple[int, int, int, int]],
) -> bool:
    try:
        if route_self_intersects(route):
            return False
        if route_intersects_routes(route, other_routes):
            return False
        if route_intersects_obstacles(route, obstacle_rects):
            return False
    except ValueError:
        return False
    return True


def apply_final_u_bend_length_padding(
    stage: Path,
    case: dict[str, object],
    paths: PathData,
) -> PathData:
    target_lengths = scalar_or_list(case["target_length"], len(paths), "target_length")
    obstacle_rects = obstacle_rectangles(case)
    current: PathData = [list(route) for route in paths]
    rows: list[dict[str, object]] = []
    for net, (route, target_length) in enumerate(zip(list(current), target_lengths)):
        base_metric_length = measurement_length(route, case)
        deficit = float(target_length) - float(base_metric_length)
        row: dict[str, object] = {
            "net": net,
            "base_length_grid": manhattan_length(route),
            "base_metric_length_grid": base_metric_length,
            "target_length_grid": target_length,
            "requested_delta_grid": clean_number(deficit / 2.0) if deficit > LOSS_EPS else 0,
            "final_length_grid": manhattan_length(route),
            "final_metric_length_grid": base_metric_length,
            "status": "already_at_or_above_target",
            "u_bend_start_idx": "",
        }
        if deficit <= LOSS_EPS:
            rows.append(row)
            continue

        other_routes = [item for idx, item in enumerate(current) if idx != net]
        delta = deficit / 2.0
        candidates: list[tuple[float, int, list[Point]]] = []
        for start_idx in range(max(0, len(simplify_collinear(route)) - 3)):
            candidate = expand_u_bend_bridge(route, start_idx, delta)
            if candidate is None:
                continue
            if not is_valid_final_padding_candidate(candidate, other_routes, obstacle_rects):
                continue
            candidate_metric_length = measurement_length(candidate, case)
            if candidate_metric_length > target_length + LOSS_EPS:
                continue
            improvement = float(candidate_metric_length) - float(base_metric_length)
            if improvement <= LOSS_EPS:
                continue
            gap = abs(float(target_length) - float(candidate_metric_length))
            candidates.append((gap, start_idx, candidate))

        if candidates:
            gap, start_idx, chosen = min(candidates, key=lambda item: (item[0], item[1]))
            current[net] = chosen
            row.update(
                {
                    "final_length_grid": manhattan_length(chosen),
                    "final_metric_length_grid": measurement_length(chosen, case),
                    "status": "ok" if gap <= LOSS_EPS else "partial",
                    "u_bend_start_idx": start_idx,
                }
            )
        else:
            row["status"] = "no_feasible_u_bend"
        rows.append(row)

    base_lengths = [measurement_length(route, case) for route in paths]
    padded_lengths = [measurement_length(route, case) for route in current]
    if max_difference(padded_lengths) > max_difference(base_lengths) + LOSS_EPS:
        for row in rows:
            row["final_length_grid"] = row["base_length_grid"]
            row["final_metric_length_grid"] = row["base_metric_length_grid"]
            if row["status"] in {"ok", "partial"}:
                row["status"] = "rejected_would_worsen_length_difference"
        write_csv(stage / "length_padding_ops.csv", rows)
        return [list(route) for route in paths]

    write_csv(stage / "length_padding_ops.csv", rows)
    return current


def max_difference(values: Sequence[float | int]) -> float | int:
    return max(values) - min(values) if values else 0


def summarize(case: dict[str, object], paths: PathData, *, preserve_order: bool = False) -> dict[str, object]:
    rows = stage_metrics("final", paths, case, preserve_order=preserve_order)
    matching_rows = [row for row in rows if row["target_length_grid"] != ""]
    diff_rows = matching_rows or rows
    return {
        "case": case.get("case", "case"),
        "max_loss": max(row["total_loss"] for row in rows),
        "total_loss": round(sum(row["total_loss"] for row in rows), 6),
        "max_bend_difference": max_difference([int(row["bend_count"]) for row in diff_rows]),
        "max_length_difference": clean_number(max_difference([float(row["length_grid"]) for row in diff_rows])),
    }


def route_loss(route: Sequence[Point], case: dict[str, object], crossing_count: int = 0) -> float:
    length = measurement_length(route, case)
    path_loss = length * pitch_um(case) * propagation_loss_per_um(case)
    bend_loss = route_bend_loss(route, case)
    crossing = crossing_count * crossing_loss_value(case)
    return round(path_loss + bend_loss + crossing, 6)


def route_losses_with_crossings(paths_by_net: dict[int, list[Point]], case: dict[str, object]) -> dict[int, float]:
    ordered_nets = list(paths_by_net)
    ordered_paths = [paths_by_net[idx] for idx in ordered_nets]
    crossings = crossing_counts(ordered_paths)
    return {
        idx: route_loss(paths_by_net[idx], case, crossings[pos])
        for pos, idx in enumerate(ordered_nets)
    }


def general_edge_capacity(case: dict[str, object]) -> float:
    general_cfg = case.get("general", {}) or {}
    pitch = pitch_um(case)
    waveguide_width_um = float(case.get("waveguide_width_um", 0.5))
    spacing_um = float(general_cfg.get("spacing_um", case.get("spacing_um", waveguide_width_um)))
    return pitch / (spacing_um + waveguide_width_um)


def overloss_penalty(overloss_ratio: float, case: dict[str, object]) -> float:
    general_cfg = case.get("general", {}) or {}
    scale = float(general_cfg.get("history_lambda", 0.02))
    sensitivity = float(general_cfg.get("history_t", general_cfg.get("t", 1.6094379124341003)))
    ratio = max(0.0, overloss_ratio)
    return scale / (1.0 + math.exp(-sensitivity * ratio))


def grid_key_for_hit(hit: tuple[object, ...]) -> Point | None:
    if not hit:
        return None
    if hit[0] == "legal_crossing" and len(hit) >= 4:
        return clean_number(round(float(hit[2]))), clean_number(round(float(hit[3])))
    if hit[0] in {"illegal_crossing", "illegal_touch"} and len(hit) >= 5:
        return clean_number(round(float(hit[3]))), clean_number(round(float(hit[4])))
    if hit[0] == "overlap" and len(hit) >= 2:
        segment = hit[1]
        if isinstance(segment, tuple) and len(segment) == 2:
            a, b = segment
            return clean_number(round((float(a[0]) + float(b[0])) / 2.0)), clean_number(
                round((float(a[1]) + float(b[1])) / 2.0)
            )
    return None


def grid_loss_hotspot(paths_by_net: dict[int, list[Point]], case: dict[str, object]) -> tuple[Point | None, set[int], float]:
    grid_loss: dict[Point, float] = {}
    grid_nets: dict[Point, set[int]] = {}
    prop_loss = propagation_loss_per_um(case)
    pitch = pitch_um(case)
    cross_loss = crossing_loss_value(case)
    for net, route in paths_by_net.items():
        points = walk_points(route)
        for a, b in zip(points, points[1:]):
            length = abs(float(a[0]) - float(b[0])) + abs(float(a[1]) - float(b[1]))
            key = b
            grid_loss[key] = grid_loss.get(key, 0.0) + length * pitch * prop_loss
            grid_nets.setdefault(key, set()).add(net)
        simplified = simplify_collinear(route)
        for a, b, c in zip(simplified, simplified[1:], simplified[2:]):
            v1 = (b[0] - a[0], b[1] - a[1])
            v2 = (c[0] - b[0], c[1] - b[1])
            if v1[0] * v2[1] - v1[1] * v2[0] != 0:
                grid_loss[b] = grid_loss.get(b, 0.0) + bend_loss_at(a, b, c, case)
                grid_nets.setdefault(b, set()).add(net)

    nets = list(paths_by_net)
    for left_pos, left_net in enumerate(nets):
        for right_net in nets[left_pos + 1 :]:
            for a, b in unit_segments(paths_by_net[left_net]):
                for c, d in unit_segments(paths_by_net[right_net]):
                    hit = segment_intersection_key(a, b, c, d)
                    key = grid_key_for_hit(hit) if hit is not None else None
                    if key is None:
                        continue
                    penalty = 2.0 * cross_loss if hit[0] == "legal_crossing" else 10.0 * cross_loss
                    grid_loss[key] = grid_loss.get(key, 0.0) + penalty
                    grid_nets.setdefault(key, set()).update({left_net, right_net})

    if not grid_loss:
        return None, set(), 0.0
    key = max(grid_loss, key=grid_loss.get)
    return key, grid_nets.get(key, set()), grid_loss[key]


def run_general_astar_once(
    case: dict[str, object],
    fixed_paths: PathData,
    hard_seed_count: int,
    ordered_general_nets: Sequence[int],
    history_costs: dict[Point, float],
    history_penalty: float,
) -> dict[int, list[Point]]:
    general_case = subset_case(case, ordered_general_nets, matching_only=False)
    pitch_um = float(case.get("pitch_um", PITCH_UM))
    nets, obstacles, origin = astar_memory_inputs(general_case)

    bbox = [int(v) for v in case["bbox"]]
    astar_cfg = case.get("astar", {}) or {}
    general_cfg = case.get("general", {}) or {}
    edge_capacity = general_edge_capacity(case)
    shifted_seed_paths = [
        [shift_point(to_point(point), origin) for point in walk_points(route)]
        for route in fixed_paths
    ]
    shifted_history_costs = [
        [*shift_point(point, origin), float(cost)]
        for point, cost in sorted(history_costs.items())
    ]
    config: dict[str, object] = {
        "pitch": pitch_um,
        "order": "input",
        "direction": int(astar_cfg.get("direction", 1)),
        "block_pins": True,
        "rudy_weight": float(general_cfg.get("rudy_weight", astar_cfg.get("rudy_weight", 0.0))),
        "block_routed_paths": False,
        "reserve_pin_stubs": True,
        "diagonal": bool(general_cfg.get("diagonal", True)),
        "congestion_aware": True,
        "congestion_beta": float(general_cfg.get("beta", 1.5)),
        "congestion_t": float(general_cfg.get("t", 1.6094379124341003)),
        "edge_capacity": edge_capacity,
        "overflow_penalty": GENERAL_OVERFLOW_PENALTY,
        "min_bend_radius_grid": min_bend_radius_grid(case, general_cfg, 1.0),
        "history_penalty": history_penalty,
        "hard_seed_count": hard_seed_count,
        **astar_loss_config(case),
    }
    result = native_route(
        nets,
        bbox[1] - bbox[0],
        bbox[3] - bbox[2],
        obstacles,
        config,
        shifted_seed_paths,
        shifted_history_costs,
    )
    native_paths = [[to_point(point) for point in route] for route in result["routes"]]
    paths = orient_paths_to_case_endpoints(unshift_paths(native_paths, origin), general_case)
    routed = {old: path for old, path in zip(ordered_general_nets, paths)}
    if len(routed) != len(ordered_general_nets):
        raise RuntimeError(f"General A* routed {len(routed)} paths for {len(ordered_general_nets)} requested nets")
    return routed


def run_general_routing(
    out_dir: Path,
    case: dict[str, object],
    general_nets: Sequence[int],
    fixed_paths: PathData,
    fixed_matching_nets: Sequence[int],
) -> dict[int, list[Point]]:
    stage = mkdir(out_dir / "05_general_astar")
    ordered_general_nets: list[int] = list(general_nets)
    general_cfg = case.get("general", {}) or {}
    general_order_policy = str(general_cfg.get("order_policy", ""))
    if general_order_policy:
        general_case = subset_case(case, ordered_general_nets, matching_only=False)
        try:
            order_indices = route_order_indices(general_case, general_order_policy)
        except ValueError as exc:
            raise ValueError(f"Unknown general.order_policy: {general_order_policy}") from exc
        ordered_general_nets = [ordered_general_nets[idx] for idx in order_indices]

    max_iter = int(general_cfg.get("max_ripup_iterations", 0))
    max_net_loss = general_cfg.get("max_net_loss")
    max_total_loss = general_cfg.get("max_total_loss")
    general_scope = set(ordered_general_nets)
    fixed_scope = set(fixed_matching_nets)
    if general_scope & fixed_scope:
        raise ValueError(f"Rip-up scope overlaps fixed matching nets: {sorted(general_scope & fixed_scope)}")
    ripup_count = {idx: 0 for idx in ordered_general_nets}
    active = list(ordered_general_nets)
    order_rank = {net: pos for pos, net in enumerate(ordered_general_nets)}
    history: list[dict[str, object]] = []
    best: dict[int, list[Point]] = {}
    history_costs: dict[Point, float] = {}
    history_inc = float(general_cfg.get("history_increment", 1.0))
    hard_seed_count = len(fixed_paths)

    for iteration in range(max_iter + 1):
        fixed_seed_paths = list(fixed_paths) + [best[idx] for idx in general_nets if idx in best and idx not in set(active)]
        history_penalty = 0.0
        if history:
            last = history[-1]
            history_penalty = float(last.get("history_penalty_next", 0.0))
        routed = run_general_astar_once(
            case,
            fixed_seed_paths,
            hard_seed_count,
            active,
            history_costs,
            history_penalty,
        )
        best.update(routed)
        loss_context = {**{-(idx + 1): path for idx, path in enumerate(fixed_paths)}, **best}
        contextual_losses = route_losses_with_crossings(loss_context, case)
        losses = {idx: contextual_losses[idx] for idx in general_nets}
        total_loss = round(sum(losses.values()), 6)
        worst_net = max(losses, key=losses.get) if losses else None
        max_net_violation = max_net_loss is not None and losses and max(losses.values()) > float(max_net_loss) + LOSS_EPS
        total_violation = max_total_loss is not None and total_loss > float(max_total_loss) + LOSS_EPS
        violation = bool(max_net_violation or total_violation)
        hotspot, hotspot_nets, hotspot_loss = grid_loss_hotspot(loss_context, case)
        record: dict[str, object] = {
            "iteration": iteration,
            "active_nets": active,
            "fixed_matching_nets": list(fixed_matching_nets),
            "fixed_seed_count": len(fixed_seed_paths),
            "ripup_scope": list(ordered_general_nets),
            "losses": losses,
            "total_loss": total_loss,
            "worst_net": worst_net,
            "max_net_violation": max_net_violation,
            "total_violation": total_violation,
            "violation": violation,
            "hotspot": list(hotspot) if hotspot is not None else None,
            "hotspot_loss": hotspot_loss,
            "hotspot_nets": sorted(idx for idx in hotspot_nets if idx in general_scope),
            "history_grid_count": len(history_costs),
            "history_penalty": history_penalty,
        }
        history.append(record)
        if not violation or iteration >= max_iter or worst_net is None:
            break

        if total_violation and hotspot is not None:
            ripup = {idx for idx in hotspot_nets if idx in general_scope}
            threshold = float(max_total_loss) if max_total_loss is not None else total_loss
            ratio = (total_loss - threshold) / threshold if threshold > 0 else 0.0
        else:
            ripup = {worst_net}
            worst_route = best[worst_net]
            for idx in general_nets:
                if idx != worst_net and route_intersects_grid_routes(worst_route, [best[idx]]):
                    ripup.add(idx)
            threshold = float(max_net_loss) if max_net_loss is not None else losses[worst_net]
            ratio = (losses[worst_net] - threshold) / threshold if threshold > 0 else 0.0
        if not ripup:
            ripup = {worst_net}
        for idx in ripup:
            ripup_count[idx] += 1
        if not ripup <= general_scope:
            raise AssertionError(f"Rip-up attempted to include fixed/non-general nets: {sorted(ripup - general_scope)}")
        if hotspot is not None:
            history_costs[hotspot] = history_costs.get(hotspot, 0.0) + history_inc
        record["ripup_nets"] = sorted(ripup)
        record["history_penalty_next"] = overloss_penalty(ratio, case)
        active = sorted(
            ripup,
            key=lambda idx: (-ripup_count[idx], order_rank[idx]),
        )
        record["next_active_nets"] = active

    write_text(stage / "ripup_history.json", json.dumps(history, indent=2) + "\n")
    write_case_literal(stage / "general_paths.txt", [best[idx] for idx in ordered_general_nets])
    bad_fixed = [idx for idx in general_nets if route_intersects_grid_routes(best[idx], fixed_paths)]
    if bad_fixed:
        raise RuntimeError(f"General nets intersect fixed matching paths after RRR: {bad_fixed}")
    return best


def run_matching_pipeline(out_dir: Path, case: dict[str, object], matching_nets: Sequence[int]) -> tuple[dict[int, list[Point]], dict[str, object], list[dict[str, object]]]:
    matching_case = subset_case(case, matching_nets, matching_only=True)
    astar_paths = run_astar(out_dir, matching_case)
    detour_paths = run_detour(out_dir, matching_case, astar_paths)
    lut_raw = run_lut(matching_case, detour_paths)
    lut_paths, lut_gate = adopt_lut_if_loss_safe(out_dir, matching_case, detour_paths, lut_raw)
    obstacles = obstacle_points(matching_case)
    final_paths = apply_final_bend_insertion(out_dir, matching_case, lut_paths, obstacles)
    rows = (
        stage_metrics("matching_astar", astar_paths, matching_case)
        + stage_metrics("matching_detour", detour_paths, matching_case)
        + stage_metrics("matching_lut", lut_paths, matching_case)
        + stage_metrics("matching_final", final_paths, matching_case)
    )
    return {old: path for old, path in zip(matching_nets, final_paths)}, lut_gate, rows


def run_matching_only_flow(path_yml: Path, case: dict[str, object]) -> dict[str, object]:
    case_name = str(case.get("case", path_yml.parent.name))
    out_dir = ROOT / "outputs" / case_name
    if out_dir.exists():
        shutil.rmtree(out_dir)
    mkdir(out_dir)
    write_text(out_dir / "path.yml", path_yml.read_text(encoding="utf-8"))

    astar_paths = run_astar(out_dir, case)
    detour_paths = run_detour(out_dir, case, astar_paths)
    lut_raw = run_lut(case, detour_paths)
    lut_paths, lut_gate = adopt_lut_if_loss_safe(out_dir, case, detour_paths, lut_raw)
    obstacles = obstacle_points(case)
    final_paths = apply_final_bend_insertion(out_dir, case, lut_paths, obstacles)

    rows = (
        stage_metrics("astar", astar_paths, case)
        + stage_metrics("detour", detour_paths, case)
        + stage_metrics("lut", lut_paths, case)
        + stage_metrics("final", final_paths, case)
    )
    write_csv(out_dir / "metrics.csv", rows)
    summary = summarize(case, final_paths)
    summary["lut"] = lut_gate
    write_text(out_dir / "metrics.json", json.dumps(summary, indent=2) + "\n")
    write_case_literal(out_dir / "final_paths.txt", final_paths)
    return summary


def run_mixed_flow(path_yml: Path, case: dict[str, object], matching_nets: Sequence[int], general_nets: Sequence[int]) -> dict[str, object]:
    case_name = str(case.get("case", path_yml.parent.name))
    out_dir = ROOT / "outputs" / case_name
    if out_dir.exists():
        shutil.rmtree(out_dir)
    mkdir(out_dir)
    write_text(out_dir / "path.yml", path_yml.read_text(encoding="utf-8"))

    matching_paths, lut_gate, matching_rows = run_matching_pipeline(mkdir(out_dir / "matching"), case, matching_nets)
    general_paths = run_general_routing(out_dir, case, general_nets, [matching_paths[idx] for idx in matching_nets], matching_nets)

    final_paths: PathData = []
    for idx in range(len(case["endpoints"])):
        if idx in matching_paths:
            final_paths.append(matching_paths[idx])
        elif idx in general_paths:
            final_paths.append(general_paths[idx])
        else:
            raise ValueError(f"No final path for net {idx}")

    rows = (
        matching_rows
        + stage_metrics(
            "general_astar",
            [general_paths[idx] for idx in general_nets],
            subset_case(case, general_nets, matching_only=False),
            preserve_order=True,
        )
        + stage_metrics("final", final_paths, case, preserve_order=True)
    )
    write_csv(out_dir / "metrics.csv", rows)
    summary = summarize(case, final_paths, preserve_order=True)
    summary["lut"] = lut_gate
    summary["matching_nets"] = list(matching_nets)
    summary["general_nets"] = list(general_nets)
    write_text(out_dir / "metrics.json", json.dumps(summary, indent=2) + "\n")
    write_case_literal(out_dir / "final_paths.txt", final_paths)
    return summary


def run_flow(path_yml: Path) -> dict[str, object]:
    case = load_case(path_yml)
    general_nets = general_net_indices(case)
    matching_nets = matching_net_indices(case)
    if general_nets:
        return run_mixed_flow(path_yml, case, matching_nets, general_nets)
    return run_matching_only_flow(path_yml, case)


def cli_main() -> None:
    if len(sys.argv) != 2:
        print("Usage: python main.py <path.yml>", file=sys.stderr)
        raise SystemExit(2)
    summary = run_flow(Path(sys.argv[1]).resolve())
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    cli_main()
