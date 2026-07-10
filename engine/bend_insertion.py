from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence


Point = tuple[int, int]
Vector = tuple[int, int]
PathData = list[list[Point]]


@dataclass(frozen=True)
class BendInsertionResult:
    path: list[Point]
    operator: str
    before_length: int
    after_length: int
    before_bends: int
    after_bends: int
    replaced_range: tuple[int, int]
    local_before: list[Point]
    local_after: list[Point]
    metadata: dict[str, object]

    @property
    def added_bends(self) -> int:
        return self.after_bends - self.before_bends


@dataclass(frozen=True)
class SpiralMatch:
    start_idx: int
    end_idx: int
    source: Point
    target: Point
    along: Vector
    side: Vector
    detour_length: int
    target_distance: int


def to_point(point: Iterable[int]) -> Point:
    x, y = point
    return int(x), int(y)


def normalize_path(route: Sequence[Iterable[int]]) -> list[Point]:
    return [to_point(point) for point in route]


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
        if (prev[0] == cur[0] == nxt[0]) or (prev[1] == cur[1] == nxt[1]):
            continue
        simplified.append(cur)
    simplified.append(route[-1])
    return simplified


def vector(a: Point, b: Point) -> Vector:
    return b[0] - a[0], b[1] - a[1]


def sign(value: int) -> int:
    if value > 0:
        return 1
    if value < 0:
        return -1
    return 0


def unit_vector(a: Point, b: Point) -> Vector:
    dx, dy = vector(a, b)
    if dx and dy:
        raise ValueError(f"Non-manhattan segment: {a} -> {b}")
    if dx == 0 and dy == 0:
        raise ValueError(f"Zero-length segment: {a} -> {b}")
    return sign(dx), sign(dy)


def scale(v: Vector, distance: int) -> Vector:
    return v[0] * distance, v[1] * distance


def add(point: Point, *vectors: Vector) -> Point:
    x, y = point
    for dx, dy in vectors:
        x += dx
        y += dy
    return x, y


def trace_moves(start: Point, moves: Sequence[tuple[Vector, int]]) -> list[Point]:
    points = [start]
    cur = start
    for direction, distance in moves:
        if distance == 0:
            continue
        cur = add(cur, scale(direction, distance))
        points.append(cur)
    return simplify_collinear(points)


def neg(v: Vector) -> Vector:
    return -v[0], -v[1]


def dot(a: Vector, b: Vector) -> int:
    return a[0] * b[0] + a[1] * b[1]


def manhattan_distance(a: Point, b: Point) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def manhattan_length(route: Sequence[Point]) -> int:
    route = simplify_collinear(route)
    return sum(manhattan_distance(a, b) for a, b in zip(route, route[1:]))


def bend_count(route: Sequence[Point]) -> int:
    route = simplify_collinear(route)
    bends = 0
    for a, b, c in zip(route, route[1:], route[2:]):
        v1 = vector(a, b)
        v2 = vector(b, c)
        if v1[0] * v2[1] - v1[1] * v2[0] != 0:
            bends += 1
    return bends


def assert_manhattan(route: Sequence[Point]) -> None:
    for a, b in zip(route, route[1:]):
        if a[0] != b[0] and a[1] != b[1]:
            raise ValueError(f"Non-manhattan segment: {a} -> {b}")


def expanded_grid_points(route: Sequence[Point]) -> list[Point]:
    route = simplify_collinear(route)
    if not route:
        return []
    points = [route[0]]
    for a, b in zip(route, route[1:]):
        step = unit_vector(a, b)
        cur = a
        while cur != b:
            cur = add(cur, step)
            points.append(cur)
    return points


def assert_simple_polyline(route: Sequence[Point]) -> None:
    points = expanded_grid_points(route)
    seen: set[Point] = set()
    for point in points:
        if point in seen:
            raise ValueError(f"Self-crossing or repeated grid point at {point}")
        seen.add(point)


def assert_same_grid_occupancy(before: Sequence[Point], after: Sequence[Point]) -> None:
    before_points = set(expanded_grid_points(before))
    after_points = set(expanded_grid_points(after))
    if before_points != after_points:
        missing = sorted(before_points - after_points)
        extra = sorted(after_points - before_points)
        raise ValueError(f"Grid occupancy changed. Missing={missing[:8]}, extra={extra[:8]}")


def blocked_grid_points(paths: Sequence[Sequence[Iterable[int]]] | None = None, points: Iterable[Iterable[int]] | None = None) -> set[Point]:
    blocked: set[Point] = set()
    for route in paths or []:
        blocked.update(expanded_grid_points(normalize_path(route)))
    for point in points or []:
        blocked.add(to_point(point))
    return blocked


def assert_no_blockage(route: Sequence[Point], blocked_points: set[Point] | None) -> None:
    if not blocked_points:
        return
    touched = sorted(set(expanded_grid_points(route)) & blocked_points)
    if touched:
        raise AssertionError(f"Route touches blocked grid points: {touched[:8]}")


def split_positive(total: int, parts: int) -> list[int]:
    if parts <= 0:
        raise ValueError(f"parts must be positive, got {parts}")
    if total < parts:
        raise ValueError(f"Cannot split {total} into {parts} positive pieces")
    base, extra = divmod(total, parts)
    return [base + (1 if idx < extra else 0) for idx in range(parts)]


def splice_path(route: Sequence[Point], start_idx: int, end_idx: int, replacement: Sequence[Point]) -> list[Point]:
    if route[start_idx] != replacement[0] or route[end_idx] != replacement[-1]:
        raise ValueError("Replacement endpoints do not match source path endpoints.")
    return simplify_collinear(list(route[:start_idx]) + list(replacement) + list(route[end_idx + 1 :]))


def validate_insertion(
    before: Sequence[Point],
    after: Sequence[Point],
    operator: str,
    expected_added_bends: int | None = None,
    blocked_points: set[Point] | None = None,
) -> tuple[int, int, int, int]:
    assert_simple_polyline(before)
    assert_simple_polyline(after)
    assert_no_blockage(after, blocked_points)
    before_length = manhattan_length(before)
    after_length = manhattan_length(after)
    before_bends = bend_count(before)
    after_bends = bend_count(after)
    if before_length != after_length:
        raise AssertionError(f"{operator} changed length: {before_length} -> {after_length}")
    if after_bends <= before_bends:
        raise AssertionError(f"{operator} did not add bends: {before_bends} -> {after_bends}")
    if expected_added_bends is not None and after_bends - before_bends != expected_added_bends:
        raise AssertionError(
            f"{operator} added {after_bends - before_bends} bends, expected {expected_added_bends}"
        )
    return before_length, after_length, before_bends, after_bends


def detect_simple_spiral_at(route: Sequence[Point], start_idx: int) -> SpiralMatch | None:
    if start_idx + 6 >= len(route):
        return None
    p = list(route[start_idx : start_idx + 7])
    source = p[0]
    target = p[6]
    try:
        side = unit_vector(p[0], p[1])
        along = unit_vector(p[1], p[2])
    except ValueError:
        return None
    if dot(side, along) != 0:
        return None

    detour_length = manhattan_distance(p[1], p[2]) + 1
    if detour_length < 5:
        return None
    target_vector = vector(source, target)
    if target_vector[0] and target_vector[1]:
        return None
    target_distance = dot(target_vector, along)
    if target_distance < detour_length - 1:
        return None

    expected = [
        source,
        add(source, scale(side, 2)),
        add(source, scale(along, detour_length - 1), scale(side, 2)),
        add(source, scale(along, detour_length - 1), side),
        add(source, along, side),
        add(source, along),
        add(source, scale(along, target_distance)),
    ]
    if p != expected:
        return None
    return SpiralMatch(
        start_idx=start_idx,
        end_idx=start_idx + 6,
        source=source,
        target=target,
        along=along,
        side=side,
        detour_length=detour_length,
        target_distance=target_distance,
    )


def find_simple_spirals(route: Sequence[Iterable[int]]) -> list[SpiralMatch]:
    path = simplify_collinear(normalize_path(route))
    return [match for idx in range(len(path)) if (match := detect_simple_spiral_at(path, idx)) is not None]


ZIGZAG_HEIGHT = 2
ZIGZAG_MIN_BASE_SPAN = 2
ZIGZAG_TAIL_PITCH = 2


def max_zigzag_u_blocks(match: SpiralMatch) -> int:
    far = match.detour_length - 1
    if match.target_distance < far + 1:
        return 0
    return max(0, (far - ZIGZAG_MIN_BASE_SPAN) // ZIGZAG_TAIL_PITCH)


def build_zigzag_spiral(match: SpiralMatch, u_blocks: int = 1) -> list[Point]:
    far = match.detour_length - 1
    max_blocks = max_zigzag_u_blocks(match)
    if u_blocks < 1 or u_blocks > max_blocks:
        raise ValueError(f"u_blocks must be in [1, {max_blocks}], got {u_blocks}")
    base_span = far - ZIGZAG_TAIL_PITCH * u_blocks
    if base_span < ZIGZAG_MIN_BASE_SPAN:
        raise ValueError(f"Need base spiral span >= 2, got {base_span}")

    moves: list[tuple[Vector, int]] = [
        (match.side, ZIGZAG_HEIGHT),
        (match.along, base_span),
        (match.side, -1),
        (match.along, -(base_span - 1)),
        (match.side, -1),
        (match.along, base_span),
    ]
    for _ in range(u_blocks):
        moves.extend(
            [
                (match.side, ZIGZAG_HEIGHT),
                (match.along, 1),
                (match.side, -ZIGZAG_HEIGHT),
                (match.along, 1),
            ]
        )
    current_along_offset = base_span + 1 + ZIGZAG_TAIL_PITCH * u_blocks
    moves.append((match.along, match.target_distance - current_along_offset))
    return trace_moves(match.source, moves)


def zigzag_spiral_candidates(
    route: Sequence[Iterable[int]],
    start_idx: int | None = None,
    expected_added_bends: int | None = None,
    blocked_points: set[Point] | None = None,
) -> list[BendInsertionResult]:
    before = simplify_collinear(normalize_path(route))
    assert_manhattan(before)
    matches = find_simple_spirals(before)
    if start_idx is not None:
        matches = [match for match in matches if match.start_idx == start_idx]
    if not matches:
        return []

    candidates: list[BendInsertionResult] = []
    for match in matches:
        for u_blocks in range(1, max_zigzag_u_blocks(match) + 1):
            try:
                local_before = before[match.start_idx : match.end_idx + 1]
                local_after = build_zigzag_spiral(match, u_blocks)
                assert_same_grid_occupancy(local_before, local_after)
                after = splice_path(before, match.start_idx, match.end_idx, local_after)
                before_length, after_length, before_bends, after_bends = validate_insertion(
                    before, after, "zigzag_spiral_insert", expected_added_bends, blocked_points
                )
                candidates.append(
                    BendInsertionResult(
                        path=after,
                        operator="zigzag_spiral_insert",
                        before_length=before_length,
                        after_length=after_length,
                        before_bends=before_bends,
                        after_bends=after_bends,
                        replaced_range=(match.start_idx, match.end_idx),
                        local_before=local_before,
                        local_after=local_after,
                        metadata={
                            "expected_added_bends": expected_added_bends,
                            "actual_added_bends": after_bends - before_bends,
                            "u_blocks": u_blocks,
                            "detour_length": match.detour_length,
                            "target_distance": match.target_distance,
                            "grid_occupancy_preserved": True,
                            "scan_order": "source_to_target",
                        },
                    )
                )
            except Exception:  # noqa: BLE001
                continue
    return sorted(candidates, key=lambda item: (item.replaced_range[0], item.added_bends))


def zigzag_spiral_insert(
    route: Sequence[Iterable[int]],
    start_idx: int | None = None,
    expected_added_bends: int | None = None,
    blocked_points: set[Point] | None = None,
) -> BendInsertionResult:
    candidates = zigzag_spiral_candidates(route, start_idx, expected_added_bends, blocked_points)
    if not candidates:
        raise ValueError("No feasible U-shape zigzag insertion found.")
    return max(candidates, key=lambda item: (item.added_bends, -item.replaced_range[0], -item.replaced_range[1]))


def build_monotonic_staircase(p0: Point, corner: Point, p2: Point, extra_bends: int = 2) -> list[Point]:
    if extra_bends <= 0 or extra_bends % 2:
        raise ValueError("monotonic staircase insertion adds bends in multiples of 2.")
    steps = extra_bends // 2
    axis_in = unit_vector(p0, corner)
    axis_out = unit_vector(corner, p2)
    if dot(axis_in, axis_out) != 0:
        raise ValueError("Expected an L corner with perpendicular incident segments.")
    len_in = manhattan_distance(p0, corner)
    len_out = manhattan_distance(corner, p2)
    if steps > min(len_in, len_out):
        raise ValueError(f"Need both L legs >= {steps}, got {(len_in, len_out)}")

    anchor = add(corner, scale(neg(axis_in), steps))
    points: list[Point] = [p0]
    if anchor != p0:
        points.append(anchor)
    cur = anchor
    for _ in range(steps):
        cur = add(cur, axis_out)
        points.append(cur)
        cur = add(cur, axis_in)
        points.append(cur)
    if cur != p2:
        points.append(p2)
    return simplify_collinear(points)


def monotonic_staircase_insert(
    route: Sequence[Iterable[int]],
    corner_idx: int,
    extra_bends: int = 2,
    blocked_points: set[Point] | None = None,
) -> BendInsertionResult:
    before = simplify_collinear(normalize_path(route))
    assert_manhattan(before)
    if not (0 < corner_idx < len(before) - 1):
        raise ValueError(f"corner_idx must be an interior index, got {corner_idx}")
    local_after = build_monotonic_staircase(before[corner_idx - 1], before[corner_idx], before[corner_idx + 1], extra_bends)
    after = splice_path(before, corner_idx - 1, corner_idx + 1, local_after)
    before_length, after_length, before_bends, after_bends = validate_insertion(
        before, after, "monotonic_staircase_insert", extra_bends, blocked_points
    )
    return BendInsertionResult(
        path=after,
        operator="monotonic_staircase_insert",
        before_length=before_length,
        after_length=after_length,
        before_bends=before_bends,
        after_bends=after_bends,
        replaced_range=(corner_idx - 1, corner_idx + 1),
        local_before=before[corner_idx - 1 : corner_idx + 2],
        local_after=local_after,
        metadata={
            "extra_bends_requested": extra_bends,
            "steps": extra_bends // 2,
            "scan_order": "source_to_target",
        },
    )


def find_zigzag_candidates(
    route: Sequence[Iterable[int]],
    remaining_bends: int,
    blocked_points: set[Point] | None = None,
) -> list[BendInsertionResult]:
    candidates: list[BendInsertionResult] = []
    for candidate in zigzag_spiral_candidates(route, blocked_points=blocked_points):
        if 0 < candidate.added_bends <= remaining_bends:
            candidates.append(candidate)
    return sorted(candidates, key=lambda item: (item.replaced_range[0], -item.added_bends))


def find_monotonic_candidates(
    route: Sequence[Iterable[int]],
    remaining_bends: int,
    blocked_points: set[Point] | None = None,
) -> list[BendInsertionResult]:
    path = simplify_collinear(normalize_path(route))
    candidates: list[BendInsertionResult] = []
    max_extra = remaining_bends - (remaining_bends % 2)
    for corner_idx in range(1, len(path) - 1):
        for extra_bends in range(2, max_extra + 1, 2):
            try:
                candidates.append(monotonic_staircase_insert(path, corner_idx, extra_bends, blocked_points))
            except (AssertionError, ValueError):
                continue
    return sorted(candidates, key=lambda item: (item.replaced_range[0], -item.added_bends))


def insert_bends_to_target(
    route: Sequence[Iterable[int]],
    target_bends: int,
    allow_partial: bool = False,
    blocked_points: set[Point] | None = None,
) -> list[BendInsertionResult]:
    current = simplify_collinear(normalize_path(route))
    history: list[BendInsertionResult] = []
    while bend_count(current) < target_bends:
        remaining = target_bends - bend_count(current)
        candidates = find_zigzag_candidates(current, remaining, blocked_points)
        if not candidates:
            candidates = find_monotonic_candidates(current, remaining, blocked_points)
        if not candidates:
            if allow_partial:
                return history
            raise ValueError(f"Cannot reach target bends {target_bends} from {bend_count(current)}")
        chosen = max(candidates, key=lambda item: (item.added_bends, -item.replaced_range[0], -item.replaced_range[1]))
        history.append(chosen)
        current = chosen.path
    if bend_count(current) != target_bends:
        raise AssertionError(f"Overshot target bends: {bend_count(current)} != {target_bends}")
    return history
