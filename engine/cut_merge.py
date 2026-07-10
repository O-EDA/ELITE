import numpy as np
def check_correspondence(elements, order):
    if len(elements) != len(order):
        return False

    sorted_elements = sorted(elements)
    ordered_elements = [elements[i] for i in order]

    return sorted_elements == ordered_elements


def find_intersections(paths, line):
    axis, value = line
    intersections = []

    for path in paths:
        path_intersections = []
        for i in range(len(path) - 1):
            (x1, y1), (x2, y2) = path[i], path[i + 1]

            if axis == 'x':  # Vertical line
                if (x1 - value) * (x2 - value) <= 0:
                    if x1 != x2:
                        y_intersection = y1 + (value - x1) * (y2 - y1) / (x2 - x1)
                        if min(y1, y2) <= y_intersection <= max(y1, y2):
                            path_intersections.append((value, y_intersection))
            elif axis == 'y':  # Horizontal line
                if (y1 - value) * (y2 - value) <= 0:
                    if y1 != y2:
                        x_intersection = x1 + (value - y1) * (x2 - x1) / (y2 - y1)
                        if min(x1, x2) <= x_intersection <= max(x1, x2):
                            path_intersections.append((x_intersection, value))
        if len(path_intersections) > 2 or len(path_intersections) == 0:
            return []
        if len(path_intersections) == 2:
            path_intersections = [path_intersections[0]]
        intersections.append(path_intersections)

    return intersections


def find_cut_position(paths, bounding_box):
    min_x, max_x, min_y, max_y = bounding_box
    leave_part = None
    paths_ends = [path[-1] for path in paths]
    variance_x = (sum((sum([point[0] for point in paths_ends]) / len(paths_ends) - point[0]) ** 2 for point in paths_ends)
                  / len(paths_ends))
    variance_y = (sum((sum([point[1] for point in paths_ends]) / len(paths_ends) - point[1]) ** 2 for point in paths_ends)
                  / len(paths_ends))
    if variance_x > variance_y:
        direction = 'horizontal'
        if (abs(sum([point[1] for point in paths_ends]) / len(paths_ends) - min_y) >
                abs(sum([point[1] for point in paths_ends]) / len(paths_ends) - max_y)):
            leave_part = 'top'
        elif (abs(sum([point[1] for point in paths_ends]) / len(paths_ends) - min_y) <
                abs(sum([point[1] for point in paths_ends]) / len(paths_ends) - max_y)):
            leave_part = 'bottom'
    else:
        direction = 'vertical'
        if (abs(sum([point[0] for point in paths_ends]) / len(paths_ends) - min_x) >
                abs(sum([point[0] for point in paths_ends]) / len(paths_ends) - max_x)):
            leave_part = 'right'
        elif (abs(sum([point[0] for point in paths_ends]) / len(paths_ends) - min_x) <
                abs(sum([point[0] for point in paths_ends]) / len(paths_ends) - max_x)):
            leave_part = 'left'
    if direction == 'horizontal':
        paths.sort(key=lambda path: path[-1][0])
        order = [i for i in range(len(paths))]
        if leave_part == 'top':
            cut_position = min_y
            for pos in range(min_y, max_y + 1):
                intersects = find_intersections(paths, ('y', pos))
                if intersects:
                    if check_correspondence(intersects, order):
                        cut_position = pos
                        return cut_position, direction, leave_part
        elif leave_part == 'bottom':
            cut_position = max_y
            for pos in range(max_y, min_y - 1, -1):
                intersects = find_intersections(paths, ('y', pos))
                if intersects:
                    if check_correspondence(intersects, order):
                        cut_position = pos
                        return cut_position, direction, leave_part
    else:
        paths.sort(key=lambda path: path[-1][1])
        order = [i for i in range(len(paths))]
        if leave_part == 'right':
            cut_position = min_x
            for pos in range(min_x, max_x + 1):
                intersects = find_intersections(paths, ('x', pos))
                if intersects:
                    if check_correspondence(intersects, order):
                        cut_position = pos
                        return cut_position, direction, leave_part
        elif leave_part == 'left':
            cut_position = max_x
            for pos in range(max_x, min_x - 1, -1):
                intersects = find_intersections(paths, ('x', pos))
                if intersects:
                    if check_correspondence(intersects, order):
                        cut_position = pos
                        return cut_position, direction, leave_part

    return 0

def line_intersection(line1, line2):
    """Returns the intersection point of two lines (if exists) given by their endpoints."""
    (x1, y1), (x2, y2) = line1
    (x3, y3), (x4, y4) = line2

    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if denom == 0:
        return None  # Lines are parallel

    intersect_x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
    intersect_y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom

    if (min(x1, x2) <= intersect_x <= max(x1, x2) and min(y1, y2) <= intersect_y <= max(y1, y2) and
            min(x3, x4) <= intersect_x <= max(x3, x4) and min(y3, y4) <= intersect_y <= max(y3, y4)):
        return (int(intersect_x), int(intersect_y))
    else:
        return None  # Intersection point is not within the line segments


def is_point_on_segment(point, segment):
    """Check if a point is exactly on a given line segment."""
    (x, y) = point
    (x1, y1), (x2, y2) = segment
    cross_product = (y - y1) * (x2 - x1) - (x - x1) * (y2 - y1)
    if abs(cross_product) != 0:
        return False

    dot_product = (x - x1) * (x2 - x1) + (y - y1) * (y2 - y1)
    if dot_product < 0:
        return False

    squared_length = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)
    if dot_product > squared_length:
        return False

    return True


def manhattan_distance(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])


def insert_points_manhattan(path, p1, p2, flag_start, flag_end):
    print(path, p1, p2, flag_start, flag_end)
    def insert_point(path, point):
        min_increase = float('inf')
        best_idx = -1

        for i in range(len(path)):
            p1 = path[i]
            p2 = path[(i + 1) % len(path)]
            if i == 0:
                increase = manhattan_distance(p1, point)
            else:
                increase = manhattan_distance(p1, point) + manhattan_distance(point, p2) - manhattan_distance(p1, p2)
            if increase < min_increase:
                min_increase = increase
                best_idx = i if i == 0 else i+1

        return np.insert(path, best_idx, [point], axis=0)

    path = np.array(path)
    if flag_start:
        path = insert_point(path, p1)
    if flag_end:
        path = insert_point(path, p2)
    return path.tolist()


def find_intersection_points(path_coords, point, direction):
    x, y = point
    if direction == 'horizontal':
        line_coords = [(min(x for x, y in path_coords) - 1, y), (max(x for x, y in path_coords) + 1, y)]
    elif direction == 'vertical':
        line_coords = [(x, min(y for x, y in path_coords) - 1), (x, max(y for x, y in path_coords) + 1)]
    else:
        raise ValueError("Direction must be either 'horizontal' or 'vertical'.")

    intersections = []
    for i in range(len(path_coords) - 1):
        path_segment = (path_coords[i], path_coords[i + 1])
        intersection = line_intersection(path_segment, line_coords)
        if intersection:
            intersections.append(intersection)
        if is_point_on_segment(point, path_segment):
            intersections.append(point)
            break  # If the point is on the segment, we don't need to check further segments
    return intersections


def _dedupe_consecutive(path):
    return [path[i] for i in range(len(path)) if i == 0 or path[i] != path[i - 1]]


def _simplify_manhattan(path):
    path = _dedupe_consecutive([tuple(point) for point in path])
    if len(path) <= 2:
        return [list(point) for point in path]
    simplified = [path[0]]
    for point in path[1:]:
        if len(simplified) >= 2:
            a = simplified[-2]
            b = simplified[-1]
            if (a[0] == b[0] == point[0]) or (a[1] == b[1] == point[1]):
                simplified[-1] = point
                continue
        simplified.append(point)
    return [list(point) for point in simplified]


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


def _nearest_point_on_segment(point, segment):
    (x, y) = point
    (x1, y1), (x2, y2) = segment
    if x1 == x2:
        return (x1, _clamp(y, min(y1, y2), max(y1, y2)))
    if y1 == y2:
        return (_clamp(x, min(x1, x2), max(x1, x2)), y1)
    return min((segment[0], segment[1]), key=lambda candidate: manhattan_distance(candidate, point))


def _nearest_point_on_path(path, point):
    point = tuple(point)
    best_point = tuple(path[0])
    best_distance = manhattan_distance(best_point, point)
    for start, end in zip(path, path[1:]):
        candidate = _nearest_point_on_segment(point, (tuple(start), tuple(end)))
        distance = manhattan_distance(candidate, point)
        if distance < best_distance:
            best_point = candidate
            best_distance = distance
    return best_point


def _insert_anchor(path, anchor):
    anchor = tuple(anchor)
    path = [tuple(point) for point in path]
    if anchor in path:
        return path
    for index, (start, end) in enumerate(zip(path, path[1:])):
        if is_point_on_segment(anchor, (start, end)):
            return path[: index + 1] + [anchor] + path[index + 1 :]
    return path + [anchor]


def _arc_forward(path, start_idx, end_idx):
    if start_idx <= end_idx:
        return path[start_idx : end_idx + 1]
    return path[start_idx:-1] + path[: end_idx + 1]


def _connector(start, end, direction):
    start = tuple(start)
    end = tuple(end)
    if start == end:
        return [start]
    if start[0] == end[0] or start[1] == end[1]:
        return [start, end]
    elbow = (start[0], end[1]) if direction == "horizontal" else (end[0], start[1])
    return [start, elbow, end]


def _attach_requested_endpoints(arc, point_start, point_end, anchor_start, anchor_end, direction):
    start_connector = _connector(tuple(point_start), anchor_start, direction)
    end_connector = _connector(anchor_end, tuple(point_end), direction)
    return _simplify_manhattan(start_connector + list(arc)[1:] + end_connector[1:])



def cut_path(path_coords, point_start, point_end, direction):
    contour = [tuple(point) for point in path_coords]
    if contour[0] != contour[-1]:
        contour.append(contour[0])

    point_start = tuple(point_start)
    point_end = tuple(point_end)
    anchor_start = point_start if point_start in contour else _nearest_point_on_path(contour, point_start)
    contour = _insert_anchor(contour, anchor_start)
    anchor_end = point_end if point_end in contour else _nearest_point_on_path(contour, point_end)
    contour = _insert_anchor(contour, anchor_end)

    start_idx = contour.index(anchor_start)
    end_idx = contour.index(anchor_end)
    arc1 = _arc_forward(contour, start_idx, end_idx)
    arc2 = list(reversed(_arc_forward(contour, end_idx, start_idx)))
    path1 = _attach_requested_endpoints(arc1, point_start, point_end, anchor_start, anchor_end, direction)
    path2 = _attach_requested_endpoints(arc2, point_start, point_end, anchor_start, anchor_end, direction)
    return path1, path2


def merge_paths(paths1, paths2):
    for i in range(len(paths1)):
        if paths1[i][0][0] > paths1[i][-1][0]:
            paths1[i] = paths1[i][::-1]
    for j in range(len(paths2)):
        if paths2[j][0][0] > paths2[j][-1][0]:
            paths2[j] = paths2[j][::-1]
    paths1 = sorted(paths1, key=lambda x: x[-1][1])
    paths2 = sorted(paths2, key=lambda x: x[-1][1])
    merged_paths = []

    for i in range(len(paths1)):
        print(paths1[i][-1], paths2[i][0])
        if paths1[i][-1] != list(paths2[i][0]):
            if paths1[i][-1] == list(paths2[i][-1]):
                paths2[i] = paths2[i][::-1]
            else:
                return "Paths cannot be connected"

        merged_path = paths1[i] + [list(point) for point in
                                 paths2[i][1:]]  # Skip the first element of paths2 to avoid duplication
        merged_paths.append(merged_path)

    return merged_paths





if __name__ == '__main__':
    path_coords = [(41, 20), (52, 20), (52, 22), (90, 22), (90, 28), (91, 28), (91, 30), (92, 30), (92, 77), (41, 77), (41, 20)]
    point_start = (41, 19)
    point_end = (92, 34)
    direction = 'horizontal'
    path_cut1, path_cut2 = cut_path(path_coords, point_start, point_end, direction)
    print(path_cut1)
    print(path_cut2)



