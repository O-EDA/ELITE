import matplotlib.pyplot as plt
from collections import deque


def visualize_path(path):
    x_coords = [point[0] for point in path]
    y_coords = [point[1] for point in path]

    plt.figure(figsize=(6, 6))
    plt.plot(x_coords, y_coords, marker="o", linestyle="-", color="b")
    plt.title("Path Visualization")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.show()


def delete_consecutive(path):
    """Remove consecutive identical points."""
    if not path:
        return []
    simplified_path = [path[0]]
    for i in range(1, len(path)):
        if path[i] != path[i - 1]:
            simplified_path.append(path[i])
    return simplified_path


def simplify_path(path):
    if not path:
        return []

    simplified_path = delete_consecutive(path)

    final_path = [simplified_path[0]]
    for i in range(1, len(simplified_path) - 1):
        x1, y1 = simplified_path[i - 1]
        x2, y2 = simplified_path[i]
        x3, y3 = simplified_path[i + 1]

        dx1, dy1 = x2 - x1, y2 - y1
        dx2, dy2 = x3 - x2, y3 - y2
        if (dx1, dy1) != (dx2, dy2):
            final_path.append((x2, y2))
    final_path.append(simplified_path[-1])
    return final_path


def is_rect_pattern(detection_points):
    p0, p1 = detection_points[0], detection_points[1]
    p2, p3 = detection_points[2], detection_points[3]
    v1 = [p1[0] - p0[0], p1[1] - p0[1]]
    v2 = [p3[0] - p2[0], p3[1] - p2[1]]
    dot = v1[0] * v2[0] + v1[1] * v2[1]
    len_0 = abs(v1[0]) + abs(v1[1])
    len_1 = abs(v2[0]) + abs(v2[1])
    min_len = min(len_0, len_1)
    return dot < 0, min_len


def calculate_length(path, length=0):
    for i in range(len(path) - 1):
        length += abs(path[i][0] - path[i + 1][0]) + abs(path[i][1] - path[i + 1][1])
    return length


def get_direction(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    if x2 < x1 and y2 == y1:
        return 0
    if x2 == x1 and y2 < y1:
        return 1
    if x2 > x1 and y2 == y1:
        return 2
    if x2 == x1 and y2 > y1:
        return 3
    raise ValueError("p1 and p2 are not in a Manhattan-adjacent direction")


def is_collinear(p1, p2, p3, tol=1e-6):
    """Return True when three points are collinear."""
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    return abs((x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2)) < tol


def reduce_length(ori_path, target_length, ori_path_length=None):
    if ori_path_length is None:
        ori_path_length = calculate_length(ori_path)
    length_diff = ori_path_length - target_length
    print(length_diff)
    if length_diff <= 0:
        return False, ori_path, ori_path_length

    detection_points = deque([None, None, None, None], maxlen=4)
    for i, point in enumerate(ori_path):
        detection_points.append(point)
        if None in detection_points:
            continue

        result, min_len = is_rect_pattern(detection_points)
        if not result:
            continue

        max_len_reduction = 2 * min_len
        if length_diff > max_len_reduction:
            len_reduction = (max_len_reduction + 1) // 2
        else:
            len_reduction = (length_diff + 1) // 2

        x_offset = 0
        y_offset = 0
        direction = get_direction(ori_path[i - 3], ori_path[i - 2])
        if direction == 0:
            x_offset = -len_reduction
        elif direction == 1:
            y_offset = len_reduction
        elif direction == 2:
            x_offset = len_reduction
        elif direction == 3:
            y_offset = -len_reduction

        ori_path[i - 1] = (ori_path[i - 1][0] + x_offset, ori_path[i - 1][1] + y_offset)
        ori_path[i - 2] = (ori_path[i - 2][0] + x_offset, ori_path[i - 2][1] + y_offset)
        if ori_path[i] == ori_path[i - 1]:
            del ori_path[i - 1]
        if i >= 3 and ori_path[i - 2] == ori_path[i - 3]:
            del ori_path[i - 2]
        ori_path_length -= len_reduction * 2
        return True, ori_path, ori_path_length

    return False, ori_path, ori_path_length


if __name__ == "__main__":
    ori_path = [
        (0, 100), (10, 100), (10, 150), (50, 150), (50, 140), (60, 140),
        (60, 130), (70, 130), (70, 120), (80, 120), (80, 110), (90, 110),
        (90, 100), (100, 100),
    ]
    target_length = 150

    visualize_path(ori_path)
    ori_path_length = None
    while True:
        is_reduced, ori_path, ori_path_length = reduce_length(ori_path, target_length, ori_path_length)
        ori_path = simplify_path(ori_path)
        leng = calculate_length(ori_path)
        print(leng)
        visualize_path(ori_path)
        if not is_reduced:
            break
