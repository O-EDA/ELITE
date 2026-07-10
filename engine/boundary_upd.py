import numpy as np
from scipy.spatial import distance
import time
from scipy.spatial import KDTree

def bresenham(x0, y0, x1, y1):
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = err * 2
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return points


def set_obstacle(grid, obstacle, boundary_area):
    x_min, x_max, y_min, y_max = boundary_area
    for i in range(len(obstacle) - 1):
        x0, y0 = obstacle[i][0] - x_min, obstacle[i][1] - y_min
        x1, y1 = obstacle[i + 1][0] - x_min, obstacle[i + 1][1] - y_min
        x0, y0, x1, y1 = round(x0, 1), round(y0, 1), round(x1, 1), round(y1, 1)
        for x, y in bresenham(x0, y0, x1, y1):
            grid[int(y), int(x)] = 1
    return grid


def remove_obstacle(grid, obstacle, boundary_area):
    x_min, x_max, y_min, y_max = boundary_area
    for i in range(len(obstacle) - 1):
        x0, y0 = obstacle[i][0] - x_min, obstacle[i][1] - y_min
        x1, y1 = obstacle[i + 1][0] - x_min, obstacle[i + 1][1] - y_min
        for x, y in bresenham(x0, y0, x1, y1):
            grid[int(y), int(x)] = 0
    return grid

def initialize_grid(boundary, obstacle, boundary_area):
    x_min, x_max, y_min, y_max = boundary_area
    width = x_max - x_min + 1
    height = y_max - y_min + 1
    grid = np.zeros((height, width))

    for i in range(len(boundary) - 1):
        x0, y0 = boundary[i][0] - x_min, boundary[i][1] - y_min
        x1, y1 = boundary[i + 1][0] - x_min, boundary[i + 1][1] - y_min
        x0, y0, x1, y1 = round(x0, 1), round(y0, 1), round(x1, 1), round(y1, 1)
        for x, y in bresenham(x0, y0, x1, y1):
            grid[y, x] = 1  # Note: here we use y as row and x as column
    if len(obstacle) > 0:
        for obs in obstacle:
            set_obstacle(grid, obs, boundary_area)
    else:
        set_obstacle(grid, obstacle, boundary_area)
    return grid


def update_grid(grid, paths, boundary_area):
    x_min, x_max, y_min, y_max = boundary_area
    for path in paths:
        for i in range(len(path) - 1):
            x0, y0 = path[i]
            x1, y1 = path[i + 1]
            for x, y in bresenham(x0, y0, x1, y1):
                grid[y - y_min, x - x_min] = 1  # Note: here we use y as row and x as column
    return grid


def find_internal_contour_points(arr):
    rows, cols = arr.shape
    contour_points = []

    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

    for i in range(rows):
        for j in range(cols):
            if arr[i, j] == 0:
                is_contour = False
                top = arr[i-1, j] if i > 0 else 0
                bottom = arr[i+1, j] if i < rows-1 else 0
                left = arr[i, j-1] if j > 0 else 0
                right = arr[i, j+1] if j < cols-1 else 0

                if ((top == 1 and bottom == 1) or (left == 1 and right == 1)):
                    continue

                for direction in directions:
                    ni, nj = i + direction[0], j + direction[1]
                    if 0 <= ni < rows and 0 <= nj < cols:
                        if arr[ni, nj] == 1:
                            is_contour = True
                            break

                if is_contour:
                    contour_points.append((j, i))

    return contour_points













def sort_contour_points(contour_points):
    point_set = set(contour_points)

    DIRS = [(1, 0), (0, -1), (-1, 0), (0, 1)]  # x, y order
    LEFT_TURN = [1, 2, 3, 0]  # if facing DIRS[i], turn left = DIRS[LEFT_TURN[i]]
    RIGHT_TURN = [3, 0, 1, 2]

    start = min(point_set, key=lambda p: (p[1], p[0]))
    curr = start
    path = [curr]
    visited = set([curr])

    facing = 0

    while True:
        turned = False
        for i in [LEFT_TURN[facing], facing, RIGHT_TURN[facing], (facing + 2) % 4]:
            dx, dy = DIRS[i]
            nxt = (curr[0] + dx, curr[1] + dy)
            if nxt in point_set and nxt not in visited:
                curr = nxt
                path.append(curr)
                visited.add(curr)
                facing = i
                turned = True
                break

        if not turned:
            break  # dead-end or closed loop

        if curr == start and len(path) > 4:
            break  # completed loop

    return path


def simplify_contour_points(points):
    if not points:
        return []
    points = sort_contour_points(points)
    print("sorted points",points)
    points.append(points[0])
    simplified_points = [points[0]]  # first point as the key point
    for i in range(1, len(points) - 1):
        prev_point = points[i - 1]
        curr_point = points[i]
        next_point = points[i + 1]

        direction_prev = (curr_point[0] - prev_point[0], curr_point[1] - prev_point[1])
        direction_next = (next_point[0] - curr_point[0], next_point[1] - curr_point[1])

        if direction_prev != direction_next:
            simplified_points.append(curr_point)

    return simplified_points


def  recover_path(simplified_path, boundary_area):
    x_min, x_max, y_min, y_max = boundary_area
    return [(point[0] + x_min, point[1] + y_min) for point in simplified_path]

if __name__ == '__main__':
    points = [(13, 14), (14, 14), (15, 14), (16, 14), (17, 14), (18, 14), (13, 15), (18, 15), (13, 16), (18, 16), (13, 17), (18, 17), (12, 18), (13, 18), (18, 18), (12, 19), (18, 19), (12, 20), (18, 20), (10, 21), (11, 21), (12, 21), (18, 21), (9, 22), (10, 22), (18, 22), (9, 23), (18, 23), (9, 24), (18, 24), (8, 25), (9, 25), (18, 25), (8, 26), (18, 26), (8, 27), (18, 27), (8, 28), (18, 28), (6, 29), (7, 29), (8, 29), (18, 29), (6, 30), (18, 30), (6, 31), (18, 31), (5, 32), (6, 32), (18, 32), (5, 33), (18, 33), (5, 34), (18, 34), (5, 35), (18, 35), (4, 36), (5, 36), (18, 36), (4, 37), (18, 37), (4, 38), (18, 38), (4, 39), (18, 39), (3, 40), (4, 40), (18, 40), (3, 41), (18, 41), (3, 42), (18, 42), (2, 43), (3, 43), (18, 43), (2, 44), (18, 44), (2, 45), (18, 45), (2, 46), (18, 46), (1, 47), (2, 47), (18, 47), (1, 48), (18, 48), (1, 49), (18, 49), (1, 50), (18, 50), (1, 51), (18, 51), (1, 52), (2, 52), (3, 52), (4, 52), (5, 52), (6, 52), (7, 52), (8, 52), (9, 52), (18, 52), (9, 53), (18, 53), (9, 54), (18, 54), (9, 55), (18, 55), (9, 56), (18, 56), (9, 57), (18, 57), (9, 58), (18, 58), (9, 59), (18, 59), (9, 60), (18, 60), (9, 61), (18, 61), (9, 62), (18, 62), (9, 63), (18, 63), (9, 64), (18, 64), (9, 65), (18, 65), (9, 66), (18, 66), (9, 67), (18, 67), (9, 68), (18, 68), (1, 69), (2, 69), (9, 69), (18, 69), (1, 70), (2, 70), (9, 70), (18, 70), (1, 71), (2, 71), (9, 71), (18, 71), (1, 72), (2, 72), (9, 72), (18, 72), (1, 73), (2, 73), (9, 73), (18, 73), (1, 74), (2, 74), (9, 74), (18, 74), (1, 75), (2, 75), (7, 75), (8, 75), (9, 75), (18, 75), (1, 76), (2, 76), (7, 76), (18, 76), (1, 77), (2, 77), (3, 77), (4, 77), (7, 77), (18, 77), (1, 78), (2, 78), (4, 78), (7, 78), (18, 78), (2, 79), (4, 79), (7, 79), (18, 79), (2, 80), (4, 80), (7, 80), (18, 80), (2, 81), (3, 81), (4, 81), (7, 81), (18, 81), (3, 82), (4, 82), (7, 82), (18, 82), (3, 83), (4, 83), (5, 83), (6, 83), (7, 83), (18, 83), (3, 84), (4, 84), (18, 84), (4, 85), (18, 85), (4, 86), (5, 86), (18, 86), (5, 87), (18, 87), (5, 88), (18, 88), (5, 89), (6, 89), (18, 89), (6, 90), (7, 90), (18, 90), (7, 91), (18, 91), (7, 92), (18, 92), (7, 93), (8, 93), (18, 93), (8, 94), (9, 94), (18, 94), (9, 95), (10, 95), (18, 95), (10, 96), (18, 96), (10, 97), (11, 97), (12, 97), (18, 97), (12, 98), (13, 98), (14, 98), (15, 98), (16, 98), (17, 98), (18, 98)]
    sorted_points = sort_contour_points(points)



    print("sorted points", sorted_points)
