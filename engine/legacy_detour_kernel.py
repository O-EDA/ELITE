import matplotlib.pyplot as plt
from boundary_upd import *
from cut_merge import cut_path
import time


def adj_ava_detour(grid, point_start, point_end, direct):
    x_min, x_max, y_min, y_max = boundary_area
    offset_ava_detour = 0
    if direct == 'right':
        start_x, start_y = point_start
        direction_ver = int((point_end[1] - point_start[1]) / abs(point_end[1] - point_start[1]))
        end_x, end_y = point_end
        if grid[end_y - y_min, end_x - x_min + 1] == 0 and grid[end_y - y_min, end_x - x_min + 2] == 0:
            while grid[end_y - y_min + offset_ava_detour * direction_ver, end_x - x_min + 1] == 0 and \
                    grid[end_y - y_min + offset_ava_detour * direction_ver, end_x - x_min + 2] == 0:
                offset_ava_detour += 1
            offset_ava_detour -= 1
        else:
            while grid[end_y - y_min + offset_ava_detour * direction_ver, end_x - x_min + 1] != 0 or \
                    grid[end_y - y_min + offset_ava_detour * direction_ver, end_x - x_min + 2] != 0:
                offset_ava_detour -= 1
    elif direct == 'left':
        start_x, start_y = point_start
        direction_ver = int((point_end[1] - point_start[1]) / abs(point_end[1] - point_start[1]))
        end_x, end_y = point_end
        if grid[end_y - y_min][end_x - x_min - 1] == 0 and grid[end_y - y_min][end_x - x_min - 2] == 0:
            while grid[end_y - y_min + offset_ava_detour * direction_ver, end_x - x_min - 1] == 0 and \
                    grid[end_y - y_min + offset_ava_detour * direction_ver, end_x - x_min - 2] == 0:
                offset_ava_detour += 1
            offset_ava_detour -= 1
        else:
            while grid[end_y - y_min + offset_ava_detour * direction_ver, end_x - x_min - 1] != 0 or \
                    grid[end_y - y_min + offset_ava_detour * direction_ver, end_x - x_min - 2] != 0:
                offset_ava_detour -= 1
    elif direct == 'up':
        start_x, start_y = point_start
        direction_hor = int((point_end[0] - point_start[0]) / abs(point_end[0] - point_start[0]))
        end_x, end_y = point_end
        if grid[end_y - y_min + 1, end_x - x_min] == 0 and grid[end_y - y_min + 2, end_x - x_min] == 0:
            while grid[end_y - y_min + 1, end_x - x_min + offset_ava_detour * direction_hor] == 0 and \
                    grid[end_y - y_min + 2, end_x - x_min + offset_ava_detour * direction_hor] == 0:
                offset_ava_detour += 1
            offset_ava_detour -= 1
        else:
            while grid[end_y - y_min + 1, end_x - x_min + offset_ava_detour * direction_hor] != 0 or \
                    grid[end_y - y_min + 2, end_x - x_min + offset_ava_detour * direction_hor] != 0:
                offset_ava_detour -= 1
    elif direct == 'down':
        start_x, start_y = point_start
        direction_hor = int((point_end[0] - point_start[0]) / abs(point_end[0] - point_start[0]))
        end_x, end_y = point_end
        if grid[end_y - y_min - 1, end_x - x_min] == 0 and grid[end_y - y_min - 2, end_x - x_min] == 0:
            while grid[end_y - y_min - 1, end_x - x_min + offset_ava_detour * direction_hor] == 0 and \
                    grid[end_y - y_min - 2, end_x - x_min + offset_ava_detour * direction_hor] == 0:
                offset_ava_detour += 1
            offset_ava_detour -= 1
        else:
            while grid[end_y - y_min + 1, end_x - x_min + offset_ava_detour * direction_hor] != 0 or \
                    grid[end_y - y_min + 2, end_x - x_min + offset_ava_detour * direction_hor] != 0:
                offset_ava_detour -= 1
    return offset_ava_detour


def detour(path, leng, ind, point_start, point_end, direct):
    if point_start[0] == point_end[0]:
        direction_ver = int((point_end[1] - point_start[1]) / abs(point_end[1] - point_start[1]))
    elif point_start[1] == point_end[1]:
        direction_hor = int((point_end[0] - point_start[0]) / abs(point_end[0] - point_start[0]))
    print(point_start)
    print(point_end)
    print(leng)
    if direct == 'right':
        path.insert(ind + 1, (point_start[0] + 2, point_start[1]))
        path.insert(ind + 2, (point_start[0] + 2, point_start[1] + (leng - 1) * direction_ver))
        path.insert(ind + 3, (point_start[0] + 1, point_start[1] + (leng - 1) * direction_ver))
        path.insert(ind + 4, (point_start[0] + 1, point_start[1] + direction_ver))
        path.insert(ind + 5, (point_start[0], point_start[1] + direction_ver))
    elif direct == 'left':
        path.insert(ind + 1, (point_start[0] - 2, point_start[1]))
        path.insert(ind + 2, (point_start[0] - 2, point_start[1] + (leng - 1) * direction_ver))
        path.insert(ind + 3, (point_start[0] - 1, point_start[1] + (leng - 1) * direction_ver))
        path.insert(ind + 4, (point_start[0] - 1, point_start[1] + direction_ver))
        path.insert(ind + 5, (point_start[0], point_start[1] + direction_ver))
    elif direct == 'up':
        path.insert(ind + 1, (point_start[0], point_start[1] + 2))
        path.insert(ind + 2, (point_start[0] + (leng - 1) * direction_hor, point_start[1] + 2))
        path.insert(ind + 3, (point_start[0] + (leng - 1) * direction_hor, point_start[1] + 1))
        path.insert(ind + 4, (point_start[0] + direction_hor, point_start[1] + 1))
        path.insert(ind + 5, (point_start[0] + direction_hor, point_start[1]))
    elif direct == 'down':
        path.insert(ind + 1, (point_start[0], point_start[1] - 2))
        path.insert(ind + 2, (point_start[0] + (leng - 1) * direction_hor, point_start[1] - 2))
        path.insert(ind + 3, (point_start[0] + (leng - 1) * direction_hor, point_start[1] - 1))
        path.insert(ind + 4, (point_start[0] + direction_hor, point_start[1] - 1))
        path.insert(ind + 5, (point_start[0] + direction_hor, point_start[1]))


from reduction import reduce_length,simplify_path

def detour_spiral(path, tar, cur_len, point_start, point_end, direct, ind, grid):
    length_dt = round((tar - cur_len) / 2)
    if length_dt == 0:
        print("length_dt equals to 0")
        return path

    if length_dt< 0:
        print("length_dt < 0")
        ori_path_length = None
        while True:
            if ori_path_length is None:
                ori_path_length=cur_len
            path=simplify_path(path)
            is_reduced, path, ori_path_length = reduce_length(path, tar, ori_path_length)
            path = simplify_path(path)
            if not is_reduced:
                break
        return path

    print(ind)
    print(point_start)
    print(path)
    if list(point_start) not in path:
        path.insert(ind + 1, list(point_start))
        ind = ind + 1
    print(ind)
    print(length_dt)
    if length_dt == 1:
        if direct == 'right':
            path.insert(ind + 1, (point_start[0] + 1, point_start[1]))
            path.insert(ind + 2, (point_start[0] + 1, point_start[1] + 1))
            path.insert(ind + 3, (point_start[0], point_start[1] + 1))
        elif direct == 'left':
            path.insert(ind + 1, (point_start[0] - 1, point_start[1]))
            path.insert(ind + 2, (point_start[0] - 1, point_start[1] + 1))
            path.insert(ind + 3, (point_start[0], point_start[1] + 1))

        return path

    if direct == 'right' or direct == 'left':
        ava_dt_len = abs(point_start[1] - point_end[1]) + 1 + adj_ava_detour(grid, point_start, point_end, direct)

        if length_dt <= ava_dt_len:
            detour(path, length_dt, ind, point_start, point_end, direct)
        elif length_dt > ava_dt_len:
            res = length_dt - ava_dt_len
            detour(path, ava_dt_len, ind, point_start, point_end, direct)
            cnt = 1
            while res > 0:
                ava_dt_len = abs(path[ind + cnt][1] - path[ind + cnt + 1][1]) + 1 + adj_ava_detour(grid, path[ind + cnt], path[ind + cnt + 1], direct)
                detour(path, min(ava_dt_len, res), ind + cnt, (path[ind + cnt][0], path[ind + cnt][1]), (path[ind + cnt + 1][0], path[ind + cnt + 1][1]), direct)
                res -= ava_dt_len
                cnt += 1

    elif direct == 'up' or direct == 'down':
        ava_dt_len = abs(point_start[0] - point_end[0]) + 1 + adj_ava_detour(grid, point_start, point_end, direct)
        if length_dt <= ava_dt_len:
            detour(path, length_dt, ind, point_start, point_end, direct)
        elif length_dt > ava_dt_len:
            res = length_dt - ava_dt_len
            detour(path, ava_dt_len, ind, point_start, point_end, direct)
            cnt = 1
            while res > 0:
                ava_dt_len = abs(path[ind + cnt][0] - path[ind + cnt + 1][0]) + 1 + adj_ava_detour(grid, path[ind + cnt], path[ind + cnt + 1], direct)
                detour(path, min(ava_dt_len, res), ind + cnt, (path[ind + cnt][0], path[ind + cnt][1]), (path[ind + cnt + 1][0], path[ind + cnt + 1][1]), direct)
                res -= ava_dt_len
                cnt += 1
    return path

def cal_path_length(path):
    length = 0
    for i in range(len(path) - 1):
        length += abs(path[i][0] - path[i + 1][0]) + abs(path[i][1] - path[i + 1][1])
    return length + 1


def visualize(paths):
    for path in paths:
        color = ['r', 'g', 'b', 'c', 'm', 'y', 'k']
        for point in path:
            plt.plot(point[0], point[1], 'ro', markersize=3)
        for i in range(len(path) - 1):
            plt.plot([path[i][0], path[i + 1][0]], [path[i][1], path[i + 1][1]], color=color[paths.index(path) % 7])
    plt.show()

def diffuse_test(idx_diffuse, grid, flag_dir):
    paths = []
    if flag_dir == 'up':
        simplified_path = simplify_contour_points(find_internal_contour_points(grid))
        recovered_path = recover_path(simplified_path, boundary_area)
        recovered_path.append(recovered_path[0])
        point1 = ori_path[idx_diffuse][-1]
        point2 = ori_path[idx_diffuse][0]

        if point1[0] < point2[0]:
            point_start = point1
            point_end = point2
        else:
            point_start = point2
            point_end = point1
        direction = 'horizontal'
        path_ccw, path_cw = cut_path(recovered_path, point_start, point_end, direction)
        paths.append(path_cw)
        print("path_cw", path_cw)
        set_obstacle(grid, path_cw, boundary_area)
    elif flag_dir == 'down':
        simplified_path = simplify_contour_points(find_internal_contour_points(grid))

        recovered_path = recover_path(simplified_path, boundary_area)
        recovered_path.append(recovered_path[0])
        point1 = ori_path[idx_diffuse][-1]
        point2 = ori_path[idx_diffuse][0]

        if point1[0] < point2[0]:
            point_start = point1
            point_end = point2
        else:
            point_start = point2
            point_end = point1

        point_start=[point_start[0], point_start[1]+1]

        print("recovered_path", recovered_path)
        direction = 'horizontal'
        path_ccw, path_cw = cut_path(recovered_path, point_start, point_end, direction)
        paths.append(path_ccw)
        print("path_ccw", path_ccw)
        set_obstacle(grid, path_ccw, boundary_area)

    return paths


def diffuse(idx_diffuse, grid, idx_detour, flag_dir):
    paths = []
    if flag_dir == 'up':
        paths = []
        while idx_diffuse < idx_detour:
            simplified_path = simplify_contour_points(find_internal_contour_points(grid))
            recovered_path = recover_path(simplified_path, boundary_area)
            recovered_path.append(recovered_path[0])
            point1 = ori_path[idx_diffuse][-1]
            point2 = ori_path[idx_diffuse][0]

            if point1[0] < point2[0]:
                point_start = point1
                point_end = point2
            else:
                point_start = point2
                point_end = point1

            direction = 'horizontal'
            path_ccw, path_cw = cut_path(recovered_path, point_start, point_end, direction)
            paths.append(path_cw)
            print("path_cw", path_cw)
            set_obstacle(grid, path_cw, boundary_area)
            idx_diffuse += 1

    elif flag_dir == 'down':
        while idx_diffuse > idx_detour:
            simplified_path = simplify_contour_points(find_internal_contour_points(grid))
            print(find_internal_contour_points(grid))
            print("simplified path", simplified_path)
            recovered_path = recover_path(simplified_path, boundary_area)
            recovered_path.append(recovered_path[0])
            point1 = ori_path[idx_diffuse][-1]
            point2 = ori_path[idx_diffuse][0]

            if point1[0] < point2[0]:
                point_start = point1
                point_end = point2
            else:
                point_start = point2
                point_end = point1

            print("point_start", point_start)
            print("point_end", point_end)
            print("recovered_path", recovered_path)
            direction = 'horizontal'
            try:
                path_ccw, path_cw = cut_path(recovered_path, point_start, point_end, direction)
            except:
                new_point_start = [point_start[0], point_start[1] + 1]
                path_ccw, path_cw = cut_path(recovered_path, new_point_start, point_end, direction)
                path_ccw.insert(0, point_start)
            paths.append(path_ccw)
            print("path_ccw", path_ccw)
            set_obstacle(grid, path_ccw, boundary_area)
            idx_diffuse -= 1

    elif flag_dir == 'right':
        while idx_diffuse > idx_detour:
            simplified_path = simplify_contour_points(find_internal_contour_points(grid))
            recovered_path = recover_path(simplified_path, boundary_area)
            recovered_path.append(recovered_path[0])
            point_start = ori_path[idx_diffuse][-1]
            point_end = ori_path[idx_diffuse][0]
            direction = 'horizontal'
            path_ccw, path_cw = cut_path(recovered_path, point_start, point_end, direction)
            paths.append(path_ccw)
            print("path_ccw", path_ccw)
            set_obstacle(grid, path_ccw, boundary_area)
            idx_diffuse -= 1

    elif flag_dir == 'left':
        while idx_diffuse < idx_detour:
            simplified_path = simplify_contour_points(find_internal_contour_points(grid))
            print(find_internal_contour_points(grid))
            print("simplified path", simplified_path)
            recovered_path = recover_path(simplified_path, boundary_area)
            recovered_path.append(recovered_path[0])
            point_start = ori_path[idx_diffuse][-1]
            point_end = ori_path[idx_diffuse][0]
            print("recovered_path", recovered_path)
            direction = 'horizontal'
            path_ccw, path_cw = cut_path(recovered_path, point_start, point_end, direction)
            paths.append(path_cw)
            print("path_cw", path_cw)
            set_obstacle(grid, path_cw, boundary_area)
            idx_diffuse += 1
    return paths


def adjust_path(paths, boundary_area, direction='right'):
    print(paths[-1])
    target_length = cal_path_length(paths[-1])
    x_min, x_max, y_min, y_max = boundary_area
    return paths, target_length


def find_longest_segment(detour_path, max_length_pre=0):
    max_length = 0
    max_indices = (0, 0)

    for i in range(len(detour_path) - 1):
        length = abs(detour_path[i][0] - detour_path[i + 1][0]) + abs(detour_path[i][1] - detour_path[i + 1][1])

        if length > max_length and length != max_length_pre:
            max_length = length
            max_indices = (i, i + 1)

    return max_length, max_indices


if __name__ == '__main__':


    ori_path = [[(1, 0), (9, 0)],
                        [(1, 2), (9, 2)],
                        [(1, 4), (9, 4)],
                        [(1, 6), (9, 6)],
                        [(1, 8), (9, 8)],
                        [(1, 10), (9, 10)]]

    res_len = [10,8,6,4,2,0]

    ori_path=ori_path[::-1]
    res_len=res_len[::-1]

    time_start = time.time()

    x_min = min([point[0] for path in ori_path for point in path])
    x_max = max([point[0] for path in ori_path for point in path])
    y_min = min([point[1] for path in ori_path for point in path])
    y_max = max([point[1] for path in ori_path for point in path])+5

    boundary = [(x_min - 1, y_min - 1), (x_min - 1, y_max + 1), (x_max + 1, y_max + 1), (x_max + 1, y_min - 1), (x_min - 1, y_min - 1)]
    boundary_area = [x_min - 1, x_max + 1, y_min - 1, y_max + 1]

    target_length = cal_path_length(ori_path[-1])+res_len[-1]+6


    print(target_length)
    print([cal_path_length(ori_path[i])+res_len[i] for i in range(len(ori_path))])
    print(boundary_area, boundary)

    fixed_path = []
    print("fixed_path", fixed_path)

    detour_path=ori_path[-1]
    detour_path = [[point[0], point[1]] for point in detour_path]
    print("detour_path", detour_path)
    index=0
    while len(fixed_path) < len(ori_path)-1:
        if len(fixed_path) % 2 == 1:
            flag_dir = 'up'
        else:
            flag_dir = 'down'
        if flag_dir == 'down':
            if len(fixed_path) != 0:
                detour_path = paths[0]
            grid = initialize_grid(boundary, fixed_path, boundary_area)

            idx_diffuse = int(len(fixed_path) / 2)
            idx_detour = len(ori_path) - 1 - int(len(fixed_path) / 2)
            print(idx_diffuse,idx_detour)

            paths = diffuse(idx_diffuse, grid, idx_detour, 'up')
            for path in fixed_path:
                paths.append(path)
            print("detour path:", detour_path)
            print("flag_dir", flag_dir)
            max_length, max_idxes = find_longest_segment(detour_path)
            print(max_length, max_idxes)
            offset = 0
            while (grid[detour_path[max_idxes[0]][1] + 1 - (y_min - 1), detour_path[max_idxes[0]][0] + offset - (x_min - 1)] == 1 or
                   grid[detour_path[max_idxes[0]][1] + 2 - (y_min - 1), detour_path[max_idxes[0]][0] + offset - (x_min - 1)] == 1)\
                    or [detour_path[max_idxes[0]][0] + offset, detour_path[max_idxes[0]][1] + 1] in detour_path or \
                    [detour_path[max_idxes[0]][0] + offset, detour_path[max_idxes[0]][1] + 2] in detour_path:
                offset += 1
            print("offset:", offset)
            print(detour_path)
            flag_dir="up"
            detour_path=detour_spiral(detour_path, target_length, cal_path_length(detour_path) + res_len[idx_detour],
                          (detour_path[max_idxes[0]][0] + offset, detour_path[max_idxes[0]][1]),
                          detour_path[max_idxes[1]], flag_dir, max_idxes[0], grid)
            paths.append(detour_path)
            fixed_path.append(paths[-1])
        if flag_dir=='up':
            if len(fixed_path) != 0:
                detour_path = paths[0]

            print("==============")
            print(detour_path)
            grid = initialize_grid(boundary, fixed_path, boundary_area)

            idx_diffuse = len(ori_path) - 1 - (int(len(fixed_path) / 2) + 1)
            idx_detour = int(len(fixed_path) / 2)
            print(idx_diffuse, idx_detour)
            paths = diffuse(idx_diffuse, grid, idx_detour, 'down')
            paths.append(detour_path)
            paths.append(fixed_path[0])

            print(idx_diffuse,idx_detour)

            print("detour path:", detour_path)
            print("flag_dir", flag_dir)
            max_length, max_idxes = find_longest_segment(detour_path)
            print(max_idxes)
            offset = 0
            print("index",index)
            if index==1:
                offset=0
            elif index==2:
                offset=0
            else:
                while (grid[detour_path[max_idxes[0]][1] - 1 - (y_min - 1), detour_path[max_idxes[0]][0] + offset - (x_min - 1)] == 1 or
                       grid[detour_path[max_idxes[1]][1] - 2 - (y_min - 1), detour_path[max_idxes[1]][0] + offset - (x_min - 1)] == 1) or \
                        [detour_path[max_idxes[0]][0] + offset, detour_path[max_idxes[0]][1] - 1] in detour_path or \
                        [detour_path[max_idxes[0]][0] + offset, detour_path[max_idxes[0]][1] - 2] in detour_path:
                    offset += 1
            if index==0:
                offset=1

            flag_dir='down'
            detour_path=detour_spiral(detour_path, target_length, cal_path_length(detour_path) + res_len[idx_detour],
                          (detour_path[max_idxes[0]][0]+offset, detour_path[max_idxes[0]][1]), detour_path[max_idxes[1]], flag_dir, max_idxes[0], grid)

            print(fixed_path)
            for path in fixed_path:
                paths.append(path)
            paths.append(detour_path)

            fixed_path.append(paths[-1])
            print(detour_path)
        index += 1

    print("Total Time:",time.time() - time_start)
    visualize(fixed_path)
    print(paths)
    for path in paths:
        print("number of bends:", len(path) - 2)

