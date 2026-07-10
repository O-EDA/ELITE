#pragma once
#include "lut.hpp"
#include "pathio.hpp"
#include <algorithm>
#include <vector>

struct RegionInfo {
  int level, start_idx, end_idx, length, bend_count;
  pathio::Path points_path;
};


struct PathTransformer {
  int rotation_angle = 0;
  bool is_mirrored = false;
  pathio::Point offset;

  PathTransformer(const pathio::Path &path, const RegionInfo &region) {
    offset = path.points[region.start_idx];
    int dx = path.points[region.end_idx].x - offset.x;
    int dy = path.points[region.end_idx].y - offset.y;

    int src_dir = pathio::Path::calculate_source_direction(region.points_path);


    rotation_angle = src_dir % 4;
    auto rotate = [](int &x, int &y) {
      int tmp = x;
      x = y;
      y = -tmp;
    };
    for (int i = 0; i < rotation_angle; ++i)
      rotate(dx, dy);


    if (dy < 0) {
      is_mirrored = true;
      dy = -dy;
    }

    transformed_m = dx;
    transformed_n = dy;

    int target_dir =
        pathio::Path::calculate_target_direction(region.points_path);
    transformed_dt = (target_dir - rotation_angle + 4) % 4;
    if (is_mirrored) {
      if (transformed_dt == 1)
        transformed_dt = 3;
      else if (transformed_dt == 3)
        transformed_dt = 1;
    }
  }


  void apply_inverse(pathio::Path &p) const {
    for (auto &pt : p.points) {
      if (is_mirrored)
        pt.y = -pt.y;

      for (int i = 0; i < rotation_angle; ++i) {
        int tmp = pt.x;
        pt.x = -pt.y;
        pt.y = tmp;
      }
      pt.x += offset.x;
      pt.y += offset.y;
    }
  }

  int transformed_m, transformed_n, transformed_dt;
};

class Optimizer {
public:
  Optimizer(const LUT &lut, int A = 1) : lut(lut), A(A) {}

  void optimize_paths(std::vector<pathio::Path> &paths) {
    pathio::GridMap grid_map(paths);

    while (true) {
      pathio::sort_paths_by_bends(paths);
      if (!implement_lut(paths[0], grid_map))
        break;
    }
  }

private:
  bool implement_lut(pathio::Path &path, pathio::GridMap &grid_map) {
    std::vector<RegionInfo> regions = segment_path(path);
    for (auto &region : regions) {
      if (try_optimize_region(path, region, grid_map)) {
        return true;
      }
    }
    return false;
  }

  std::vector<RegionInfo> segment_path(const pathio::Path &path) {
    std::vector<RegionInfo> base_regions;
    int start = 0;


    while (start < (int)path.size() - 1) {
      int end = start + 1;
      int win_min_x = path[start].x, win_max_x = path[start].x;
      int win_min_y = path[start].y, win_max_y = path[start].y;

      while (end < (int)path.size()) {
        const auto &p = path[end];
        int i_min_x = std::min(path[start].x, p.x),
            i_max_x = std::max(path[start].x, p.x);
        int i_min_y = std::min(path[start].y, p.y),
            i_max_y = std::max(path[start].y, p.y);

        win_min_x = std::min(win_min_x, p.x);
        win_max_x = std::max(win_max_x, p.x);
        win_min_y = std::min(win_min_y, p.y);
        win_max_y = std::max(win_max_y, p.y);

        if (win_min_x < i_min_x || win_max_x > i_max_x || win_min_y < i_min_y ||
            win_max_y > i_max_y) {
          break;
        }
        end++;
      }

      int actual_end = (end == (int)path.size() && !base_regions.empty())
                           ? end - 1
                           : std::max(start + 1, end - 1);


      base_regions.push_back(create_region_info(path, start, actual_end, 1));
      start = actual_end;
    }


    std::vector<RegionInfo> all_regions = base_regions;
    int num_base = (int)base_regions.size();

    for (int k = 2; k <= A; ++k) {
      for (int i = 0; i <= num_base - k; ++i) {
        int range_start_idx = base_regions[i].start_idx;
        int range_end_idx = base_regions[i + k - 1].end_idx;


        int ideal_min_x =
            std::min(path[range_start_idx].x, path[range_end_idx].x);
        int ideal_max_x =
            std::max(path[range_start_idx].x, path[range_end_idx].x);
        int ideal_min_y =
            std::min(path[range_start_idx].y, path[range_end_idx].y);
        int ideal_max_y =
            std::max(path[range_start_idx].y, path[range_end_idx].y);


        int act_min_x = path[range_start_idx].x,
            act_max_x = path[range_start_idx].x;
        int act_min_y = path[range_start_idx].y,
            act_max_y = path[range_start_idx].y;

        for (int j = i; j < i + k; ++j) {
          const auto &r = base_regions[j];
          for (int idx : {r.start_idx, r.end_idx}) {
            act_min_x = std::min(act_min_x, path[idx].x);
            act_max_x = std::max(act_max_x, path[idx].x);
            act_min_y = std::min(act_min_y, path[idx].y);
            act_max_y = std::max(act_max_y, path[idx].y);
          }
        }

        if (act_min_x == ideal_min_x && act_max_x == ideal_max_x &&
            act_min_y == ideal_min_y && act_max_y == ideal_max_y) {

          all_regions.push_back(
              create_region_info(path, range_start_idx, range_end_idx, k));
        }
      }
    }

    return all_regions;
  }
  RegionInfo create_region_info(const pathio::Path &path, int s, int e,
                                int level) {
    RegionInfo r{level, s, e, 0, e - s - 1};
    for (int i = s; i <= e; ++i) {
      r.points_path.add_point(path[i].x, path[i].y);
      if (i > s)
        r.length += std::abs(path[i].x - path[i - 1].x) +
                    std::abs(path[i].y - path[i - 1].y);
    }
    return r;
  }

  bool try_optimize_region(pathio::Path &path, RegionInfo &region,
                           pathio::GridMap &grid_map) {
    PathTransformer trans(path, region);

    LutKey key{(int16_t)trans.transformed_m, (int16_t)trans.transformed_n,
               (int16_t)region.length, (int8_t)trans.transformed_dt};
    auto candidates = lut.query_better(key, region.bend_count);
    if (candidates.empty()) {
      return false;
    }

    grid_map.mark_path(region.points_path, 0);
    for (const auto &cand : candidates) {
      pathio::Path transformed_cand = cand.first;
      trans.apply_inverse(transformed_cand);

      if (!grid_map.has_conflict(transformed_cand)) {
        path.replace_segment(region.start_idx, region.end_idx,
                             transformed_cand.points);
        grid_map.mark_path(transformed_cand, 1);
        return true;
      }
    }

    grid_map.mark_path(region.points_path, 1);
    return false;
  }

  const LUT &lut;
  int A;
};
