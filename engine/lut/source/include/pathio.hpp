#pragma once

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace pathio {


struct Point {
  int x;
  int y;

  Point(int x = 0, int y = 0) : x(x), y(y) {}

  bool operator==(const Point &other) const {
    return x == other.x && y == other.y;
  }

  bool operator!=(const Point &other) const { return !(*this == other); }
};


struct Path {
  std::vector<Point> points;
  Point &operator[](size_t index) { return points[index]; }

  bool operator==(const Path &other) const {
    if (points.size() != other.points.size())
      return false;
    for (size_t i = 0; i < points.size(); ++i) {
      if (points[i] != other.points[i])
        return false;
    }
    return true;
  }


  bool operator<(const Path &other) const {
    if (points.size() != other.points.size()) {
      return points.size() < other.points.size();
    }

    for (size_t i = 0; i < points.size(); ++i) {
      if (points[i].x != other.points[i].x)
        return points[i].x < other.points[i].x;
      if (points[i].y != other.points[i].y)
        return points[i].y < other.points[i].y;
    }
    return false;
  }
  const Point &operator[](size_t index) const { return points[index]; }

  void add_point(int x, int y) { points.emplace_back(x, y); }

  void replace_segment(size_t start_idx, size_t end_idx,
                       const std::vector<Point> &candidate) {
    if (start_idx > end_idx || end_idx >= points.size())
      return;

    // 1. Remove the old segment [start_idx, end_idx]
    // erase(first, last) removes up to but NOT including 'last', so we use +1
    auto it_start = points.begin() + start_idx;
    auto it_end = points.begin() + end_idx + 1;
    points.erase(it_start, it_end);

    // 2. Insert the candidate points at start_idx
    points.insert(points.begin() + start_idx, candidate.begin(),
                  candidate.end());

    // 3. Optional: Clean up collinear points after replacement
    remove_duplicates();
  }
  size_t size() const { return points.size(); }


  void remove_duplicates() {
    if (points.size() < 3)
      return;

    std::vector<Point> unique_points;
    unique_points.push_back(points[0]); // Keep the first point

    for (size_t i = 1; i < points.size() - 1; ++i) {
      const Point &prev = unique_points.back();
      const Point &curr = points[i];
      const Point &next = points[i + 1];

      // Check if the current point is collinear with the previous two points
      int dx1 = curr.x - prev.x;
      int dy1 = curr.y - prev.y;
      int dx2 = next.x - curr.x;
      int dy2 = next.y - curr.y;

      // If the points are collinear, skip the current point
      if (dx1 * dy2 == dx2 * dy1) {
        continue; // Skip collinear point
      }

      // Otherwise, add the current point to the result
      unique_points.push_back(curr);
    }

    // Add the last point
    unique_points.push_back(points.back());

    points = std::move(unique_points);
  }


  int calculate_bends() const {
    int bends = 0;

    for (size_t i = 1; i < points.size() - 1; ++i) {
      const Point &prev = points[i - 1];
      const Point &curr = points[i];
      const Point &next = points[i + 1];


      int dx1 = curr.x - prev.x;
      int dy1 = curr.y - prev.y;
      int dx2 = next.x - curr.x;
      int dy2 = next.y - curr.y;



      if ((dx1 == 0 && dx2 != 0) || (dy1 == 0 && dy2 != 0) ||
          (dx1 != 0 && dy1 != 0 && dx2 != 0 && dy2 != 0)) {
        ++bends;
      }
    }

    return bends;
  }



  static int calculate_direction(const Point &from, const Point &to) {
    int dx = to.x - from.x;
    int dy = to.y - from.y;


    if (dx > 0 && dy == 0)
      return 0;
    else if (dx == 0 && dy > 0)
      return 1;
    else if (dx < 0 && dy == 0)
      return 2;
    else if (dx == 0 && dy < 0)
      return 3;
    else
      return -1;
  }



  static int calculate_source_direction(const Path &path) {
    if (path.points.size() < 2) {
      return -1;
    }
    return calculate_direction(path.points[0], path.points[1]);
  }



  static int calculate_target_direction(const Path &path) {
    if (path.points.size() < 2) {
      return -1;
    }
    size_t last_idx = path.points.size() - 1;
    return calculate_direction(path.points[last_idx - 1],
                               path.points[last_idx]);
  }
};


class GridMap {
private:
  int width_;
  int height_;
  int offset_x_ = 0;
  int offset_y_ = 0;
  std::vector<std::vector<int>> grid_;

public:
  GridMap(int width, int height) : width_(width), height_(height) {
    grid_.resize(height_, std::vector<int>(width_, 0));
  }


  GridMap(const std::vector<Path> &paths) {

    int min_x = 0, min_y = 0;
    int max_x = 0, max_y = 0;
    for (const auto &path : paths) {
      for (const auto &point : path.points) {
        min_x = std::min(min_x, point.x);
        min_y = std::min(min_y, point.y);
        max_x = std::max(max_x, point.x);
        max_y = std::max(max_y, point.y);
      }
    }


    offset_x_ = -min_x;
    offset_y_ = -min_y;


    width_ = max_x - min_x + 1;
    height_ = max_y - min_y + 1;
    grid_.resize(height_, std::vector<int>(width_, 0));


    for (const auto &path : paths) {
      mark_path(path, 1);
    }
  }


  int width() const { return width_; }
  int height() const { return height_; }


  bool is_valid(int x, int y) const {
    int grid_x = x + offset_x_;
    int grid_y = y + offset_y_;
    return grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_;
  }


  int get(int x, int y) const {
    int grid_x = x + offset_x_;
    int grid_y = y + offset_y_;
    if (!is_valid(x, y)) {
      return -1;
    }
    return grid_[grid_y][grid_x];
  }


  void set(int x, int y, int value) {
    int grid_x = x + offset_x_;
    int grid_y = y + offset_y_;
    if (is_valid(x, y)) {
      grid_[grid_y][grid_x] = value;
    }
  }


  void mark_path(const Path &path, int value = 1) {
    for (size_t i = 0; i < path.points.size(); ++i) {
      const Point &p = path.points[i];
      set(p.x, p.y, value);


      if (i + 1 < path.points.size()) {
        const Point &next = path.points[i + 1];
        mark_line(p, next, value);
      }
    }
  }


  void mark_line(const Point &p1, const Point &p2, int value) {
    int dx = std::abs(p2.x - p1.x);
    int dy = std::abs(p2.y - p1.y);
    int steps = std::max(dx, dy);

    if (steps == 0)
      return;

    float x_inc = static_cast<float>(p2.x - p1.x) / steps;
    float y_inc = static_cast<float>(p2.y - p1.y) / steps;

    float x = static_cast<float>(p1.x);
    float y = static_cast<float>(p1.y);

    for (int i = 0; i <= steps; ++i) {
      set(static_cast<int>(std::round(x)), static_cast<int>(std::round(y)),
          value);
      x += x_inc;
      y += y_inc;
    }
  }


  bool has_conflict(const Path &path) const {
    for (size_t i = 0; i < path.points.size(); ++i) {
      const Point &p = path.points[i];


      if (get(p.x, p.y) > 0) {
        return true;
      }


      if (i + 1 < path.points.size()) {
        const Point &next = path.points[i + 1];
        if (has_line_conflict(p, next)) {
          return true;
        }
      }
    }
    return false;
  }


  bool has_line_conflict(const Point &p1, const Point &p2) const {
    int dx = std::abs(p2.x - p1.x);
    int dy = std::abs(p2.y - p1.y);
    int steps = std::max(dx, dy);

    if (steps == 0)
      return false;

    float x_inc = static_cast<float>(p2.x - p1.x) / steps;
    float y_inc = static_cast<float>(p2.y - p1.y) / steps;

    float x = static_cast<float>(p1.x);
    float y = static_cast<float>(p1.y);

    for (int i = 0; i <= steps; ++i) {
      int grid_x = static_cast<int>(std::round(x));
      int grid_y = static_cast<int>(std::round(y));


      if (get(grid_x, grid_y) > 0) {
        return true;
      }

      x += x_inc;
      y += y_inc;
    }
    return false;
  }


};


inline void sort_paths_by_bends(std::vector<Path> &paths) {
  std::sort(paths.begin(), paths.end(), [](const Path &a, const Path &b) {
    return a.calculate_bends() > b.calculate_bends();
  });
}
} // namespace pathio
