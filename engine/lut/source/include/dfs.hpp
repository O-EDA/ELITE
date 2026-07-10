#pragma once

#include "pathio.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <random>
#include <tuple>
#include <utility>
#include <vector>

class DFS {
public:
  using Visitor = std::function<void(const pathio::Path &)>;

  DFS(int m, int n, int samples_per_bend = 100, int max_bend_limit = 20,
      std::uint32_t seed = 1)
      : m(m), n(n), samples_per_bend(std::max(1, samples_per_bend)),
        max_bend_limit(std::max(0, max_bend_limit)),
        rng(seed ^ (std::uint32_t)(m * 73856093u) ^
            (std::uint32_t)(n * 19349663u)) {
    visited.assign((m + 1) * (n + 1), false);
  }

  void set_visitor(Visitor v) { visitor = std::move(v); }

  std::uint64_t run() {
    total_count = 0;
    state1_counts.clear();
    state2_counts.clear();

    for (current_max_bend = 0; current_max_bend <= max_bend_limit;
         ++current_max_bend) {
      for (int attempt = 0; attempt < samples_per_bend; ++attempt) {
        reset_path();
        bool found_at_this_bend = false;

        std::vector<int> start_options;
        for (int nx = 1; nx <= m; ++nx) {
          start_options.push_back(nx);
        }
        std::shuffle(start_options.begin(), start_options.end(), rng);

        for (int nx : start_options) {
          if (try_move(0, 0, nx, 0, true)) {
            dfs(nx, 0, false, 0, nx, found_at_this_bend);
            try_move(0, 0, nx, 0, false);
            path.points.pop_back();
            if (found_at_this_bend) {
              break;
            }
          }
        }
      }
    }

    return total_count;
  }

private:
  void reset_path() {
    std::fill(visited.begin(), visited.end(), false);
    path.points.clear();
    path.add_point(0, 0);
    visited[0] = true;
  }

  bool try_move(int x1, int y1, int x2, int y2, bool mark) {
    int dx = (x2 > x1) - (x2 < x1);
    int dy = (y2 > y1) - (y2 < y1);
    int width = m + 1;

    if (mark) {
      int cx = x1;
      int cy = y1;
      int steps = 0;
      bool blocked = false;

      while (cx != x2 || cy != y2) {
        cx += dx;
        cy += dy;
        int idx = cy * width + cx;
        if (visited[idx]) {
          blocked = true;
          break;
        }
        visited[idx] = true;
        steps++;
      }

      if (blocked) {
        cx -= dx;
        cy -= dy;
        for (int i = 0; i < steps; ++i) {
          visited[cy * width + cx] = false;
          cx -= dx;
          cy -= dy;
        }
        return false;
      }

      path.add_point(x2, y2);
      return true;
    }

    int cx = x1;
    int cy = y1;
    while (cx != x2 || cy != y2) {
      cx += dx;
      cy += dy;
      visited[cy * width + cx] = false;
    }
    return true;
  }

  void dfs(int x, int y, bool horiz, int bends, int current_len,
           bool &found_any) {
    if (found_any) {
      return;
    }

    int dist_to_end = std::abs(m - x) + std::abs(n - y);
    if (current_len + dist_to_end > 2 * (m + n)) {
      return;
    }

    if (x == m && y == n) {
      if (bends == current_max_bend) {
        bool arrived_horizontally = !horiz;
        std::pair<int, bool> state1 = {current_len, arrived_horizontally};
        std::tuple<int, int, bool> state2 = {current_len, bends,
                                             arrived_horizontally};
        state1_counts[state1]++;
        state2_counts[state2]++;
        total_count++;
        found_any = true;
        if (visitor) {
          visitor(path);
        }
      }
      return;
    }

    int min_bends_needed = (x != m && y != n) ? 1 : 0;
    if (bends + min_bends_needed > current_max_bend) {
      return;
    }

    if (horiz) {
      std::vector<int> next_steps;
      for (int nx = 0; nx <= m; ++nx) {
        if (nx != x) {
          next_steps.push_back(nx);
        }
      }
      std::shuffle(next_steps.begin(), next_steps.end(), rng);

      for (int nx : next_steps) {
        int step_dist = std::abs(nx - x);
        if (try_move(x, y, nx, y, true)) {
          dfs(nx, y, !horiz, bends + 1, current_len + step_dist, found_any);
          try_move(x, y, nx, y, false);
          path.points.pop_back();
          if (found_any) {
            return;
          }
        }
      }
    } else {
      std::vector<int> next_steps;
      for (int ny = 0; ny <= n; ++ny) {
        if (ny != y) {
          next_steps.push_back(ny);
        }
      }
      std::shuffle(next_steps.begin(), next_steps.end(), rng);

      for (int ny : next_steps) {
        int step_dist = std::abs(ny - y);
        if (try_move(x, y, x, ny, true)) {
          dfs(x, ny, !horiz, bends + 1, current_len + step_dist, found_any);
          try_move(x, y, x, ny, false);
          path.points.pop_back();
          if (found_any) {
            return;
          }
        }
      }
    }
  }

  int m;
  int n;
  int samples_per_bend;
  int max_bend_limit;
  int current_max_bend = 0;
  std::uint64_t total_count = 0;
  std::vector<bool> visited;
  std::map<std::pair<int, bool>, int> state1_counts;
  std::map<std::tuple<int, int, bool>, int> state2_counts;
  pathio::Path path;
  Visitor visitor;
  std::mt19937 rng;
};
