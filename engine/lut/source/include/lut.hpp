#pragma once
#include "pathio.hpp"
#include <algorithm>
#include <cstdint>
#include <unordered_map>
#include <vector>

// ======================= key =======================
struct LutKey {
  int16_t m;
  int16_t n;
  int16_t l;
  int8_t d_t;

  bool operator==(const LutKey &o) const noexcept {
    return m == o.m && n == o.n && l == o.l && d_t == o.d_t;
  }
};

struct LutKeyHash {
  size_t operator()(const LutKey &k) const noexcept {
    uint64_t x = 0;
    x |= (uint64_t)(uint16_t)k.m;
    x |= (uint64_t)(uint16_t)k.n << 16;
    x |= (uint64_t)(uint16_t)k.l << 32;
    x |= (uint64_t)(uint8_t)k.d_t << 48;
    return (size_t)x;
  }
};

// ======================= props =======================
struct PathProps {
  int end_x = 0, end_y = 0;
  int len = 0;
  int end_dir = -1; // last move
  int bends = 0;    // direction changes
  bool valid = false;
};

static inline PathProps calc_props(const pathio::Path &path) {
  PathProps p;

  if (path.size() == 0) {
    p.len = 0;
    return p;
  }

  // Calculate actual path length (sum of Manhattan distances between
  // consecutive points)
  p.len = 0;
  for (size_t i = 1; i < path.size(); ++i) {
    p.len += std::abs(path[i].x - path[i - 1].x) +
             std::abs(path[i].y - path[i - 1].y);
  }

  // Calculate bends using the Path's own method
  p.bends = path.calculate_bends();

  // Get end point
  p.end_x = path[path.size() - 1].x;
  p.end_y = path[path.size() - 1].y;

  // Calculate end direction if we have at least 2 points
  if (path.size() > 1) {
    int dx = path[path.size() - 1].x - path[path.size() - 2].x;
    int dy = path[path.size() - 1].y - path[path.size() - 2].y;

    // Convert to direction: 0=R,1=U,2=L,3=D
    if (dx > 0)
      p.end_dir = 0; // Right
    else if (dy > 0)
      p.end_dir = 1; // Up
    else if (dx < 0)
      p.end_dir = 2; // Left
    else if (dy < 0)
      p.end_dir = 3; // Down
  }

  p.valid = true;
  return p;
}

// ======================= LUT =======================
class LUT {
public:
  struct Entry {
    int bends = 0;
    pathio::Path path;
  };

  struct Bucket {
    std::vector<Entry> entries; // sorted by bends after finalize()
  };

  // K<=0: keep all; K>0: keep only smallest-bend K entries per bucket
  explicit LUT(int K = 0) : K(K) {}

  // ---------- build API ----------
  void add_solution(const pathio::Path &path) {
    // Create a copy of the path to optimize it
    pathio::Path optimized_path = path;

    // Remove redundant points on straight lines
    optimized_path.remove_duplicates();

    PathProps p = calc_props(optimized_path);
    if (!p.valid)
      return;

    LutKey key;
    key.m = (int16_t)p.end_x;
    key.n = (int16_t)p.end_y;
    key.l = (int16_t)p.len;
    key.d_t = (int8_t)p.end_dir;

    table[key].entries.push_back(Entry{p.bends, optimized_path});
  }

  // Sort each bucket and optionally keep top-K (smallest bends).
  void finalize() {
    for (auto &kv : table) {
      auto &b = kv.second;

      std::sort(b.entries.begin(), b.entries.end(),
                [](const Entry &a, const Entry &c) {
                  if (a.bends != c.bends)
                    return a.bends < c.bends;
                  // For paths with same bends, compare by size (shorter first)
                  return a.path.size() < c.path.size();
                });

      if (K > 0 && (int)b.entries.size() > K)
        b.entries.resize(K);
    }
  }

  void deduplicate_buckets() {
    for (auto &kv : table) {
      auto &b = kv.second;
      if (b.entries.empty())
        continue;



      std::sort(b.entries.begin(), b.entries.end(),
                [](const Entry &a, const Entry &c) {
                  if (a.bends != c.bends)
                    return a.bends < c.bends;
                  return a.path < c.path;
                });


      auto last =
          std::unique(b.entries.begin(), b.entries.end(),
                      [](const Entry &a, const Entry &c) {
                        return a.path == c.path;
                      });

      b.entries.erase(last, b.entries.end());
    }
  }
  // ---------- query API ----------
  std::vector<std::pair<pathio::Path, int>>
  query_better(const pathio::Path &hi_path, int limit = 0) const {
    std::vector<std::pair<pathio::Path, int>> out;

    PathProps p = calc_props(hi_path);
    if (!p.valid)
      return out;

    LutKey key;
    key.m = (int16_t)p.end_x;
    key.n = (int16_t)p.end_y;
    key.l = (int16_t)p.len;
    key.d_t = (int8_t)p.end_dir;

    auto it = table.find(key);
    if (it == table.end())
      return out;

    const auto &vec = it->second.entries;
    const int bend_hi = p.bends;

    auto pos =
        std::lower_bound(vec.begin(), vec.end(), bend_hi,
                         [](const Entry &e, int b) { return e.bends < b; });

    for (auto i = vec.begin(); i != pos; ++i) {
      out.emplace_back(i->path, i->bends);
      if (limit > 0 && (int)out.size() >= limit)
        break;
    }
    return out;
  }

  std::vector<std::pair<pathio::Path, int>>
  query_better(LutKey &key, int &bend_hi, int limit = 0) const {
    std::vector<std::pair<pathio::Path, int>> out;
    auto it = table.find(key);
    if (it == table.end())
      return out;

    const auto &vec = it->second.entries;
    auto pos =
        std::lower_bound(vec.begin(), vec.end(), bend_hi,
                         [](const Entry &e, int b) { return e.bends < b; });

    for (auto i = vec.begin(); i != pos; ++i) {
      out.emplace_back(i->path, i->bends);
      if (limit > 0 && (int)out.size() >= limit)
        break;
    }
    return out;
  }

  // ---------- IO-friendly API ----------
  void clear() { table.clear(); }

  void import_bucket(const LutKey &key, std::vector<Entry> &&entries) {
    if (entries.empty())
      return;
    Bucket b;
    b.entries = std::move(entries);
    table.emplace(key, std::move(b));
  }

  const std::unordered_map<LutKey, Bucket, LutKeyHash> &data() const {
    return table;
  }

private:
  int K;
  std::unordered_map<LutKey, Bucket, LutKeyHash> table;
};
