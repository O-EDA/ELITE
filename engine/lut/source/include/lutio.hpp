#pragma once
#include "lut.hpp"
#include "pathio.hpp"

#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace lutio {

static inline void write_bytes(std::ostream &os, const void *p, size_t n) {
  os.write(reinterpret_cast<const char *>(p), (std::streamsize)n);
  if (!os)
    throw std::runtime_error("lutio: write failed");
}

static inline void read_bytes(std::istream &is, void *p, size_t n) {
  is.read(reinterpret_cast<char *>(p), (std::streamsize)n);
  if (!is)
    throw std::runtime_error("lutio: read failed");
}

template <class T> static inline void write_pod(std::ostream &os, const T &v) {
  static_assert(std::is_trivially_copyable<T>::value, "POD required");
  write_bytes(os, &v, sizeof(T));
}

template <class T> static inline T read_pod(std::istream &is) {
  static_assert(std::is_trivially_copyable<T>::value, "POD required");
  T v{};
  read_bytes(is, &v, sizeof(T));
  return v;
}

struct Header {
  char magic[4];    // "LUTB"
  uint32_t ver;     // 2
  uint32_t buckets; // number of buckets
};

// Format v3:
// - Key: int16 m, int16 n, int16 l, int8 d_t
// - Entries:
//    uint32 count
//    for each entry:
//      int32 bends
//      uint16 plen (number of points)
//      plen * {int16 x, int16 y}  (path points)
static inline bool save_binary(const LUT &lut, const std::string &path) {
  std::ofstream out(path, std::ios::binary);
  if (!out.is_open())
    return false;

  try {
    const auto &table = lut.data();

    Header h{};
    h.magic[0] = 'L';
    h.magic[1] = 'U';
    h.magic[2] = 'T';
    h.magic[3] = 'B';
    h.ver = 3;
    h.buckets = (uint32_t)table.size();
    write_pod(out, h);

    for (const auto &kv : table) {
      const LutKey &k = kv.first;
      const auto &entries = kv.second.entries;

      // key (m,n,l,d_t)
      write_pod(out, k.m);
      write_pod(out, k.n);
      write_pod(out, k.l);
      write_pod(out, k.d_t);

      // entries
      uint32_t n = (uint32_t)entries.size();
      write_pod(out, n);

      for (const auto &e : entries) {
        // bends
        int32_t bends = (int32_t)e.bends;
        write_pod(out, bends);

        // path length (number of points)
        uint16_t plen = (uint16_t)e.path.size();
        write_pod(out, plen);

        // path points (x, y)
        for (size_t i = 0; i < e.path.size(); ++i) {
          int16_t x = (int16_t)e.path[i].x;
          int16_t y = (int16_t)e.path[i].y;
          write_pod(out, x);
          write_pod(out, y);
        }
      }
    }
  } catch (...) {
    return false;
  }

  return true;
}

static inline bool load_binary(LUT &lut, const std::string &path) {
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open())
    return false;

  try {
    Header h = read_pod<Header>(in);
    if (!(h.magic[0] == 'L' && h.magic[1] == 'U' && h.magic[2] == 'T' &&
          h.magic[3] == 'B'))
      throw std::runtime_error("lutio: bad magic");

    if (h.ver != 3)
      throw std::runtime_error("lutio: unsupported version");

    lut.clear();

    for (uint32_t bi = 0; bi < h.buckets; ++bi) {
      LutKey k{};
      k.m = read_pod<int16_t>(in);
      k.n = read_pod<int16_t>(in);
      k.l = read_pod<int16_t>(in);
      k.d_t = read_pod<int8_t>(in);

      uint32_t n = read_pod<uint32_t>(in);
      std::vector<LUT::Entry> entries;
      entries.reserve(n);

      for (uint32_t i = 0; i < n; ++i) {
        int32_t bends = read_pod<int32_t>(in);
        uint16_t plen = read_pod<uint16_t>(in);

        pathio::Path path;
        for (uint16_t j = 0; j < plen; ++j) {
          int16_t x = read_pod<int16_t>(in);
          int16_t y = read_pod<int16_t>(in);
          path.add_point(x, y);
        }

        entries.push_back(LUT::Entry{(int)bends, std::move(path)});
      }

      // data is assumed finalized on disk (sorted / truncated as you saved it)
      lut.import_bucket(k, std::move(entries));
    }
  } catch (...) {
    return false;
  }

  return true;
}

} // namespace lutio