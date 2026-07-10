#include "dfs.hpp"
#include "lut.hpp"
#include "lutio.hpp"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

int parse_int(const char *value, const char *name) {
  try {
    return std::stoi(value);
  } catch (const std::exception &) {
    std::cerr << "invalid " << name << ": " << value << "\n";
    std::exit(1);
  }
}

std::uint32_t parse_seed(const char *value) {
  try {
    return static_cast<std::uint32_t>(std::stoul(value));
  } catch (const std::exception &) {
    std::cerr << "invalid seed: " << value << "\n";
    std::exit(1);
  }
}

void usage(const char *argv0) {
  std::cerr << "usage: " << argv0
            << " <output_lut> [M N samples_per_bend max_bend seed]\n";
}

} // namespace

int main(int argc, char **argv) {
  std::string output = "optimizer_lut_60_60_100_20_final";
  int max_m = 60;
  int max_n = 60;
  int samples_per_bend = 100;
  int max_bend = 20;
  std::uint32_t seed = 1;

  if (argc == 2) {
    output = argv[1];
  } else if (argc == 6 || argc == 7) {
    output = argv[1];
    max_m = parse_int(argv[2], "M");
    max_n = parse_int(argv[3], "N");
    samples_per_bend = parse_int(argv[4], "samples_per_bend");
    max_bend = parse_int(argv[5], "max_bend");
    if (argc == 7) {
      seed = parse_seed(argv[6]);
    }
  } else if (argc != 1) {
    usage(argv[0]);
    return 1;
  }

  if (max_m < 0 || max_n < 0 || samples_per_bend <= 0 || max_bend < 0) {
    usage(argv[0]);
    return 1;
  }

  auto start = std::chrono::high_resolution_clock::now();
  LUT lut;
  std::uint64_t total_paths = 0;

  for (int m = 0; m <= max_m; ++m) {
    for (int n = 0; n <= max_n; ++n) {
      std::cout << "generate_lut(" << m << "," << n << ")\n";
      DFS dfs(m, n, samples_per_bend, max_bend, seed);
      dfs.set_visitor([&](const pathio::Path &path) {
        lut.add_solution(path);
      });
      total_paths += dfs.run();
    }
  }

  lut.finalize();
  lut.deduplicate_buckets();

  if (!lutio::save_binary(lut, output)) {
    std::cerr << "failed to write LUT: " << output << "\n";
    return 2;
  }

  auto end = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

  std::cout << "wrote " << output << "\n";
  std::cout << "lut buckets = " << lut.data().size() << "\n";
  std::cout << "sampled paths = " << total_paths << "\n";
  std::cout << "generate_lut time = " << duration.count() << " ms\n";
  return 0;
}
