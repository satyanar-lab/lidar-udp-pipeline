// src/bench_parser.cpp
// Benchmark harness for the RPLIDAR parser.
// Loads a capture once, then feeds it through a fresh Parser many times,
// in the same small-chunk pattern the real driver uses, and times only
// the parsing work.

#include "rplidar_parser.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <vector>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <capture.bin> [passes]\n", argv[0]);
        return 1;
    }

    const char* path = argv[1];
    const int passes = (argc >= 3) ? std::atoi(argv[2]) : 2000;

    // Slurp the whole capture into memory once: we want to measure
    // parsing, not disk I/O.
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        std::fprintf(stderr, "cannot open %s\n", path);
        return 1;
    }
    std::vector<std::uint8_t> data(
        (std::istreambuf_iterator<char>(in)),
        std::istreambuf_iterator<char>());
    if (data.empty()) {
        std::fprintf(stderr, "%s is empty\n", path);
        return 1;
    }

    constexpr std::size_t kChunk = 512;

    const auto t0 = std::chrono::steady_clock::now();

    std::size_t total_scans = 0;
    for (int p = 0; p < passes; ++p) {
        rplidar::Parser parser;              // fresh parser each pass
        std::vector<rplidar::Scan> scans;
        for (std::size_t off = 0; off < data.size(); off += kChunk) {
            const std::size_t n = std::min(kChunk, data.size() - off);
            parser.feed(data.data() + off, n, scans);
        }
        total_scans += scans.size();
    }

    const auto t1 = std::chrono::steady_clock::now();
    const double ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    const double mb = (double(data.size()) * passes) / (1024.0 * 1024.0);

    std::printf("passes=%d  bytes/pass=%zu  total=%.1f MB\n",
                passes, data.size(), mb);
    std::printf("time=%.1f ms   throughput=%.1f MB/s\n",
                ms, mb / (ms / 1000.0));
    std::printf("scans=%zu\n", total_scans);
    return 0;
}