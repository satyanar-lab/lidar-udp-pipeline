#include "rplidar_parser.hpp"

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    std::string path = (argc > 1) ? argv[1] : "captures/scan.bin";

    std::ifstream in(path, std::ios::binary);
    if (!in) { std::cerr << "cannot open " << path << "\n"; return 1; }

    rplidar::Parser parser;
    std::vector<rplidar::Scan> scans;

    std::uint8_t chunk[512];                 // deliberately NOT a multiple of 5
    while (in) {
        in.read(reinterpret_cast<char*>(chunk), sizeof(chunk));
        std::streamsize got = in.gcount();
        if (got > 0)
            parser.feed(chunk, static_cast<std::size_t>(got), scans);
    }

    std::cout << "parsed " << scans.size() << " complete scans\n";
    if (!scans.empty()) {
        const rplidar::Scan& s = scans.front();
        std::cout << "first scan: " << s.size() << " points\n";
        if (!s.empty()) {
            const rplidar::Measurement& m = s.front();
            std::cout << "  first point: angle " << m.angle_deg
                      << " deg, distance " << m.distance_mm << " mm\n";
            float min_d = s.front().distance_mm, min_a = s.front().angle_deg;
            float max_d = min_d,                 max_a = min_a;
            for (const rplidar::Measurement& mm : s) {
                if (mm.distance_mm < min_d) { min_d = mm.distance_mm; min_a = mm.angle_deg; }
                if (mm.distance_mm > max_d) { max_d = mm.distance_mm; max_a = mm.angle_deg; }
            }
            std::cout << "  nearest:  " << min_d << " mm at " << min_a << " deg\n";
            std::cout << "  farthest: " << max_d << " mm at " << max_a << " deg\n";
        }
    }
    return 0;
}