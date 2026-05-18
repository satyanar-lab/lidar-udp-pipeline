#pragma once

#include <cstdint>
#include <vector>
#include <cstddef>

namespace rplidar {

// One decoded measurement: the readable form of one 5-byte record.
struct Measurement {
    float        angle_deg   = 0.0f;   // 0 .. 360
    float        distance_mm = 0.0f;   // 0 means "no echo"
    std::uint8_t quality     = 0;      // 0 .. 63, signal strength
    bool         start       = false;  // true on the first sample of a spin
};

// A full 360-degree rotation is just a list of measurements.
using Scan = std::vector<Measurement>;

class Parser {
public:
    // Feed raw bytes (any size, any alignment). Any complete 360-degree
    // scans found are appended to `out`. Returns how many were appended.
    std::size_t feed(const std::uint8_t* data, std::size_t len,
                     std::vector<Scan>& out);

private:
    std::vector<std::uint8_t> buf_;            // bytes received, not yet parsed
    Scan                      current_;        // the scan being assembled now
};

} // namespace rplidar