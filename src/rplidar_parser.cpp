#include "rplidar_parser.hpp"

#include <cstddef>
#include <cstdint>
#include <utility>

namespace rplidar {
namespace {

// Decode exactly one 5-byte RPLIDAR record into a Measurement.
// Same bit layout as the Python encoder, reversed.
Measurement decode_record(const std::uint8_t* p) {
    Measurement m;

    std::uint8_t  byte0       = p[0];
    std::uint16_t angle_field = static_cast<std::uint16_t>(p[1] | (p[2] << 8));
    std::uint16_t dist_q2     = static_cast<std::uint16_t>(p[3] | (p[4] << 8));

    m.start   = (byte0 & 0x1) != 0;
    m.quality = static_cast<std::uint8_t>(byte0 >> 2);

    std::uint16_t angle_q6 = static_cast<std::uint16_t>(angle_field >> 1);
    m.angle_deg   = angle_q6 / 64.0f;
    m.distance_mm = dist_q2  / 4.0f;

    return m;
}

// Structural sanity check: do these 5 bytes look like a real record?
// byte0: the start flag and its inverse must disagree (S != !S).
// byte1: bit 0 is the check bit, always 1.
bool looks_valid(const std::uint8_t* p) {
    bool s     = ( p[0]       & 0x1) != 0;
    bool inv_s = ((p[0] >> 1) & 0x1) != 0;
    bool check = ( p[1]       & 0x1) != 0;
    return (s != inv_s) && check;
}

} // anonymous namespace

std::size_t Parser::feed(const std::uint8_t* data, std::size_t len,
                         std::vector<Scan>& out) {
    constexpr std::size_t kScanReserve = 512;   // a rotation is ~360 pts;
                                                // round up so framing jitter
                                                // never triggers a realloc
    buf_.insert(buf_.end(), data, data + len);

    std::size_t completed = 0;
    std::size_t pos = 0;                        // read cursor into buf_

    while (buf_.size() - pos >= 5) {
        const std::uint8_t* p = buf_.data() + pos;

        if (!looks_valid(p)) {
            ++pos;                              // misaligned: skip 1, retry
            continue;
        }

        Measurement m = decode_record(p);
        pos += 5;                               // consumed one record

        if (m.start) {
            if (!current_.empty()) {            // a full rotation just ended
                out.push_back(std::move(current_));
                current_.clear();
                ++completed;
            }
            current_.reserve(kScanReserve);     // one allocation per scan,
            current_.push_back(m);              // not nine
        } else if (!current_.empty()) {
            current_.push_back(m);              // continue the open rotation
        }
        // a non-start record with no open scan = leading partial -> dropped
    }

    buf_.erase(buf_.begin(), buf_.begin() + pos);  // drop consumed prefix once
    return completed;
}

} // namespace rplidar