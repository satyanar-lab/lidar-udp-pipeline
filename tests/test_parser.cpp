#include "rplidar_parser.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

namespace {

int g_failures = 0;

void expect(bool ok, const char* what) {
    if (!ok) {
        std::printf("  FAIL: %s\n", what);
        ++g_failures;
    }
}

bool approx(float a, float b) {            // never compare floats with ==
    return std::fabs(a - b) < 1e-3f;
}

// Three hand-decoded records, the same arithmetic done by hand against xxd:
//  R0: start=1 q=47 angle=0.0   dist=2000   (the bd 01 00 40 1f we verified)
//  R1: start=0 q=30 angle=90.0  dist=1000
//  R2: start=1 q=10 angle=180.0 dist=500
const std::uint8_t kThreeRecords[] = {
    0xBD, 0x01, 0x00, 0x40, 0x1F,
    0x7A, 0x01, 0x2D, 0xA0, 0x0F,
    0x29, 0x01, 0x5A, 0xD0, 0x07,
};

void test_decode_and_segmentation() {
    std::printf("decode + scan segmentation\n");

    rplidar::Parser parser;
    std::vector<rplidar::Scan> out;
    std::size_t completed = parser.feed(kThreeRecords, sizeof(kThreeRecords), out);

    expect(completed == 1,  "one scan completed");
    expect(out.size() == 1, "one scan emitted");
    if (out.size() == 1 && out[0].size() == 2) {
        const rplidar::Scan& s = out[0];
        expect(s[0].start,                         "p0 start flag set");
        expect(s[0].quality == 47,                 "p0 quality 47");
        expect(approx(s[0].angle_deg, 0.0f),       "p0 angle 0");
        expect(approx(s[0].distance_mm, 2000.0f),  "p0 dist 2000");
        expect(!s[1].start,                        "p1 start flag clear");
        expect(approx(s[1].angle_deg, 90.0f),      "p1 angle 90");
        expect(approx(s[1].distance_mm, 1000.0f),  "p1 dist 1000");
    } else {
        expect(false, "scan should contain exactly R0 and R1");
    }
}

void test_resync_after_garbage() {
    std::printf("resynchronise after garbage\n");

    // 3 junk bytes (0x00 fails the validity check at every offset),
    // then two start records -> exactly one clean scan must survive.
    const std::uint8_t bytes[] = {
        0x00, 0x00, 0x00,
        0xBD, 0x01, 0x00, 0x40, 0x1F,
        0x29, 0x01, 0x5A, 0xD0, 0x07,
    };

    rplidar::Parser parser;
    std::vector<rplidar::Scan> out;
    std::size_t completed = parser.feed(bytes, sizeof(bytes), out);

    expect(completed == 1,  "one scan completed despite junk prefix");
    expect(out.size() == 1, "one scan emitted");
    if (out.size() == 1 && out[0].size() == 1) {
        expect(approx(out[0][0].distance_mm, 2000.0f),
               "recovered record decodes correctly");
    } else {
        expect(false, "scan should hold exactly the one pre-resync record");
    }
}

void test_chunk_boundary_independence() {
    std::printf("chunk-boundary independence (1 byte per feed call)\n");

    rplidar::Parser parser;
    std::vector<rplidar::Scan> out;
    std::size_t completed = 0;
    for (std::size_t i = 0; i < sizeof(kThreeRecords); ++i) {
        completed += parser.feed(kThreeRecords + i, 1, out);
    }

    expect(completed == 1,  "same one scan, regardless of chunking");
    expect(out.size() == 1, "same single scan emitted");
    if (out.size() == 1 && out[0].size() == 2) {
        expect(approx(out[0][0].distance_mm, 2000.0f) &&
               approx(out[0][1].distance_mm, 1000.0f),
               "identical decode across split feed() calls");
    } else {
        expect(false, "chunked feeding must yield the same scan");
    }
}

} // namespace

int main() {
    test_decode_and_segmentation();
    test_resync_after_garbage();
    test_chunk_boundary_independence();

    if (g_failures == 0) {
        std::printf("ALL TESTS PASSED\n");
        return 0;
    }
    std::printf("%d CHECK(S) FAILED\n", g_failures);
    return 1;
}