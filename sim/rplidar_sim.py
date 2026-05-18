#!/usr/bin/env python3
"""
rplidar_sim.py - synthetic RPLIDAR A-series UDP stream generator.

Hardware-in-the-loop test source: emits the exact 5-byte measurement
record the real RPLIDAR SDK uses, over UDP, so the C++ parser can be
built and tested with zero hardware.
"""
import socket, struct, time, math

DEST              = ("127.0.0.1", 9000)
SAMPLES_PER_SCAN  = 360      # one measurement per degree
SCAN_HZ           = 5.0      # rotations per second
MEAS_PER_DATAGRAM = 64       # arbitrary: stream is chopped, datagram != scan

def synthetic_distance_mm(angle_deg: float) -> float:
    """Fake 4m x 6m room with a pillar, so captured scans look real-ish."""
    a = math.radians(angle_deg)
    half_w, half_h = 2000.0, 3000.0  # mm
    dx = abs(math.cos(a)) / half_w
    dy = abs(math.sin(a)) / half_h
    dist = 1.0 / max(dx, dy)
    if 25 <= angle_deg <= 35:        # a pillar in front
        dist = min(dist, 800.0)
    return dist

def encode_measurement(angle_deg: float, dist_mm: float, start: bool) -> bytes:
    quality = 47                                  # 0..63, plausible
    s, inv_s = (1, 0) if start else (0, 1)
    byte0 = (quality << 2) | (inv_s << 1) | s

    angle_q6    = int(round(angle_deg * 64.0)) & 0x7FFF
    angle_field = (angle_q6 << 1) | 1             # bit0 = check bit

    dist_q2 = int(round(dist_mm * 4.0)) & 0xFFFF

    # <BHH = little-endian: 1-byte byte0, 2-byte angle_field, 2-byte dist_q2
    return struct.pack("<BHH", byte0, angle_field, dist_q2)

def main():
    sock   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / (SAMPLES_PER_SCAN * SCAN_HZ)   # seconds per sample
    buf    = bytearray()
    print(f"streaming synthetic RPLIDAR -> {DEST[0]}:{DEST[1]} "
          f"({SAMPLES_PER_SCAN} samples/scan @ {SCAN_HZ} Hz)")
    try:
        while True:
            for i in range(SAMPLES_PER_SCAN):
                angle = 360.0 * i / SAMPLES_PER_SCAN
                buf += encode_measurement(angle,
                                          synthetic_distance_mm(angle),
                                          start=(i == 0))
                if len(buf) >= MEAS_PER_DATAGRAM * 5:
                    sock.sendto(bytes(buf), DEST)
                    buf.clear()
                time.sleep(period)
            if buf:
                sock.sendto(bytes(buf), DEST)
                buf.clear()
    except KeyboardInterrupt:
        print("\nstopped")

if __name__ == "__main__":
    main()
