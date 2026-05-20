# lidar-udp-pipeline

A from-scratch RPLIDAR data pipeline in C++ and Python: a byte-faithful UDP
sensor stream, a dependency-free C++ parser library that decodes the RPLIDAR
wire format, an offline replay/verifier, and a thin ROS 2 node that
republishes the decoded scans as `sensor_msgs/LaserScan` and
`sensor_msgs/PointCloud2`.

The project is built around one principle: **replay must equal live**. The
same bytes flow through the same parser whether they come from a recorded
file, a UDP socket, or a serial device, and every decoded value is checked by
hand against known geometry rather than assumed correct.

## What it does

- **`sim/rplidar_sim.py`** — emits a synthetic RPLIDAR scan stream over UDP,
  byte-for-byte in the documented RPLIDAR 5-byte measurement format. It models
  a known room (a 4 m × 6 m rectangle with an 800 mm pillar) so every decoded
  distance has a value that can be checked against geometry.
- **`include/rplidar_parser.hpp` / `src/rplidar_parser.cpp`** — a standalone,
  ROS-free C++ library that turns a raw byte stream into structured scans. It
  tolerates partial and corrupt input: bad records are caught by their
  validity bits and the parser resynchronises rather than desyncing
  permanently.
- **`src/udp_capture.cpp`** — a protocol-agnostic UDP-to-file recorder. It
  knows nothing about RPLIDAR; it just preserves bytes, so a capture replays
  identically to the live stream.
- **`src/parse_file.cpp`** — an offline driver that runs the parser over a
  capture and reports scan count, the first measurement, and min/max range,
  for verification against the modelled geometry.
- **`ros2_ws/src/rplidar_ros/`** — a thin ROS 2 (Jazzy) node that reuses the
  parser **unchanged**, reads the live stream, and publishes
  `sensor_msgs/LaserScan` on `/scan`.
- The node also publishes `sensor_msgs/PointCloud2` on `/scan_cloud`
  (polar→Cartesian) alongside `LaserScan`, and reads from **UDP** (default)
  or a **serial** device (POSIX termios, raw 8N1) selected by a `source`
  parameter.

## Architecture

The parser is a pure library with no ROS, no sockets, and no I/O of its own —
it consumes bytes and produces scans. The capture tool and the ROS 2 node are
thin adapters around it: one feeds it a file, the others feed it a socket or
a serial port. That is why the offline path and the live paths are guaranteed
to behave identically, and why a real RPLIDAR (over USB serial, or a
serial-to-UDP bridge) is a drop-in byte-source swap with no parser changes.
The node also bounds the bytes it reads per timer tick, so it publishes at a
steady cadence and stays responsive regardless of how fast the source
delivers data.

Correctness is established by decoding, not by trust. With the simulator's
modelled room, the decoded stream is verified to show:

- ~2.00 m straight ahead (0°) — the near wall at 2000 mm,
- ~0.80 m across roughly 25°–35° — the modelled pillar,
- a maximum near 3.58 m around 57° — the far corner of the rectangle
  (√(2000² + 3000²) ≈ 3.6 m).

These values are read out of the live data, checked against the geometry by
hand, and frozen into the test suite so they cannot silently regress.

## Performance

The parser's buffering was written the simple, obviously-correct way first,
then optimised under measurement — not guesswork. `src/bench_parser.cpp`
replays a capture through the parser in the real small-chunk call pattern;
callgrind located the hotspots, wall-clock quantified them.

| Stage | Throughput | Instructions (callgrind) |
|---|---|---|
| Baseline | 200 MB/s | 163.1 M |
| Remove O(n) front-of-buffer erase | 220 MB/s | 90.1 M |
| Reserve the scan vector to known rotation size | 287 MB/s | 67.6 M |

Net: +43% throughput, -59% instructions, output bit-identical throughout
(216,000 scans, unchanged). Optimisation was deliberately stopped there: the
dominant remaining cost is the irreducible per-record decode itself, not a
fixable inefficiency.

## Build

C++ pipeline (plain CMake):

```bash
cmake -S . -B build
cmake --build build
```

ROS 2 node:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rplidar_ros
source install/setup.bash
```

## Run

Start the simulated sensor (streams UDP to `127.0.0.1:9000`):

```bash
python3 sim/rplidar_sim.py
```

Offline path — record, then replay through the parser:

```bash
./build/udp_capture
./build/parse_file captures/scan.bin
```

Benchmark the parser and run the test suite:

```bash
./build/bench_parser captures/scan.bin 2000
ctest --test-dir build --output-on-failure
```

Live path — the ROS 2 node, with the simulator running:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run rplidar_ros rplidar_node
```

The node logs a throttled heartbeat as it publishes, e.g.
`published scan #50  points=360  range[0]=2.00 m` and
`cloud: 360 points  first=(2.00, 0.00) m`.

![Live RPLIDAR pipeline: the ROS 2 node decoding the live stream and publishing LaserScan](assets/live_pipeline.png)

*The node running against the live simulator. Each heartbeat is a complete
360-point scan, and `range[0]` reads 2.00 m — the 2000 mm wall at 0°,
matching the modelled room geometry. The parser producing this is the same
source compiled, unchanged, into the offline tools.*

To read from a serial device instead of UDP, set the `source` parameter:

```bash
ros2 run rplidar_ros rplidar_node --ros-args \
  -p source:=serial -p serial_port:=/dev/ttyUSB0 -p serial_baud:=115200
```

With no physical sensor, the serial path can be exercised by feeding a
capture into a pseudo-terminal (`socat`) and pointing `serial_port` at it.

## Status and limitations

Stated plainly, because the point of the project is that the claims are
checkable:

- **The sensor is simulated, not physical.** The simulator is byte-faithful
  to the documented RPLIDAR wire format, and the architecture treats the
  device as an interchangeable byte source, but no physical RPLIDAR has been
  on the other end.
- **The serial source is verified via a virtual-serial (pty) loopback, not a
  physical RPLIDAR.** It uses real POSIX `open`/termios (raw 8N1) and is
  selected with the `source` parameter; it was verified end-to-end over a
  `socat` pty fed the real captured wire bytes, with `LaserScan` and
  `PointCloud2` streaming continuously and correctly. No physical RPLIDAR was
  on hand; non-standard rates (e.g. 256000) need custom-baud `termios2` and
  are out of scope.
- **Cross-process ROS 2 introspection does not work under WSL2.**
  `ros2 topic echo` and RViz rely on inter-process DDS discovery, which the
  WSL2 network stack does not support even in mirrored mode (the Hyper-V
  firewall blocks DDS discovery traffic) — a documented WSL2 limitation,
  independent of this code. The node is verified directly via its own
  in-process scan logging. On native Linux it publishes standard messages and
  is visualisable in RViz like any other laser source.

## Layout

```
sim/rplidar_sim.py          synthetic RPLIDAR UDP stream
src/udp_capture.cpp         protocol-agnostic UDP -> file recorder
src/rplidar_parser.cpp      parser library implementation
include/rplidar_parser.hpp  parser library public header
src/parse_file.cpp          offline replay / verifier
src/bench_parser.cpp        parser micro-benchmark harness
tests/test_parser.cpp       dependency-free CTest suite
CMakeLists.txt              plain CMake build for the above
ros2_ws/src/rplidar_ros/    ROS 2 (Jazzy) node reusing the parser unchanged
captures/                   capture fixtures (binary, gitignored)
assets/                     screenshots used in this README
```
