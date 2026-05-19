#include "rplidar_parser.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

namespace {
// RPLIDAR A1 is 115200; A2/A3 use non-standard rates (e.g. 256000) that
// need custom-baud termios2 and are intentionally out of scope here.
speed_t baud_to_speed(int baud) {
    switch (baud) {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
        case 230400:  return B230400;
        case 460800:  return B460800;
        case 921600:  return B921600;
        case 1000000: return B1000000;
        default:
            throw std::runtime_error("unsupported serial baud (use 115200 "
                "for RPLIDAR A1; non-standard rates need termios2)");
    }
}
} // namespace

class RplidarNode : public rclcpp::Node {
public:
    RplidarNode() : rclcpp::Node("rplidar_node") {
        scan_pub_  = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);

        const std::string source = declare_parameter<std::string>("source", "udp");
        if (source == "serial") {
            const std::string port = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
            const int baud         = declare_parameter<int>("serial_baud", 115200);
            open_serial(port, baud);
        } else {
            open_udp_socket("127.0.0.1", 9000);
        }

        timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            [this]() { poll(); });
        RCLCPP_INFO(get_logger(),
                    "rplidar_node started, publishing on /scan and /scan_cloud");
    }

    ~RplidarNode() {
        if (fd_ >= 0)
            ::close(fd_);
    }

private:
    int fd_ = -1;
    std::uint64_t published_ = 0;
    rplidar::Parser parser_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void open_udp_socket(const char* ip, std::uint16_t port) {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0)
            throw std::runtime_error(std::string("socket: ") + std::strerror(errno));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(port);
        if (::inet_pton(AF_INET, ip, &addr.sin_addr) != 1) {
            ::close(fd_);
            fd_ = -1;
            throw std::runtime_error("bad bind ip");
        }
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            std::string e = std::strerror(errno);
            ::close(fd_);
            fd_ = -1;
            throw std::runtime_error("bind: " + e);
        }

        int flags = ::fcntl(fd_, F_GETFL, 0);
        ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

        RCLCPP_INFO(get_logger(), "listening for UDP on %s:%u", ip, port);
    }

    void open_serial(const std::string& port, int baud) {
        const speed_t sp = baud_to_speed(baud);   // validate before opening

        fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0)
            throw std::runtime_error("open " + port + ": " + std::strerror(errno));

        termios tio{};
        if (::tcgetattr(fd_, &tio) != 0) {
            std::string e = std::strerror(errno);
            ::close(fd_); fd_ = -1;
            throw std::runtime_error("tcgetattr: " + e);
        }

        // raw 8N1, no flow control, non-blocking reads
        tio.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR
                         | BRKINT | ISTRIP);
        tio.c_oflag &= ~OPOST;
        tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | IEXTEN);
        tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
        tio.c_cflag |= (CS8 | CLOCAL | CREAD);
        tio.c_cc[VMIN]  = 0;
        tio.c_cc[VTIME] = 0;

        ::cfsetispeed(&tio, sp);
        ::cfsetospeed(&tio, sp);

        if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
            std::string e = std::strerror(errno);
            ::close(fd_); fd_ = -1;
            throw std::runtime_error("tcsetattr: " + e);
        }

        RCLCPP_INFO(get_logger(), "reading serial %s @ %d 8N1",
                    port.c_str(), baud);
    }

    void poll() {
        std::uint8_t buf[2048];
        std::vector<rplidar::Scan> scans;

        // Bounded drain: a faster-than-realtime source (e.g. a replay
        // firehose) must never trap one timer tick. Read a fixed budget,
        // publish, and let the next tick continue.
        constexpr int kMaxReadsPerTick = 64;
        for (int i = 0; i < kMaxReadsPerTick; ++i) {
            ssize_t n = ::read(fd_, buf, sizeof(buf));
            if (n < 0) {
                if (errno == EINTR) continue;
                if (errno == EAGAIN || errno == EWOULDBLOCK) break;  // drained
                RCLCPP_WARN(get_logger(), "read: %s", std::strerror(errno));
                break;
            }
            if (n == 0) break;                                       // EOF
            parser_.feed(buf, static_cast<std::size_t>(n), scans);
        }

        for (rplidar::Scan& s : scans) {
            publish_scan(s);
            publish_cloud(s);
        }
    }

    void publish_scan(const rplidar::Scan& s);
    void publish_cloud(const rplidar::Scan& s);
};

void RplidarNode::publish_scan(const rplidar::Scan& s) {
    constexpr int   N  = 360;
    constexpr float PI = 3.14159265358979f;

    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp    = now();
    msg.header.frame_id = "laser";

    msg.angle_min       = 0.0f;
    msg.angle_max       = 2.0f * PI;
    msg.angle_increment = (2.0f * PI) / static_cast<float>(N);
    msg.range_min       = 0.05f;
    msg.range_max       = 12.0f;

    msg.ranges.assign(N, std::numeric_limits<float>::infinity());

    for (const rplidar::Measurement& m : s) {
        int bin = static_cast<int>(m.angle_deg / 360.0f * N + 0.5f) % N;
        msg.ranges[bin] = m.distance_mm / 1000.0f;   // mm -> m
    }

    scan_pub_->publish(msg);

    if (++published_ % 5 == 0) {
        float r0 = msg.ranges.empty() ? -1.0f : msg.ranges[0];
        RCLCPP_INFO(get_logger(),
                    "published scan #%llu  points=%zu  range[0]=%.2f m",
                    static_cast<unsigned long long>(published_),
                    s.size(), static_cast<double>(r0));
    }
}

void RplidarNode::publish_cloud(const rplidar::Scan& s) {
    constexpr float PI = 3.14159265358979f;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp    = now();
    cloud.header.frame_id = "laser";
    cloud.height          = 1;
    cloud.is_dense        = false;

    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(s.size());

    sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(cloud, "z");

    for (const rplidar::Measurement& m : s) {
        const float th = m.angle_deg * (PI / 180.0f);
        const float r  = m.distance_mm / 1000.0f;   // mm -> m
        *ix = r * std::cos(th);
        *iy = r * std::sin(th);
        *iz = 0.0f;
        ++ix; ++iy; ++iz;
    }

    cloud_pub_->publish(cloud);

    if (!s.empty() && published_ % 5 == 0) {
        const float th = s.front().angle_deg * (PI / 180.0f);
        const float r  = s.front().distance_mm / 1000.0f;
        RCLCPP_INFO(get_logger(),
                    "cloud: %zu points  first=(%.2f, %.2f) m",
                    s.size(),
                    static_cast<double>(r * std::cos(th)),
                    static_cast<double>(r * std::sin(th)));
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RplidarNode>());
    rclcpp::shutdown();
    return 0;
}