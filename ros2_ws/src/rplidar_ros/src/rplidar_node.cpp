#include "rplidar_parser.hpp"

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
#include <unistd.h>

class RplidarNode : public rclcpp::Node {
public:
    RplidarNode() : rclcpp::Node("rplidar_node") {
        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        open_udp_socket("127.0.0.1", 9000);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            [this]() { poll(); });
        RCLCPP_INFO(get_logger(), "rplidar_node started, publishing on /scan");
    }

    ~RplidarNode() {
        if (sock_fd_ >= 0)
            ::close(sock_fd_);
    }

private:
    int sock_fd_ = -1;
    std::uint64_t published_ = 0;
    rplidar::Parser parser_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void open_udp_socket(const char* ip, std::uint16_t port) {
        sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_fd_ < 0)
            throw std::runtime_error(std::string("socket: ") + std::strerror(errno));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(port);
        if (::inet_pton(AF_INET, ip, &addr.sin_addr) != 1) {
            ::close(sock_fd_);
            sock_fd_ = -1;
            throw std::runtime_error("bad bind ip");
        }
        if (::bind(sock_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            std::string e = std::strerror(errno);
            ::close(sock_fd_);
            sock_fd_ = -1;
            throw std::runtime_error("bind: " + e);
        }

        // non-blocking: a ROS timer callback must never wait on the socket
        int flags = ::fcntl(sock_fd_, F_GETFL, 0);
        ::fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK);

        RCLCPP_INFO(get_logger(), "listening for UDP on %s:%u", ip, port);
    }

    void poll() {
        std::uint8_t buf[2048];
        std::vector<rplidar::Scan> scans;

        while (true) {
            ssize_t n = ::recvfrom(sock_fd_, buf, sizeof(buf), 0, nullptr, nullptr);
            if (n < 0) {
                if (errno == EINTR) continue;                        // retry
                if (errno == EAGAIN || errno == EWOULDBLOCK) break;  // drained
                RCLCPP_WARN(get_logger(), "recvfrom: %s", std::strerror(errno));
                break;
            }
            if (n == 0) continue;                                    // empty datagram
            parser_.feed(buf, static_cast<std::size_t>(n), scans);
        }

        for (rplidar::Scan& s : scans)
            publish_scan(s);
    }

    void publish_scan(const rplidar::Scan& s);
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
    msg.range_min       = 0.05f;     // metres
    msg.range_max       = 12.0f;     // metres

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

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RplidarNode>());
    rclcpp::shutdown();
    return 0;
}