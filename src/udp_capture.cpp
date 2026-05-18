// udp_capture.cpp - protocol-agnostic UDP -> file recorder.
//
// A pure byte mover. It deliberately knows nothing about the
// RPLIDAR format. Its only job is to produce a deterministic capture
// file that the parser and profiler replay. Replaying that
// file is equivalent to running live on the socket.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

namespace { volatile std::sig_atomic_t g_stop = 0; }

extern "C" void on_sigint(int /*sig*/) { g_stop = 1; }

int main(int argc, char** argv) {
    const char*   bind_ip  = "127.0.0.1";
    std::uint16_t port     = 9000;
    std::string   out_path = "captures/scan.bin";
    if (argc > 1) out_path = argv[1];
    if (argc > 2) port = static_cast<std::uint16_t>(std::stoi(argv[2]));

    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) { 
        std::perror("socket"); 
        return 1; }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port);
    if (::inet_pton(AF_INET, bind_ip, &addr.sin_addr) != 1) {
        std::cerr << "bad bind ip\n"; 
        ::close(fd); 
        return 1;
    }
    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::perror("bind"); 
        ::close(fd); 
        return 1;
    }

    timeval tv{};
    tv.tv_sec  = 1;
    tv.tv_usec = 0;
    if (::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        std::perror("setsockopt(SO_RCVTIMEO)");
        ::close(fd);
        return 1;
    }

    std::ofstream out(out_path, std::ios::binary | std::ios::trunc);
    if (!out) { 
        std::cerr << "cannot open " << out_path << "\n";
        ::close(fd); 
        return 1; 
    }

    std::signal(SIGINT, on_sigint);
    std::cout << "capturing udp " << bind_ip << ":" << port
              << " -> " << out_path << "  (Ctrl+C to stop)\n";

    std::uint8_t  buf[2048];
    std::uint64_t total_bytes = 0, datagrams = 0;
    auto last_report = std::chrono::steady_clock::now();

    while (!g_stop) {
        ssize_t n = ::recvfrom(fd, buf, sizeof(buf), 0, nullptr, nullptr);
        if (n < 0) {
            if (errno == EINTR ||
                errno == EAGAIN || errno == EWOULDBLOCK) continue; // benign: signal or timeout
            std::perror("recvfrom"); 
            break;
        }
        if (n == 0) continue;

        out.write(reinterpret_cast<const char*>(buf), n);
        total_bytes += static_cast<std::uint64_t>(n);
        ++datagrams;

        auto now = std::chrono::steady_clock::now();
        if (now - last_report > std::chrono::seconds(1)) {
            std::cout << "\r" << datagrams << " datagrams, "
                      << total_bytes << " bytes" << std::flush;
            last_report = now;
        }
    }
    
    out.flush(); out.close(); ::close(fd);
    std::cout << "\nstopped: " << datagrams << " datagrams, "
              << total_bytes << " bytes -> " << out_path << "\n";
    return 0;
}




