#pragma once
// Single-client OpenCV TCP image streaming server for Linux / Orange Pi.
// Client sends: "RVI1" + uint16_be UTF8_window_name_length + UTF8_window_name
// Server sends: "FRM1" + uint32_be jpeg_length + JPEG bytes, repeated.
// Usage: TcpImshowServer remote(9000); remote.imshow("Camera", frame);
// Compile: g++ -std=c++17 main.cpp -o app $(pkg-config --cflags --libs opencv4) -pthread

#include <opencv2/opencv.hpp>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

class TcpImshowServer {
public:
    explicit TcpImshowServer(uint16_t port = 9000, int jpegQuality = 75,
                             const std::string& bindIp = "0.0.0.0")
        : port_(port), jpegQuality_(jpegQuality), listeningFd_(-1), clientFd_(-1) {
        if (jpegQuality_ < 1 || jpegQuality_ > 100) {
            throw std::invalid_argument("jpegQuality must be 1..100");
        }
        listeningFd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (listeningFd_ < 0) throw std::runtime_error("socket() failed");

        int reuse = 1;
        ::setsockopt(listeningFd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port_);
        if (::inet_pton(AF_INET, bindIp.c_str(), &addr.sin_addr) != 1) {
            ::close(listeningFd_);
            listeningFd_ = -1;
            throw std::invalid_argument("Invalid TCP bind IPv4 address: " + bindIp);
        }
        if (::bind(listeningFd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0 ||
            ::listen(listeningFd_, 4) < 0) {
            int err = errno;
            ::close(listeningFd_);
            listeningFd_ = -1;
            throw std::runtime_error("bind/listen failed (port in use?): " + std::to_string(err));
        }
        worker_ = std::thread(&TcpImshowServer::serve, this);
        std::cout << "[TCP imshow] Listening on " << bindIp << ":" << port_ << std::endl;
    }

    TcpImshowServer(const TcpImshowServer&) = delete;
    TcpImshowServer& operator=(const TcpImshowServer&) = delete;

    ~TcpImshowServer() {
        stop_.store(true);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (clientFd_ >= 0) ::shutdown(clientFd_, SHUT_RDWR);
        }
        condition_.notify_all();
        if (worker_.joinable()) worker_.join();
        if (listeningFd_ >= 0) ::close(listeningFd_);
    }

    // Drop-in logical replacement for cv::imshow(name, image).
    // Only frames matching the viewer's requested name are copied.
    // Sending + JPEG encoding run in a background worker: slow TCP won't freeze processing.
    void imshow(const std::string& name, const cv::Mat& image) {
        if (image.empty() || stop_.load()) return;
        std::lock_guard<std::mutex> lock(mutex_);
        if (clientFd_ < 0 || name != requestedName_) return;
        latestFrame_ = image.clone(); // Own copy: safe even when caller reuses frame memory.
        frameReady_ = true;
        condition_.notify_one();
    }

private:
    static bool recvExact(int fd, void* target, size_t total) {
        uint8_t* p = static_cast<uint8_t*>(target);
        while (total != 0) {
            ssize_t got = ::recv(fd, p, total, 0);
            if (got <= 0) {
                if (got < 0 && errno == EINTR) continue;
                return false;
            }
            p += got;
            total -= static_cast<size_t>(got);
        }
        return true;
    }

    static bool sendExact(int fd, const void* source, size_t total) {
        const uint8_t* p = static_cast<const uint8_t*>(source);
        while (total != 0) {
            ssize_t sent = ::send(fd, p, total, MSG_NOSIGNAL);
            if (sent <= 0) {
                if (sent < 0 && errno == EINTR) continue;
                return false;
            }
            p += sent;
            total -= static_cast<size_t>(sent);
        }
        return true;
    }

    static bool readClientName(int fd, std::string& name) {
        uint8_t h[6];
        if (!recvExact(fd, h, sizeof(h))) return false;
        if (std::memcmp(h, "RVI1", 4) != 0) return false;
        unsigned nameLength = (static_cast<unsigned>(h[4]) << 8) | h[5];
        if (nameLength < 1 || nameLength > 255) return false;
        name.resize(nameLength);
        return recvExact(fd, &name[0], nameLength);
    }

    void streamOneClient(int fd) {
        timeval readTimeout{3, 0};
        ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &readTimeout, sizeof(readTimeout));
        timeval writeTimeout{2, 0};
        ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &writeTimeout, sizeof(writeTimeout));
        std::string name;
        if (!readClientName(fd, name)) return;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            clientFd_ = fd;
            requestedName_ = name;
            latestFrame_.release();
            frameReady_ = false;
        }
        std::cout << "[TCP imshow] Viewer selected: " << name << std::endl;

        while (!stop_.load()) {
            cv::Mat frame;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait_for(lock, std::chrono::milliseconds(250), [&] {
                    return stop_.load() || frameReady_;
                });
                if (stop_.load()) break;
                if (frameReady_) {
                    frame = std::move(latestFrame_);
                    frameReady_ = false;
                }
            }
            if (frame.empty()) {
                // The client can close while its selected name is not being published.
                // Detect this, or reconnecting with a corrected name would be impossible.
                char probe;
                ssize_t r = ::recv(fd, &probe, 1, MSG_PEEK | MSG_DONTWAIT);
                if (r == 0) break;
                if (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) break;
                continue;
            }

            try {
                cv::Mat jpegImage;
                if (frame.depth() != CV_8U) {
                    frame.convertTo(jpegImage, CV_8U); // display data is normally CV_8U
                } else {
                    jpegImage = frame;
                }
                if (jpegImage.channels() == 4) {
                    cv::cvtColor(jpegImage, jpegImage, cv::COLOR_BGRA2BGR);
                }
                if (jpegImage.channels() != 1 && jpegImage.channels() != 3) continue;
                std::vector<unsigned char> jpeg;
                if (!cv::imencode(".jpg", jpegImage, jpeg,
                                  {cv::IMWRITE_JPEG_QUALITY, jpegQuality_})) continue;
                if (jpeg.empty() || jpeg.size() > 8u * 1024u * 1024u) continue;
                uint32_t n = static_cast<uint32_t>(jpeg.size());
                uint8_t hdr[8] = {'F','R','M','1',
                    static_cast<uint8_t>(n >> 24),
                    static_cast<uint8_t>(n >> 16),
                    static_cast<uint8_t>(n >> 8),
                    static_cast<uint8_t>(n)};
                if (!sendExact(fd, hdr, sizeof(hdr)) ||
                    !sendExact(fd, jpeg.data(), jpeg.size())) break;
            } catch (const cv::Exception& ex) {
                std::cerr << "[TCP imshow] OpenCV encoding error: " << ex.what() << std::endl;
            }
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            clientFd_ = -1;
            requestedName_.clear();
            latestFrame_.release();
            frameReady_ = false;
        }
    }

    void serve() {
        while (!stop_.load()) {
            fd_set set;
            FD_ZERO(&set);
            FD_SET(listeningFd_, &set);
            timeval tv{0, 250000};
            int result = ::select(listeningFd_ + 1, &set, nullptr, nullptr, &tv);
            if (stop_.load()) break;
            if (result < 0) {
                if (errno == EINTR) continue;
                break;
            }
            if (result == 0) continue;
            sockaddr_in peer{};
            socklen_t len = sizeof(peer);
            int fd = ::accept(listeningFd_, reinterpret_cast<sockaddr*>(&peer), &len);
            if (fd < 0) continue;
            std::cout << "[TCP imshow] Windows viewer connected." << std::endl;
            streamOneClient(fd);
            ::close(fd);
            std::cout << "[TCP imshow] Windows viewer disconnected." << std::endl;
        }
    }

    uint16_t port_;
    int jpegQuality_;
    int listeningFd_;
    int clientFd_;  // Only accessed with mutex_ (worker owns close).
    std::atomic<bool> stop_{false};
    std::thread worker_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::string requestedName_;
    cv::Mat latestFrame_;
    bool frameReady_ = false;
};
