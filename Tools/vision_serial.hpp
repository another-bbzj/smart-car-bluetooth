#pragma once
#include "vision_wire.hpp"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace visionlink {
// One writer, fixed 115200 8N1, no application backlog and no blocking tcdrain.
class Serial {
    int fd_ = -1;
public:
    explicit Serial(const std::string& device) {
        fd_ = ::open(device.c_str(), O_WRONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0) throw std::runtime_error("Cannot open vision serial: " + std::string(std::strerror(errno)));
        termios settings{};
        if (flock(fd_, LOCK_EX | LOCK_NB) || tcgetattr(fd_, &settings)) {
            ::close(fd_); fd_ = -1;
            throw std::runtime_error("Vision serial busy or not a tty");
        }
        cfmakeraw(&settings);
        cfsetispeed(&settings, B115200); cfsetospeed(&settings, B115200);
        settings.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS | CSIZE);
        settings.c_cflag |= CS8 | CLOCAL | CREAD;
        if (tcsetattr(fd_, TCSANOW, &settings) || tcflush(fd_, TCOFLUSH)) {
            ::close(fd_); fd_ = -1;
            throw std::runtime_error("Cannot configure vision serial 115200 8N1");
        }
    }
    ~Serial() { if (fd_ >= 0) ::close(fd_); }
    Serial(const Serial&) = delete;
    Serial& operator=(const Serial&) = delete;
    bool send(const Bytes& bytes) {
        int queued = 0;
        if (ioctl(fd_, TIOCOUTQ, &queued)) throw std::runtime_error("Vision serial unavailable");
        if (queued) return false; // Drop this observation instead of queuing stale frames.
        const auto count = ::write(fd_, bytes.data(), bytes.size());
        if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) return false;
        if (count != static_cast<ssize_t>(bytes.size()))
            throw std::runtime_error("Vision serial short/failed write; sender stopped");
        return true;
    }
};
} // namespace visionlink
