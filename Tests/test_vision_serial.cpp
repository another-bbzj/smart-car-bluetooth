#include "../Tools/vision_serial.hpp"
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <poll.h>
#include <unistd.h>

int main()
{
    int master = posix_openpt(O_RDWR | O_NOCTTY);
    assert(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0);
    const std::string device = ptsname(master);
    {
        visionlink::Serial serial(device);
        auto frame = visionlink::track(123, 1, true, 1, 5);
        assert(serial.send(frame));
        pollfd ready{master, POLLIN, 0};
        assert(poll(&ready, 1, 1000) == 1);
        unsigned char buffer[64];
        const auto count = read(master, buffer, sizeof(buffer));
        assert(count == static_cast<ssize_t>(frame.size()));
        assert(std::equal(frame.begin(), frame.end(), buffer));
        bool rejected = false;
        try { visionlink::Serial duplicate(device); } catch (const std::exception&) { rejected = true; }
        assert(rejected);
        close(master);
        bool disconnected = false;
        try { (void)serial.send(frame); } catch (const std::exception&) { disconnected = true; }
        assert(disconnected);
    }
    bool rejected = false;
    try { visionlink::Serial missing("/does/not/exist"); } catch (const std::exception&) { rejected = true; }
    assert(rejected);
    std::cout << "serial PTY transport: all checks passed\n";
}
