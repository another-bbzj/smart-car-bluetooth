#include "../Tools/vision_wire.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

int main()
{
    const unsigned char expected[] = {
        0xa5,0x5a,0x01,0x03,0x00,0x10,0x00,0x01,0x12,0x34,0x56,0x78,0x00,0x0c,
        0x00,0x04,0x03,0xe8,0,0,0,0,0x07,0xd0,0,0xff,0,0,0,0,0xe6,0x7e
    };
    auto bytes = visionlink::track(0x12345678, 1, true, 2.0, 12);
    assert(bytes == visionlink::Bytes(std::begin(expected), std::end(expected)));
    assert(visionlink::hello(123).size() == 16);
    for (auto curvature : {21.0, -21.0, std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity()})
    {
        auto invalid = visionlink::track(123, 1, true, curvature, 1);
        assert(invalid[14] == 0 && invalid[15] == 0);
    }
    for (unsigned age : {100U, 200U, 70000U})
    {
        auto stale = visionlink::track(123, 1, true, 0, age);
        assert(stale[15] == 0 && stale[12] == 0 && stale[13] <= 200);
    }
    auto lost = visionlink::track(123, 0, false, 0, 0);
    assert(lost[15] == 0 && lost[17] == 0 && lost[6] == 0 && lost[7] == 0);
    bool rejected = false;
    try { (void)visionlink::hello(0); } catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
    std::cout << "vision wire protocol: all checks passed\n";
}
