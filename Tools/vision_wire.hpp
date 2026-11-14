#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

// F407 vision v1: type 3 carries the original RIGHT-positive pursuit curvature.
namespace visionlink {
using Bytes = std::vector<std::uint8_t>;
inline void u16(Bytes& b, unsigned value) {
    b.push_back(static_cast<std::uint8_t>(value >> 8));
    b.push_back(static_cast<std::uint8_t>(value));
}
inline std::uint16_t crc(const Bytes& b, std::size_t offset) {
    std::uint16_t value = 0xffff;
    for (std::size_t i = offset; i < b.size(); ++i) {
        value ^= static_cast<std::uint16_t>(b[i] << 8);
        for (int bit = 0; bit < 8; ++bit)
            value = static_cast<std::uint16_t>((value << 1) ^ ((value & 0x8000) ? 0x1021 : 0));
    }
    return value;
}
inline Bytes frame(unsigned type, std::uint32_t session, std::uint16_t sequence,
                   const Bytes& payload, unsigned age) {
    if (!session) throw std::invalid_argument("vision session must be nonzero");
    Bytes b{0xa5, 0x5a, 1, static_cast<std::uint8_t>(type)};
    u16(b, static_cast<unsigned>(payload.size())); u16(b, sequence);
    u16(b, session >> 16); u16(b, session); u16(b, age);
    b.insert(b.end(), payload.begin(), payload.end());
    u16(b, crc(b, 2));
    return b;
}
inline Bytes hello(std::uint32_t session) { return frame(0, session, 0, {}, 0); }
inline Bytes track(std::uint32_t session, std::uint16_t sequence, bool valid,
                   double curvatureRight, unsigned ageMs) {
    valid = valid && std::isfinite(curvatureRight) && std::abs(curvatureRight) <= 20.0 && ageMs < 100;
    Bytes payload;
    u16(payload, valid ? 4 : 0); // Only pursuit curvature is provided, not cross-track error.
    u16(payload, valid ? 1000 : 0); // Binary quality gate; NOT a calibrated probability.
    u16(payload, 0); u16(payload, 0); // Lateral/heading unavailable.
    const int curvature = valid ? static_cast<int>(std::lround(curvatureRight * 1000.0)) : 0;
    u16(payload, static_cast<std::uint16_t>(curvature));
    payload.push_back(0); payload.push_back(255); // No obstacle classification or tag yet.
    u16(payload, 0); u16(payload, 0);
    return frame(3, session, sequence, payload, std::min(ageMs, 200U));
}
} // namespace visionlink
