#pragma once

#include <cstdint>

namespace ruka3::protocol {

inline constexpr std::uint16_t kHeartbeatSubject = 7509U;
inline constexpr std::uint32_t kNetworkTimeoutMs = 3000U;
inline constexpr std::uint32_t kBootRequestMagic = 0xB00710ADUL;

// Existing VBBoot interprets this backup word using an 80 MHz FDCAN clock:
// nominal 1 * (1+55+24) -> 1 Mbit/s; data 2 * (1+5+4) -> 4 Mbit/s.
// Preserve its wire ABI so CAN-only application updates can request 1M/4M
// even while the installed bootloader still has legacy 1M/8M defaults.
constexpr std::uint32_t boot_transport_1m4(std::uint16_t node_id) {
    return (static_cast<std::uint32_t>(node_id) & 0x7FFU) |
           (1U << 11U) | (2U << 19U) | (1U << 27U) | (1U << 28U);
}

}  // namespace ruka3::protocol
