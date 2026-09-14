#pragma once

#include <cstdint>

namespace ruka3::protocol {

inline constexpr std::uint16_t kHeartbeatSubject = 7509U;
inline constexpr std::uint16_t kMitCommandBase = 2107U;
inline constexpr std::uint16_t kSpecificControlBase = 3407U;
inline constexpr std::uint32_t kNetworkTimeoutMs = 3000U;
inline constexpr std::uint32_t kBootRequestMagic = 0xB00710ADUL;

}  // namespace ruka3::protocol
