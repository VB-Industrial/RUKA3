#pragma once
#include <cstdint>
struct FDCAN_HandleTypeDef {};
struct FDCAN_ErrorCountersTypeDef {
    std::uint32_t ErrorLogging{}, RxErrorCnt{}, TxErrorCnt{};
};
struct FDCAN_ProtocolStatusTypeDef {
    std::uint32_t LastErrorCode{}, DataLastErrorCode{}, Warning{}, ErrorPassive{}, BusOff{};
};
inline constexpr int HAL_OK = 0;
inline int HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef*, FDCAN_ErrorCountersTypeDef*) { return HAL_OK; }
inline int HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef*, FDCAN_ProtocolStatusTypeDef*) { return HAL_OK; }
// Host-only math tests do not invoke any HAL or critical-section operations.
inline void arm_abs_f32(const float* input, float* output, unsigned count) {
    for (unsigned i = 0; i < count; ++i) output[i] = input[i] < 0.0F ? -input[i] : input[i];
}
