#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace motor_calibration {
// Generation and runtime must address the table in the same raw encoder frame.
constexpr std::size_t lut_index(std::uint16_t raw) { return raw >> 3U; }

inline int circular_error(int measured, int expected, int period) {
    int error = (measured - expected) % period;
    if (error > period / 2) error -= period;
    if (error < -period / 2) error += period;
    return error;
}

inline int electrical_offset_mean(const int* samples, int count, int electrical_period) {
    const int anchor = samples[0];
    std::int64_t sum = 0;
    for (int i = 0; i < count; ++i)
        sum += circular_error(samples[i], anchor, electrical_period);
    return anchor + static_cast<int>(sum / count);
}

template<std::size_t N>
bool monotonic_lut(const std::array<int, N>& lut, int cpr) {
    for (std::size_t i = 0; i < N; ++i) {
        const auto next = (i + 1U) % N;
        const int raw_step = next == 0 ? cpr - static_cast<int>(i * 8U) : 8;
        const std::int64_t corrected_step = static_cast<std::int64_t>(raw_step) + lut[i] - lut[next];
        if (corrected_step <= 0 || lut[i] < -cpr / 2 || lut[i] > cpr / 2) return false;
    }
    return true;
}
}
