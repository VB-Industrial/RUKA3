#pragma once

#include <array>
#include <cstdint>

namespace ruka3 {

// Physical ADC/sensor conversion, independent of the motor's direction and
// the public joint coordinate. Calibration offsets are ADC input volts.
struct PhaseCurrentSense {
    float adc_reference_v;
    float adc_resolution_counts;
    float oversampling_ratio;
    float volts_per_amp;
    float bus_voltage_gain;
    std::array<std::int8_t, 3> phase_sign;
    std::uint16_t offset_samples;

    constexpr float adc_voltage(std::uint32_t raw) const {
        return static_cast<float>(raw) * adc_reference_v /
               (adc_resolution_counts * oversampling_ratio);
    }
    constexpr float phase_current(std::uint32_t raw, float offset_v, unsigned phase) const {
        return static_cast<float>(phase_sign[phase]) * (adc_voltage(raw) - offset_v) / volts_per_amp;
    }
};

} // namespace ruka3
