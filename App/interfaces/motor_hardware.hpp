#pragma once

#include <cstdint>

namespace ruka3 {

struct MotorTelemetry {
    float position{};
    float velocity{};
    float torque{};
    float bus_voltage{};
    float bus_current{};
    float electric_angle{};
    std::uint32_t rotor_encoder{};
    std::uint32_t output_encoder{};
};

class MotorHardware {
public:
    virtual ~MotorHardware() = default;
    virtual void force_safe() = 0;
    virtual bool calibration_valid() const = 0;
    virtual MotorTelemetry telemetry() const = 0;
};

}  // namespace ruka3
