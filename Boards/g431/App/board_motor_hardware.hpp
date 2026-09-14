#pragma once

#include "interfaces/motor_hardware.hpp"

namespace ruka3::g431 {

class BoardMotorHardware final : public MotorHardware {
public:
    void force_safe() override;
    bool calibration_valid() const override;
    MotorTelemetry telemetry() const override;
};

}  // namespace ruka3::g431
