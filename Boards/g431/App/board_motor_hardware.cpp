#include "board_motor_hardware.hpp"

#include "app.h"

namespace ruka3::g431 {

void BoardMotorHardware::force_safe() {
    if (auto* motor = get_motor()) {
        motor->set_foc_point(FOCTarget{0});
        (void)motor->set_bridge_outputs_enabled(false);
    }
}

bool BoardMotorHardware::calibration_valid() const {
    return is_motor_calibrated();
}

MotorTelemetry BoardMotorHardware::telemetry() const {
    auto* motor = get_motor();
    if (!motor) return {};
    return {
        .position = motor->get_angle(),
        .velocity = motor->get_velocity(),
        .torque = motor->get_torque(),
        .bus_voltage = motor->get_voltage(),
        .bus_current = motor->get_working_current(),
        .electric_angle = motor->get_electric_angle(),
        .rotor_encoder = motor->get_rotor_encoder_value(),
        .output_encoder = motor->get_shaft_encoder_value(),
    };
}

}  // namespace ruka3::g431
