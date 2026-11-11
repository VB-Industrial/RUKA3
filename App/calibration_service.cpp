#include "calibration_service.hpp"
#include "robot_config.h"
#include <algorithm>
#include <cmath>

namespace {
std::int32_t reference_zero{};
std::int32_t manual_velocity{};
std::uint32_t manual_command_at{};
bool outputs_enabled{};
float commanded_current{};
std::uint32_t current_updated_at{};
void reset_current_command() { commanded_current = 0; current_updated_at = 0; }
constexpr float units_per_rad = ruka3::kCalibrationReferenceTicksPerTurn / 6.28318530718F;
std::int32_t reference() { return std::lround(calibration_board_angle() * units_per_rad); }
bool manual_stage(EncoderCalibrationState state) {
    return state == EncoderCalibrationState::WaitLimitA || state == EncoderCalibrationState::WaitMiddle ||
        state == EncoderCalibrationState::WaitLimitB || state == EncoderCalibrationState::Ready;
}
}

float calibration_current_target() { return outputs_enabled ? commanded_current : 0.0F; }

void calibration_cancel() {
    auto& core = joint_calibration_instance().core();
    if (core.blocks_normal_control()) core.abort();
    if (outputs_enabled) calibration_board_disable();
    outputs_enabled = false;
    manual_velocity = 0;
    reset_current_command();
}

bool calibration_manual_command(std::int32_t command, std::uint32_t now) {
    auto& cal = joint_calibration_instance();
    auto& core = cal.core();
    if (command < 1 || command > 5 || !calibration_board_available()) return false;
    manual_velocity = 0;
    calibration_board_disable();
    outputs_enabled = false;
    reset_current_command();
    const bool ok = command == 1 ? core.begin_manual(true, calibration_board_raw()) :
        core.advance_manual(now, true, calibration_board_raw(), command);
    if (!ok) return false;
    if (command == 5) {
        reference_zero = reference();
        outputs_enabled = calibration_board_enable();
        if (!outputs_enabled) { core.fail(EncoderCalibrationError::Driver); return false; }
    }
    cal.refresh();
    return true;
}

bool calibration_manual_move(std::int32_t velocity, std::uint32_t now) {
    auto& core = joint_calibration_instance().core();
    if (!manual_stage(core.state()) || !calibration_board_available()) return false;
    if (velocity != 0 && !outputs_enabled) {
        reset_current_command();
        outputs_enabled = calibration_board_enable();
        if (!outputs_enabled) return false;
    }
    manual_velocity = velocity;
    manual_command_at = now;
    if (velocity == 0) { calibration_board_disable(); outputs_enabled = false; }
    return true;
}

bool calibration_begin_backlash(std::uint32_t now) {
    auto& core = joint_calibration_instance().core();
    if (!calibration_board_available()) return false;
    calibration_board_disable();
    outputs_enabled = false;
    manual_velocity = 0;
    reset_current_command();
    reference_zero = reference();
    if (!core.begin_backlash(now, true, calibration_board_raw())) return false;
    outputs_enabled = calibration_board_enable();
    if (!outputs_enabled) core.fail(EncoderCalibrationError::Driver);
    return outputs_enabled;
}

void calibration_service_update(std::uint32_t now) {
    auto& cal = joint_calibration_instance();
    auto& core = cal.core();
    if (!core.blocks_normal_control()) return;
    const auto previous = core.state();
    if (outputs_enabled && !calibration_board_available()) core.fail(EncoderCalibrationError::Driver);
    const auto motor_reference = reference();
    auto action = core.update(now, true, calibration_board_raw(), motor_reference - reference_zero);
    if (action.zero_tmc_position) reference_zero = motor_reference; // Same snapshot as the captured sample.
    if (manual_stage(core.state()) && outputs_enabled) {
        if (now - manual_command_at >= ruka3::joint_profile(RUKA3_JOINT_INDEX).safety.command_timeout_ms)
            manual_velocity = 0;
        action.command_velocity = true;
        action.velocity_steps = manual_velocity;
    }
    if (action.disable_driver) { calibration_board_disable(); outputs_enabled = false; }
    else if (outputs_enabled && action.command_velocity) {
        const float target_velocity = std::clamp(action.velocity_steps / units_per_rad,
            -ruka3::kCalibrationMaxVelocityRadS, ruka3::kCalibrationMaxVelocityRadS);
        // A stopped TMC velocity target brakes the rotor. Keep velocity feedback
        // active at zero too; zero torque would coast through settle/rock points.
        constexpr auto& tuning = ruka3::joint_profile(RUKA3_JOINT_INDEX).command;
        const float velocity = calibration_board_velocity();
        const float current = std::clamp(
            tuning.calibration_velocity_gain_a * (target_velocity - velocity),
            -tuning.calibration_current_limit_a, tuning.calibration_current_limit_a);
        const float dt = std::min<std::uint32_t>(now - current_updated_at, 20U) * 0.001F;
        const float step = ruka3::kCalibrationCurrentSlewAS * dt;
        // Slew only the buildup of motoring torque. Cutting torque and braking
        // must react immediately when a stuck rotor breaks free.
        if (current * velocity < 0.0F ||
            (current * commanded_current >= 0.0F && std::fabs(current) <= std::fabs(commanded_current))) {
            commanded_current = current;
        } else {
            if (current * commanded_current < 0.0F) commanded_current = 0.0F;
            commanded_current += std::clamp(current - commanded_current, -step, step);
        }
        current_updated_at = now;
        calibration_board_current(commanded_current);
    }
    cal.refresh();
    if (previous != core.state() && core.state() == EncoderCalibrationState::Complete) calibration_board_sync();
}
