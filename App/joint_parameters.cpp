#include "joint_parameters.hpp"
#include "service_motion.hpp"
#include "calibration_service.hpp"
#include "robot_config.h"
#include "stm32g4xx_hal.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr float unavailable_float = std::numeric_limits<float>::quiet_NaN();
constexpr std::int32_t missing = std::numeric_limits<std::int32_t>::min();
constexpr float ref_per_rad = ruka3::kCalibrationReferenceTicksPerTurn / 6.28318530718F;
struct SessionDiagnostics {
    std::uint32_t sequence{}, fault_at{}, fault_mask{}, previous_fault{}, enables{}, disables{};
    std::int32_t raw_previous{}, raw_unwrapped{}, raw_min{}, raw_max{}, max_delta{};
    bool initialized{}, armed{};
} session;

std::int32_t scalar(ParameterId id) {
    ParameterValue value;
    if (!read_board_parameter(id, value)) return missing;
    if (auto* v = std::get_if<std::uint32_t>(&value)) return static_cast<std::int32_t>(*v);
    if (auto* v = std::get_if<std::int32_t>(&value)) return *v;
    if (auto* v = std::get_if<bool>(&value)) return *v;
    return missing;
}
AngleDiagnostics low_angles() {
    ParameterValue value;
    if (read_board_parameter(ParameterId::GET_ANGLE, value)) return std::get<AngleDiagnostics>(value);
    AngleDiagnostics result; result.fill(unavailable_float); return result;
}
}

void update_register_diagnostics(std::uint32_t now) {
    static std::uint32_t last;
    if (now - last < 10) return;
    last = now;
    const auto fault = static_cast<std::uint32_t>(scalar(ParameterId::FAULT_ACTIVE));
    if (fault && fault != session.previous_fault) {
        ++session.sequence; session.fault_at = now; session.fault_mask = fault;
    }
    session.previous_fault = fault;
    const bool armed = scalar(ParameterId::ARM) == 1;
    if (armed != session.armed) { if (armed) ++session.enables; else ++session.disables; session.armed = armed; }
    const auto a = low_angles();
    const auto raw = static_cast<std::int32_t>(a[0]);
    const auto cpr = ruka3::joint_profile(RUKA3_JOINT_INDEX).observer.output_encoder_cpr;
    if (!session.initialized) {
        session.raw_unwrapped = session.raw_min = session.raw_max = raw;
        session.initialized = true;
    } else {
        auto delta = raw - session.raw_previous;
        if (delta > static_cast<std::int32_t>(cpr / 2)) delta -= cpr;
        if (delta < -static_cast<std::int32_t>(cpr / 2)) delta += cpr;
        session.raw_unwrapped += delta;
        session.raw_min = std::min(session.raw_min, session.raw_unwrapped);
        session.raw_max = std::max(session.raw_max, session.raw_unwrapped);
        session.max_delta = std::max(session.max_delta, std::abs(delta));
    }
    session.raw_previous = raw;
}

// Keep register snapshots out of the already deep Cyphal codec stack frame.
[[gnu::noinline]] bool read_parameter(ParameterId id, ParameterValue& value) {
    const auto& profile = ruka3::joint_profile(RUKA3_JOINT_INDEX);
    const auto& cal = joint_calibration_instance();
    const auto& core = cal.core();
    const auto& data = core.data();
    IntegerDiagnostics integers; integers.fill(missing);
    AngleDiagnostics floats; floats.fill(unavailable_float);
    switch (id) {
    case ParameterId::POSITION_KP: value = joint_motion_instance().position_kp(); return true;
    case ParameterId::POSITION_KD: value = joint_motion_instance().position_kd(); return true;
    case ParameterId::POSITION_KI: value = joint_motion_instance().position_ki(); return true;
    case ParameterId::INTEGRAL_LIMIT: value = joint_motion_instance().integral_limit(); return true;
    case ParameterId::INTEGRAL_CURRENT: value = joint_motion_instance().integral_current(); return true;
    case ParameterId::PROFILE_VELOCITY: value = joint_motion_instance().profile_velocity(); return true;
    case ParameterId::PROFILE_ACCELERATION: value = joint_motion_instance().profile_acceleration(); return true;
    case ParameterId::ACTIVE_GAINS:
        floats[0] = joint_motion_instance().snapshot().kp;
        floats[1] = joint_motion_instance().snapshot().kd;
        value = floats; return true;
    case ParameterId::MAN_CAL: case ParameterId::MOVE_CAL: value = std::int32_t{0}; return true;
    case ParameterId::ZERO_SET: case ParameterId::LUFT_CAL: value = std::int32_t{0}; return true;
    case ParameterId::ENCODER_CPR: value = profile.observer.output_encoder_cpr; return true;
    case ParameterId::ARM: {
        ParameterValue driver;
        integers[0] = scalar(ParameterId::ARM);
        if (read_board_parameter(ParameterId::DRIVER_DIAG, driver))
            integers[0] = std::get<IntegerDiagnostics>(driver)[1];
        value = integers; return true;
    }
    case ParameterId::BOOTLOADER: case ParameterId::RESET:
        value = scalar(id); return true;
    case ParameterId::FAULT_ACK: value = scalar(ParameterId::FAULT_LEVEL); return true;
    case ParameterId::VERSION:
        integers[0] = 1; integers[1] = 2; integers[2] = 0; integers[3] = profile.index;
        value = integers; return true;
    case ParameterId::CAL_DATA: {
        const auto span = std::abs(data.manual_span_ticks) - 2 * data.safe_margin_ticks;
        integers[0] = core.has_stored_data(); integers[1] = data.manual_span_ticks;
        integers[2] = data.tmc_span_steps;
        integers[3] = span > 0 ? static_cast<std::int32_t>(std::abs(static_cast<std::int64_t>(data.tmc_span_steps)) * profile.observer.output_encoder_cpr / span) : 0;
        integers[4] = ruka3::kCalibrationReferenceTicksPerTurn;
        integers[5] = data.backlash_steps; integers[6] = data.point_count; integers[7] = data.safe_margin_ticks;
        value = integers; return true;
    }
    case ParameterId::CAL_DIAG:
        integers[0] = static_cast<int>(core.state()); integers[1] = static_cast<int>(core.error());
        integers[2] = core.failure_state();
        std::copy(core.validation_diagnostics().begin(), core.validation_diagnostics().end(), integers.begin() + 3);
        value = integers; return true;
    case ParameterId::EEPROM_DIAG: {
        encoder_calibration_storage_diagnostics d{}; encoder_calibration_storage_get_diagnostics(&d);
        integers[0] = d.connected; integers[1] = d.valid_slot_mask; integers[2] = d.active_slot;
        integers[3] = d.sequence; integers[4] = d.last_save_ok; integers[5] = d.save_count;
        value = integers; return true;
    }
    case ParameterId::FAULT_HISTORY:
        integers[0] = session.sequence != 0; integers[1] = session.sequence;
        integers[2] = session.fault_at; integers[3] = session.fault_mask;
        value = integers; return true;
    case ParameterId::BOOT_DIAG:
        integers[0] = joint_uptime_ms(); integers[1] = RCC->CSR; integers[2] = 1;
        integers[3] = session.enables; integers[4] = session.disables;
        value = integers; return true;
    case ParameterId::CYPHAL_DIAG:
        integers[0] = missing; integers[1] = missing;
        integers[2] = scalar(ParameterId::CMD_AGE_MS);
        value = integers; return true;
    case ParameterId::IS_FAULT:
        integers[0] = scalar(ParameterId::FAULT_ACTIVE); integers[1] = scalar(ParameterId::FAULT_LATCHED);
        integers[2] = scalar(ParameterId::FAULT_LEVEL); integers[3] = scalar(ParameterId::STOP_REASON);
        integers[4] = scalar(ParameterId::NET_STATE); integers[5] = scalar(ParameterId::JOINT_STATE);
        integers[8] = RCC->CSR; integers[15] = static_cast<int>(core.state());
        integers[16] = static_cast<int>(core.error()); integers[17] = session.sequence;
        integers[18] = session.fault_at; integers[19] = session.fault_mask;
        integers[25] = core.failure_state();
        value = integers; return true;
    default: break;
    }
    if (id == ParameterId::GET_ANGLE || id == ParameterId::LIMITS || id == ParameterId::CONTROL_DIAG ||
        id == ParameterId::ENCODER_DIAG || id == ParameterId::MOTOR_POS || id == ParameterId::POS_SET) {
        const auto a = low_angles(); // Common board snapshot, kept internal.
        if (id == ParameterId::POS_SET) return read_board_parameter(id, value);
        if (id == ParameterId::MOTOR_POS) { value = static_cast<std::int32_t>(std::lround(a[2] * ref_per_rad)); return true; }
        const float control_mode = core.blocks_normal_control() ? 3 :
            ((a[22] == 1 || a[22] == 5) ? 1 : (a[22] == 0 || a[22] == 4) ? 0 : 2);
        if (id == ParameterId::GET_ANGLE) {
            floats[0] = a[0]; floats[1] = a[2] * ref_per_rad;
            floats[2] = a[5]; floats[3] = a[2]; floats[4] = a[10]; floats[5] = a[8];
            floats[6] = a[9]; floats[7] = a[11]; floats[8] = a[33];
            floats[9] = control_mode;
            floats[10] = core.has_stored_data() && core.zero_valid();
            floats[11] = a[16]; floats[12] = a[25]; floats[13] = a[26]; floats[14] = a[17];
            floats[15] = profile.speed_limit > 0 ? -profile.speed_limit : unavailable_float;
            floats[16] = profile.speed_limit > 0 ? profile.speed_limit : unavailable_float;
            floats[17] = static_cast<int>(core.state()); floats[18] = core.progress_percent();
            floats[19] = a[30];
            floats[25] = session.raw_min; floats[26] = session.raw_max;
            floats[27] = session.raw_max - session.raw_min; floats[28] = session.max_delta;
        } else if (id == ParameterId::LIMITS) {
            floats[0] = cal.snapshot().limits_valid;
            floats[1] = a[16]; floats[2] = a[16]; floats[3] = a[25]; floats[4] = a[26];
            floats[5] = a[17]; floats[6] = a[17];
        } else if (id == ParameterId::CONTROL_DIAG) {
            floats[0] = control_mode;
            floats[1] = a[23]; floats[2] = a[24]; floats[3] = a[34];
            floats[4] = a[23] * ref_per_rad;
            floats[5] = std::isfinite(a[35]) ? std::fabs(a[35]-a[8]) < 0.001F : unavailable_float;
            floats[6] = scalar(ParameterId::CMD_AGE_MS); floats[7] = scalar(ParameterId::JOINT_STATE);
        } else {
            if (read_board_parameter(ParameterId::ENCODER_DIAG, value))
                floats = std::get<AngleDiagnostics>(value);
            std::int32_t corrected{};
            floats[0] = a[0];
            if (core.calibrated_position_ticks(static_cast<std::uint16_t>(a[0]), corrected)) floats[1] = corrected;
            floats[2] = a[5]; floats[3] = a[3]; floats[4] = a[6]; floats[5] = session.max_delta;
            floats[9] = a[33]; floats[10] = a[30];
        }
        value = floats; return true;
    }
    return read_board_parameter(id, value);
}

[[gnu::noinline]] ParameterWriteResult write_runtime_parameter(ParameterId id, const ParameterValue& value) {
    const auto now = joint_uptime_ms();
    switch (id) {
    case ParameterId::POSITION_KP:
        return joint_motion_instance().set_position_gains(std::get<float>(value), joint_motion_instance().position_kd())
            ? ParameterWriteResult::OK : ParameterWriteResult::INVALID;
    case ParameterId::POSITION_KD:
        return joint_motion_instance().set_position_gains(joint_motion_instance().position_kp(), std::get<float>(value))
            ? ParameterWriteResult::OK : ParameterWriteResult::INVALID;
    case ParameterId::PROFILE_VELOCITY:
        return joint_motion_instance().set_profile_velocity(std::get<float>(value))
            ? ParameterWriteResult::OK : ParameterWriteResult::INVALID;
    case ParameterId::PROFILE_ACCELERATION:
        return joint_motion_instance().set_profile_acceleration(std::get<float>(value))
            ? ParameterWriteResult::OK : ParameterWriteResult::INVALID;
    case ParameterId::POSITION_KI:
        return joint_motion_instance().set_integral(std::get<float>(value), joint_motion_instance().integral_limit())
            ? ParameterWriteResult::OK : ParameterWriteResult::INVALID;
    case ParameterId::INTEGRAL_LIMIT:
        return joint_motion_instance().set_integral(joint_motion_instance().position_ki(), std::get<float>(value))
            ? ParameterWriteResult::OK : ParameterWriteResult::INVALID;
    case ParameterId::ARM:
        if (!std::get<bool>(value)) calibration_cancel();
        return write_board_parameter(id, value);
    case ParameterId::BOOTLOADER: case ParameterId::RESET:
        if (std::get<bool>(value)) calibration_cancel();
        return write_board_parameter(id, value);
    case ParameterId::MAN_CAL:
        return calibration_manual_command(std::get<std::int32_t>(value), now) ? ParameterWriteResult::OK : ParameterWriteResult::INVALID;
    case ParameterId::MOVE_CAL:
        return calibration_manual_move(std::get<std::int32_t>(value), now) ? ParameterWriteResult::OK : ParameterWriteResult::INVALID;
    case ParameterId::MOTOR_POS: {
        const auto a = low_angles();
        const float target = a[8] + std::get<std::int32_t>(value) / ref_per_rad - a[2];
        return write_board_parameter(ParameterId::POS_SET, ParameterValue{target});
    }
    default: return write_board_parameter(id, value);
    }
}
