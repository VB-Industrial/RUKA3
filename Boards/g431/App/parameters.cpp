#include "app.h"
#include "parameters.hpp"
#include "robot_config.h"
#include "calibration_service.hpp"
#include "tim.h"
#include <limits>

#ifndef VBDRIVE_MODEL
#error "VBDRIVE_MODEL must be defined by CMake"
#endif
#ifndef VBDRIVE_FIRMWARE_REV
#error "VBDRIVE_FIRMWARE_REV must be defined by CMake"
#endif

static uint32_t invalid_commands_counter = 0;
void record_invalid_command() { invalid_commands_counter += 1; }

bool read_board_parameter(ParameterId id, ParameterValue& value) {
    value = {};
    const auto& can_faults = get_can_fault_snapshot();
    const auto& command = get_command_snapshot();
    const auto& faults = get_fault_snapshot();
    const auto& output = get_output_observer_snapshot();
    const auto& mechanical = get_mechanical_calibration_snapshot();
    const auto& service = get_service_motion_snapshot();
    switch (id) {
        case ParameterId::NODE_ID: value.emplace<uint32_t>(ruka3::joint_profile(RUKA3_JOINT_INDEX).node_id); return true;
        case ParameterId::BOOTLOADER: value.emplace<bool>(bootloader_reboot_pending); return true;
        case ParameterId::RESET: value.emplace<bool>(reset_pending); return true;
        case ParameterId::CMD_ERRORS: value.emplace<uint32_t>(invalid_commands_counter); return true;
        case ParameterId::MODEL: value.emplace<std::string_view>(VBDRIVE_MODEL); return true;
        case ParameterId::REVISION: value.emplace<std::string_view>(VBDRIVE_FIRMWARE_REV); return true;
        case ParameterId::IS_FAULT: value.emplace<bool>(faults.level == ruka3::FaultLevel::fault); return true;
        case ParameterId::FAULT_ACTIVE: value.emplace<uint32_t>(faults.active); return true;
        case ParameterId::FAULT_LATCHED: value.emplace<uint32_t>(faults.latched); return true;
        case ParameterId::FAULT_LEVEL: value.emplace<uint32_t>(static_cast<uint32_t>(faults.level)); return true;
        case ParameterId::STOP_REASON: value.emplace<uint32_t>(static_cast<uint32_t>(faults.stop_reason)); return true;
        case ParameterId::NET_STATE: value.emplace<uint32_t>(static_cast<uint32_t>(faults.network_state)); return true;
        case ParameterId::FAULT_ACK: value.emplace<bool>(false); return true;
        case ParameterId::JOINT_STATE: value.emplace<uint32_t>(get_joint_state()); return true;
        case ParameterId::CAN_ERR_TOTAL: value.emplace<uint32_t>(can_faults.session_error_count); return true;
        case ParameterId::CAN_RX_ERR: value.emplace<uint32_t>(can_faults.rx_error_count); return true;
        case ParameterId::CAN_TX_ERR: value.emplace<uint32_t>(can_faults.tx_error_count); return true;
        case ParameterId::CAN_ERR_LOG: value.emplace<uint32_t>(can_faults.error_logging_count); return true;
        case ParameterId::CAN_ARB_LEC: value.emplace<uint32_t>(can_faults.arbitration_last_error); return true;
        case ParameterId::CAN_DATA_LEC: value.emplace<uint32_t>(can_faults.data_last_error); return true;
        case ParameterId::CAN_WARNING: value.emplace<bool>(can_faults.warning); return true;
        case ParameterId::CAN_PASSIVE: value.emplace<bool>(can_faults.error_passive); return true;
        case ParameterId::CAN_BUS_OFF: value.emplace<bool>(can_faults.bus_off); return true;
        case ParameterId::CMD_COUNT: value.emplace<uint32_t>(command.accepted_count); return true;
        case ParameterId::CMD_AGE_MS: value.emplace<uint32_t>(command.age_ms); return true;
        case ParameterId::CMD_TIMEOUT: value.emplace<bool>(command.timed_out); return true;
        case ParameterId::ZERO_SET: value.emplace<int32_t>(0); return true;
        case ParameterId::SET_LIMIT: value.emplace<uint32_t>(mechanical.limit_stage); return true;
        case ParameterId::MOVE: value.emplace<float>(service.mode == ruka3::ServiceMotionMode::voltage ? service.target : 0.0F); return true;
        case ParameterId::POS_SET: value.emplace<float>((service.mode == ruka3::ServiceMotionMode::position || service.mode == ruka3::ServiceMotionMode::servo) ? service.goal : output.fused_angle); return true;
        case ParameterId::CURRENT: value = service.mode == ruka3::ServiceMotionMode::current ? service.target : 0.0F; return true;
        default: break;
    }
    auto* motor = get_motor();
    if (!motor) return false;
    const auto& inverter = static_cast<const VBInverter&>(motor->get_inverter());
    switch (id) {
        case ParameterId::ARM: value.emplace<bool>(is_motor_enabled()); return true;
        case ParameterId::BUS_VOLTAGE: value.emplace<float>(motor->get_voltage()); return true;
        case ParameterId::BUS_CURRENT: value.emplace<float>(motor->get_working_current()); return true;
        case ParameterId::TEMP_MCU: value.emplace<float>(inverter.get_mcu_temperature() + 273.15f); return true;
        case ParameterId::TEMP_STATOR: value.emplace<float>(inverter.get_stator_temperature() + 273.15f); return true;
        case ParameterId::GET_ANGLE: {
            const auto& profile = ruka3::joint_profile(RUKA3_JOINT_INDEX);
            value.emplace<AngleDiagnostics>(AngleDiagnostics{
                static_cast<float>(motor->get_shaft_encoder_value()),
                static_cast<float>(motor->get_rotor_encoder_value()),
                static_cast<float>(ruka3::joint_profile(RUKA3_JOINT_INDEX).position_direction) * motor->get_angle(),
                static_cast<float>(ruka3::joint_profile(RUKA3_JOINT_INDEX).position_direction) * motor->get_velocity(),
                motor->get_electric_angle(),
                output.encoder_angle, output.output_velocity, output.motor_residual,
                output.fused_angle, output.fused_velocity, output.fusion_offset,
                output.encoder_error, static_cast<float>(get_motor_calibration_offset()),
                static_cast<float>(mechanical.encoder_zero), profile.gear_ratio,
                static_cast<float>(profile.observer.fusion_encoder_corridor_ticks),
                mechanical.lower_limit, mechanical.upper_limit,
                static_cast<float>(mechanical.limit_a_raw), static_cast<float>(mechanical.limit_b_raw),
                static_cast<float>(mechanical.limit_stage), static_cast<float>(mechanical.limit_direction),
                static_cast<float>(service.mode), service.target, service.error,
                mechanical.soft_lower_limit, mechanical.soft_upper_limit,
                static_cast<float>(mechanical.safe_margin_ticks),
                mechanical.zero_valid ? 1.0F : 0.0F,
                mechanical.limits_valid ? 1.0F : 0.0F,
                output.encoder_prediction_error,
                output.reversal_peak,
                static_cast<float>(output.motor_direction),
                output.expected_backlash_rad, service.velocity, service.goal,
            });
            return true;
        }
        case ParameterId::CALIB_VALID: value.emplace<bool>(is_motor_calibrated()); return true;
        case ParameterId::CALIB_INVERTED: value.emplace<bool>(is_motor_encoder_inverted()); return true;
        case ParameterId::MOTOR_CALIB: value.emplace<bool>(is_motor_calibration_pending()); return true;
        case ParameterId::DRIVER_WAKE: value.emplace<bool>(is_driver_awake()); return true;
        case ParameterId::DRIVER_DIAG: {
            IntegerDiagnostics diag{};
            diag[0] = HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin);
            diag[1] = motor->is_on(); diag[2] = is_driver_awake(); diag[3] = is_motor_calibrated();
            diag[4] = std::numeric_limits<std::int32_t>::min();
            diag[5] = TIM1->CR1; diag[6] = TIM1->CCER; diag[7] = TIM1->BDTR;
            diag[8] = TIM1->CCR1; diag[9] = TIM1->CCR2; diag[10] = TIM1->CCR3;
            value = diag; return true;
        }
        case ParameterId::ENCODER_DIAG: {
            AngleDiagnostics diag; diag.fill(std::numeric_limits<float>::quiet_NaN());
            CRITICAL_SECTION({
                diag[6] = motor->get_rotor_encoder_value();
                diag[7] = motor->get_preobserver_rotor_angle();
                diag[8] = motor->get_electric_angle();
                diag[11] = motor->get_encoder().CPR;
            })
            value = diag; return true;
        }
        case ParameterId::CURRENT_DIAG: {
            AngleDiagnostics diag; diag.fill(std::numeric_limits<float>::quiet_NaN());
            diag[0] = inverter.get_A(); diag[1] = inverter.get_B(); diag[2] = inverter.get_C();
            diag[4] = motor->get_working_current();
            diag[5] = joint_calibration_instance().core().blocks_normal_control()
                ? ruka3::joint_profile(RUKA3_JOINT_INDEX).position_direction * calibration_current_target()
                : service.mode == ruka3::ServiceMotionMode::current ? service.target : 0;
            diag[8] = motor->get_voltage(); diag[9] = inverter.get_A() + inverter.get_B() + inverter.get_C();
            diag[17] = ruka3::joint_profile(RUKA3_JOINT_INDEX).motor.current_limit;
            diag[18] = service.mode == ruka3::ServiceMotionMode::current ? 5 :
                service.mode == ruka3::ServiceMotionMode::voltage ? 3 : 4;
            diag[19] = motor->is_on(); diag[20] = command.age_ms;
            diag[21] = diag[4] * ruka3::joint_profile(RUKA3_JOINT_INDEX).position_direction;
            diag[22] = ruka3::joint_profile(RUKA3_JOINT_INDEX).motor.control_period_s;
            diag[23] = ruka3::joint_profile(RUKA3_JOINT_INDEX).motor_command_sign;
            value = diag; return true;
        }
        default: return false;
    }
}

[[gnu::noinline]] ParameterWriteResult write_board_parameter(ParameterId id, const ParameterValue& value) {
    if (id == ParameterId::BOOTLOADER) {
        bootloader_reboot_pending |= std::get<bool>(value);
        return ParameterWriteResult::OK;
    }
    if (id == ParameterId::RESET) {
        if (!std::get<bool>(value)) return ParameterWriteResult::INVALID;
        (void)set_motor_enabled(false);
        if (!reset_pending) reset_requested_at = millis_32();
        reset_pending = true;
        return ParameterWriteResult::OK;
    }
    if (id == ParameterId::FAULT_ACK) {
        return std::get<bool>(value) && acknowledge_faults() ? ParameterWriteResult::OK : ParameterWriteResult::INVALID;
    }
    if (id == ParameterId::MOTOR_CALIB) {
        return std::get<bool>(value) && request_motor_calibration()
            ? ParameterWriteResult::OK : ParameterWriteResult::INVALID;
    }
    if (id == ParameterId::DRIVER_WAKE) {
        return set_driver_awake(std::get<bool>(value))
            ? ParameterWriteResult::OK : ParameterWriteResult::INVALID;
    }
    if (id == ParameterId::SET_LIMIT) {
        return service_set_limit(std::get<uint32_t>(value)) ? ParameterWriteResult::OK : ParameterWriteResult::INVALID;
    }
    if (id == ParameterId::MOVE) {
        return service_set_voltage(std::get<float>(value)) ? ParameterWriteResult::OK : ParameterWriteResult::INVALID;
    }
    if (id == ParameterId::CURRENT) {
        return service_set_current(std::get<float>(value)) ? ParameterWriteResult::OK : ParameterWriteResult::INVALID;
    }
    if (id == ParameterId::POS_SET) {
        return service_set_position(std::get<float>(value)) ? ParameterWriteResult::OK : ParameterWriteResult::INVALID;
    }
    if (id != ParameterId::ARM) return ParameterWriteResult::READ_ONLY;
    const bool requested_on = std::get<bool>(value);
    return set_motor_enabled(requested_on) ? ParameterWriteResult::OK : ParameterWriteResult::INVALID;
}
