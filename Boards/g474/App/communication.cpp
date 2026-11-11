#include "app.h"
#include "fdcan.h"
#include "command_supervisor.hpp"
#include "fault_manager.hpp"
#include "joint_controller.hpp"
#include "protocol.h"
#include "calibration_service.hpp"
#include "joint_register_codec.hpp"
#include "service_motion.hpp"
#include "output_state_observer.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <cyphal/allocators/o1/o1_allocator.h>
#include <cyphal/node/node_info_handler.h>
#include <cyphal/node/registers_handler.hpp>
#include <cyphal/providers/G4CAN.h>
#include <uavcan/node/Heartbeat_1_0.h>
#include <uavcan/node/Health_1_0.h>
#include <uavcan/node/Mode_1_0.h>
#include <voltbro/foc/command_1_0.h>
#include <voltbro/foc/state_simple_1_0.h>

TYPE_ALIAS(HBeat, uavcan_node_Heartbeat_1_0)
TYPE_ALIAS(JS_msg, voltbro_foc_command_1_0)
TYPE_ALIAS(FocState, voltbro_foc_state_simple_1_0)

std::uint32_t joint_uptime_ms() { return HAL_GetTick(); }

namespace {
constexpr auto& profile = active_joint_profile;
constexpr float direction = static_cast<float>(profile.position_direction);
constexpr unsigned register_count = PARAMETER_CATALOG.size();
UtilityConfig utilities(micros_64, Error_Handler);
std::shared_ptr<CyphalInterface> interface;
ReservedObject<NodeInfoReader> node_info;
ReservedObject<RegistersHandler<register_count>> registers;
ruka3::JointController joint;
ruka3::CommandSupervisor commands;
ruka3::CanFaultMonitor can_monitor{hfdcan1};
ruka3::FaultManager faults;
ruka3::JointCalibration mechanical_calibration(profile.index, profile.observer.output_encoder_cpr,
    profile.output_encoder_sign * profile.position_direction, ruka3::kCalibrationReferenceTicksPerTurn,
    ruka3::calibration_margin_ticks(profile.observer.output_encoder_cpr));
std::uint32_t command_errors = 0;
ruka3::ServiceMotion service_controller{profile.command, std::min(profile.motor.stall_current, profile.motor.nominal_current)};
const auto& service_motion = service_controller.snapshot();
ruka3::OutputStateObserver output_observer{{
    profile.observer.output_encoder_cpr, profile.output_encoder_zero,
    static_cast<std::int8_t>(profile.output_encoder_sign * profile.position_direction),
    profile.observer.output_velocity_lpf, profile.observer.fusion_encoder_corridor_ticks,
    profile.observer.expected_backlash_rad, profile.observer.fusion_encoder_weight}};
void update_service_motion();
void sample_faults();

void hold_current_position() {
    service_controller.hold(output_observer.snapshot().fused_angle, HAL_GetTick());
    update_service_motion();
}

void update_service_motion() {
    auto& motor = get_motor();
    if (!joint.accepts_motor_commands() || !motor.is_on()) return;
    const auto now = HAL_GetTick();
    const auto& output = output_observer.snapshot();
    const auto& limits = mechanical_calibration.snapshot();
    const bool was_running = service_controller.autonomous_motion_active();
    if (!service_controller.update(now, output.fused_angle, limits)) return;
    if (was_running && !service_controller.autonomous_motion_active()) commands.complete_autonomous_command(now);
    if (service_motion.mode == ruka3::ServiceMotionMode::voltage) {
        motor.set_voltage_point(direction * profile.motor_command_sign * service_motion.target *
            ruka3::outward_command_scale(service_motion.target, output.fused_angle, limits));
    } else if (service_motion.mode == ruka3::ServiceMotionMode::current) {
        motor.set_current_point(direction * service_motion.target *
            ruka3::outward_command_scale(service_motion.target, output.fused_angle, limits));
    } else {
        const auto local = ruka3::wrapped_position_target(service_motion);
        const float amp_to_torque = profile.motor.torque_const * profile.gear_ratio;
        const float base_current = ruka3::position_current_request(service_motion, output.fused_velocity,
            profile.motor.torque_const, profile.gear_ratio);
        const float current_limit = std::min(profile.motor.nominal_current, profile.motor.torque_limit/amp_to_torque);
        const float load_current = service_controller.compensate_load(now, output.fused_angle, base_current, current_limit, limits);
        motor.set_foc_point({.torque = direction * (local.torque + load_current*amp_to_torque),
            .angle = motor.get_fusion_angle() + direction * local.angle_error,
            .velocity = motor.get_fusion_velocity() + direction * (service_motion.velocity-output.fused_velocity),
            // Same motor-side gain contract as VBDrive, without a second gear division.
            .angle_kp = service_motion.kp / profile.gear_ratio,
            .velocity_kp = service_motion.kd / profile.gear_ratio});
    }
}

void disarm() {
    HAL_IMPORTANT(get_motor().stop())
    joint.disarm();
    service_controller.cancel();
}

bool arm() {
    auto& motor = get_motor();
    if (joint.accepts_motor_commands() && motor.is_on()) return true;
    if (mechanical_calibration.core().blocks_normal_control() || !mechanical_calibration.snapshot().limits_valid ||
        faults.snapshot().latched || !std::isfinite(motor.get_voltage()) || motor.get_voltage() <= 0.0F ||
        !joint.arm()) return false;
    if (motor.start() != HAL_OK) { joint.disarm(); return false; }
    hold_current_position();
    commands.note_accepted_command(HAL_GetTick());
    return true;
}

class JSSub final : public AbstractSubscription<JS_msg> {
public:
    JSSub(InterfacePtr ptr, CanardPortID port): AbstractSubscription<JS_msg>(ptr, port) {}
    void handler(const JS_msg::Type& input, CanardRxTransfer*) override {
        if (!joint.accepts_motor_commands() || !get_motor().is_on() ||
            !service_controller.servo(input.angle.radian, input.velocity.radian_per_second,
                input._torque.newton_meter, input.angle_kp.value, input.velocity_kp.value,
                output_observer.snapshot().fused_angle, mechanical_calibration.snapshot(), HAL_GetTick())) {
            ++command_errors;
            return;
        }
        commands.note_accepted_command(HAL_GetTick());
        update_service_motion();
    }
};
ReservedObject<JSSub> servo;

void sample_faults() {
    can_monitor.poll();
    const float unavailable = std::numeric_limits<float>::quiet_NaN();
    const bool driver_fault = get_motor().is_on() && HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin) == GPIO_PIN_RESET;
    if (faults.update({.can = can_monitor.snapshot(), .calibration_valid = true,
            .driver_fault = driver_fault, .mcu_temperature_c = unavailable, .stator_temperature_c = unavailable,
            .current_loop_fault = get_motor().get_current_fault() != CurrentLoopFault::none})) {
        disarm();
        joint.fault();
    }
}
} // namespace

std::shared_ptr<CyphalInterface>& get_interface() { return interface; }
const EncoderCalibration& get_output_calibration_core() { return mechanical_calibration.core(); }
ruka3::JointCalibration& joint_calibration_instance() { return mechanical_calibration; }
ruka3::ServiceMotion& joint_motion_instance() { return service_controller; }
std::uint16_t calibration_board_raw() {
    const auto& encoder = get_gear_encoder();
    return profile.output_encoder_sign < 0 ? encoder.CPR - encoder.get_value() : encoder.get_value();
}
float calibration_board_angle() { return direction * get_motor().get_angle(); }
float calibration_board_velocity() { return direction * get_motor().get_velocity(); }
bool calibration_board_available() { return faults.snapshot().latched == 0 && get_motor().get_voltage() > 0; }
bool calibration_board_enable() { return get_motor().start() == HAL_OK; }
void calibration_board_disable() { disarm(); }
void calibration_board_current(float current) { get_motor().set_current_point(direction * current); }
void calibration_board_sync() {
    mechanical_calibration.refresh(); get_motor().reset_output_tracking();
    output_observer.set_encoder_zero(mechanical_calibration.snapshot().encoder_zero);
    const auto& limits = mechanical_calibration.snapshot();
    if (limits.limits_valid) output_observer.set_calibrated_limits(limits.lower_limit, limits.upper_limit);
}

void record_invalid_command() { ++command_errors; }

bool read_board_parameter(ParameterId id, ParameterValue& value) {
    auto& motor = get_motor();
    const auto& limits = mechanical_calibration.snapshot();
    const auto& fault = faults.snapshot();
    const auto& can = can_monitor.snapshot();
    const auto& command = commands.snapshot();
    switch (id) {
    case ParameterId::NODE_ID: value = std::uint32_t{profile.node_id}; return true;
    case ParameterId::ARM: value = joint.accepts_motor_commands() && motor.is_on(); return true;
    case ParameterId::BOOTLOADER: value = bootloader_reboot_pending; return true;
    case ParameterId::RESET: value = reset_pending; return true;
    case ParameterId::MODEL: value = std::string_view{"ruka3.g474"}; return true;
    case ParameterId::REVISION: value = std::string_view{"shared-motion"}; return true;
    case ParameterId::BUS_VOLTAGE: value = motor.get_voltage(); return true;
    case ParameterId::BUS_CURRENT: value = motor.get_working_current(); return true;
    case ParameterId::TEMP_MCU: case ParameterId::TEMP_STATOR:
        value = std::numeric_limits<float>::quiet_NaN(); return true;
    case ParameterId::IS_FAULT: value = fault.latched != 0; return true;
    case ParameterId::CALIB_VALID: value = true; return true; // Commissioned compile-time electric offset.
    case ParameterId::CALIB_INVERTED: value = profile.motor_encoder_sign < 0; return true;
    case ParameterId::MOTOR_CALIB: value = false; return true;
    case ParameterId::DRIVER_WAKE: value = HAL_GPIO_ReadPin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin) == GPIO_PIN_SET; return true;
    case ParameterId::CMD_ERRORS: value = command_errors; return true;
    case ParameterId::CMD_COUNT: value = command.accepted_count; return true;
    case ParameterId::CMD_AGE_MS: value = command.age_ms; return true;
    case ParameterId::CMD_TIMEOUT: value = command.timed_out; return true;
    case ParameterId::FAULT_ACTIVE: value = fault.active; return true;
    case ParameterId::FAULT_LATCHED: value = fault.latched; return true;
    case ParameterId::FAULT_LEVEL: value = static_cast<std::uint32_t>(fault.level); return true;
    case ParameterId::STOP_REASON: value = static_cast<std::uint32_t>(fault.stop_reason); return true;
    case ParameterId::NET_STATE: value = static_cast<std::uint32_t>(fault.network_state); return true;
    case ParameterId::JOINT_STATE: value = static_cast<std::uint32_t>(joint.state()); return true;
    case ParameterId::CAN_ERR_TOTAL: value = can.session_error_count; return true;
    case ParameterId::CAN_RX_ERR: value = can.rx_error_count; return true;
    case ParameterId::CAN_TX_ERR: value = can.tx_error_count; return true;
    case ParameterId::CAN_ERR_LOG: value = can.error_logging_count; return true;
    case ParameterId::CAN_ARB_LEC: value = can.arbitration_last_error; return true;
    case ParameterId::CAN_DATA_LEC: value = can.data_last_error; return true;
    case ParameterId::CAN_WARNING: value = can.warning; return true;
    case ParameterId::CAN_PASSIVE: value = can.error_passive; return true;
    case ParameterId::CAN_BUS_OFF: value = can.bus_off; return true;
    case ParameterId::CURRENT: value = service_motion.mode == ruka3::ServiceMotionMode::current ? service_motion.target : 0.0F; return true;
    case ParameterId::MOVE: value = service_motion.mode == ruka3::ServiceMotionMode::voltage ? service_motion.target : 0.0F; return true;
    case ParameterId::POS_SET: value = (service_motion.mode == ruka3::ServiceMotionMode::position || service_motion.mode == ruka3::ServiceMotionMode::servo) ? service_motion.goal : output_observer.snapshot().fused_angle; return true;
    case ParameterId::GET_ANGLE: {
        AngleDiagnostics a; a.fill(std::numeric_limits<float>::quiet_NaN());
        CRITICAL_SECTION({
            a[0] = calibration_board_raw(); a[1] = motor.get_encoder().get_value();
            a[2] = direction * motor.get_angle(); a[3] = direction * motor.get_velocity();
            const auto& output = output_observer.snapshot();
            a[4] = motor.get_electric_angle(); a[5] = output.encoder_angle;
            a[6] = output.output_velocity; a[7] = output.motor_residual;
            a[8] = output.fused_angle; a[9] = output.fused_velocity;
            a[10] = output.fusion_offset; a[11] = output.encoder_error;
            a[30] = output.encoder_prediction_error; a[31] = output.reversal_peak;
            a[32] = output.motor_direction;

        })
        a[12] = motor.get_encoder_electric_offset(); a[13] = limits.encoder_zero;
        a[14] = profile.gear_ratio; a[15] = profile.observer.fusion_encoder_corridor_ticks;
        a[16] = limits.lower_limit; a[17] = limits.upper_limit;
        a[18] = limits.limit_a_raw; a[19] = limits.limit_b_raw;
        a[20] = limits.limit_stage; a[21] = limits.limit_direction;
        a[22] = static_cast<int>(service_motion.mode); a[23] = service_motion.target; a[24] = service_motion.error;
        a[25] = limits.soft_lower_limit; a[26] = limits.soft_upper_limit; a[27] = limits.safe_margin_ticks;
        a[28] = limits.zero_valid; a[29] = limits.limits_valid;
        a[34] = service_motion.velocity; a[35] = service_motion.goal;
        a[33] = output_observer.snapshot().expected_backlash_rad;
        value = a; return true;
    }
    case ParameterId::DRIVER_DIAG: {
        IntegerDiagnostics a{};
        a[0] = HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin);
        a[1] = motor.is_on(); a[2] = HAL_GPIO_ReadPin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin);
        a[3] = 1; a[4] = static_cast<int>(motor.get_current_fault());
        a[5] = TIM1->CR1; a[6] = TIM1->CCER; a[7] = TIM1->BDTR;
        a[8] = TIM1->CCR1; a[9] = TIM1->CCR2; a[10] = TIM1->CCR3;
        value = a; return true;
    }
    case ParameterId::ENCODER_DIAG: {
        AngleDiagnostics a; a.fill(std::numeric_limits<float>::quiet_NaN());
        CRITICAL_SECTION({
            a[6] = motor.get_encoder().get_value();
            a[7] = motor.get_preobserver_rotor_angle();
            a[8] = motor.get_electric_angle();
            a[11] = motor.get_encoder().CPR;
        })
        value = a; return true;
    }
    case ParameterId::CURRENT_DIAG: {
        const auto& inverter = static_cast<const Inverter&>(motor.get_inverter());
        CurrentLoopSnapshot loop;
        CRITICAL_SECTION({ loop = motor.current_snapshot(); })
        const auto raw = inverter.raw_adc();
        const auto offsets = inverter.offset_volts();
        AngleDiagnostics a; a.fill(std::numeric_limits<float>::quiet_NaN());
        a[0] = inverter.get_A(); a[1] = inverter.get_B(); a[2] = inverter.get_C();
        a[3] = loop.id; a[4] = loop.iq; a[5] = loop.iq_target; a[6] = loop.vd; a[7] = loop.vq;
        a[8] = motor.get_voltage(); a[9] = a[0] + a[1] + a[2];
        for (unsigned i = 0; i < 4; ++i) a[10+i] = raw[i];
        for (unsigned i = 0; i < 3; ++i) a[14+i] = offsets[i];
        a[17] = motor.get_current_limit(); a[18] = static_cast<int>(motor.get_point_type());
        a[19] = motor.is_on(); a[20] = command.age_ms;
        a[21] = direction * motor.get_working_current(); a[22] = profile.motor.control_period_s;
        a[23] = profile.motor_command_sign; a[24] = static_cast<int>(motor.get_current_fault());
        value = a; return true;
    }
    default: return false;
    }
}

ParameterWriteResult write_board_parameter(ParameterId id, const ParameterValue& value) {
    auto& motor = get_motor();
    if (id == ParameterId::ARM) return (std::get<bool>(value) ? arm() : (disarm(), true)) ? ParameterWriteResult::OK : ParameterWriteResult::INVALID;
    if (id == ParameterId::RESET || id == ParameterId::BOOTLOADER) {
        if (!std::get<bool>(value)) return ParameterWriteResult::INVALID;
        disarm(); reset_requested_at = HAL_GetTick();
        if (id == ParameterId::RESET) reset_pending = true; else bootloader_reboot_pending = true;
        return ParameterWriteResult::OK;
    }
    if (id == ParameterId::FAULT_ACK) {
        if (!std::get<bool>(value) || !motor.clear_current_fault()) return ParameterWriteResult::INVALID;
        sample_faults();
        return faults.acknowledge() && joint.acknowledge_fault(true) ? ParameterWriteResult::OK : ParameterWriteResult::INVALID;
    }
    if (id == ParameterId::DRIVER_WAKE) {
        if (motor.is_on()) return ParameterWriteResult::INVALID;
        HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, std::get<bool>(value) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        return ParameterWriteResult::OK;
    }
    if (id == ParameterId::MOTOR_CALIB) return ParameterWriteResult::UNAVAILABLE;
    if (!joint.accepts_motor_commands() || !motor.is_on()) return ParameterWriteResult::INVALID;
    if (id == ParameterId::CURRENT || id == ParameterId::MOVE || id == ParameterId::POS_SET) {
        const float target = std::get<float>(value);
        const auto now = HAL_GetTick();
        const auto& limits = mechanical_calibration.snapshot();
        bool accepted;
        if (id == ParameterId::POS_SET) {
            accepted = service_controller.position(target, output_observer.snapshot().fused_angle, limits, now);
        } else {
            const auto mode = id == ParameterId::CURRENT ? ruka3::ServiceMotionMode::current : ruka3::ServiceMotionMode::voltage;
            const float bound = id == ParameterId::CURRENT ? motor.get_current_limit() : ruka3::kG474CommissioningVoltageLimitV;
            accepted = service_controller.direct(mode, target, bound, limits, now);
        }
        if (!accepted) return ParameterWriteResult::INVALID;
        commands.note_accepted_command(now);
        update_service_motion();
        return ParameterWriteResult::OK;
    }
    return ParameterWriteResult::READ_ONLY;
}

void heartbeat() {
    static CanardTransferID transfer_id = 0;
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    HBeat::Type message{};
    message.uptime = HAL_GetTick() / 1000U;
    message.health.value = faults.snapshot().latched || faults.snapshot().level == ruka3::FaultLevel::fault ?
        uavcan_node_Health_1_0_CAUTION : uavcan_node_Health_1_0_NOMINAL;
    message.mode.value = uavcan_node_Mode_1_0_OPERATIONAL;
    interface->send_msg<HBeat>(&message, ruka3::protocol::kHeartbeatSubject, &transfer_id);
}

void setup_cyphal_comms() {
    interface = std::shared_ptr<CyphalInterface>(CyphalInterface::create_heap<G4CAN, O1Allocator>(
        profile.node_id, &hfdcan1, 200, utilities));
    node_info.create(interface, "ruka3.g474.current", uavcan_node_Version_1_0{1, 0},
        uavcan_node_Version_1_0{1, 0}, uavcan_node_Version_1_0{1, 1}, 0);
    std::array<RegisterDefinition, register_count> definitions;
    for (unsigned i = 0; i < definitions.size(); ++i) {
        definitions[i] = {std::string(PARAMETER_NAMES[i]), [i](const uavcan_register_Value_1_0& input,
            uavcan_register_Value_1_0& output, RegisterAccessResponse::Type& response) {
                handle_parameter_register(i, input, output, response);
            }};
    }
    registers.create(std::move(definitions), interface);
    servo.create(interface, profile.servo_subject_id);
}

void setup_can() {
    HAL_IMPORTANT(HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1,
        hfdcan1.Init.DataTimeSeg1 * hfdcan1.Init.DataPrescaler, 0))
    HAL_IMPORTANT(HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1))
    HAL_IMPORTANT(HAL_FDCAN_Start(&hfdcan1))
}

void setup_motor_control() {
    mechanical_calibration.load(profile.output_encoder_zero);
    get_motor().reset_output_tracking();
    output_observer.set_calibration(&mechanical_calibration.core());
    output_observer.set_encoder_zero(mechanical_calibration.snapshot().encoder_zero);
    const auto& limits = mechanical_calibration.snapshot();
    if (limits.limits_valid) output_observer.set_calibrated_limits(limits.lower_limit, limits.upper_limit);
    output_observer.update(HAL_GetTick(), calibration_board_raw(), direction * get_motor().get_angle());
    joint.configuration_loaded(true); // Electrical offsets are compile-time robot_config data.
    commands.initialize(HAL_GetTick(), profile.safety.command_timeout_ms);
    faults.initialize();
}

void comms_loop(millis now) {
    interface->loop();
    // Register/subscription callbacks timestamp commands during loop(). Read
    // time afterwards so unsigned age cannot underflow for a fresh command.
    now = HAL_GetTick();
    sample_faults();
    calibration_service_update(now);
    output_observer.update(now, calibration_board_raw(), direction * get_motor().get_angle());
    update_service_motion();
    update_register_diagnostics(now);
    const float unavailable = std::numeric_limits<float>::quiet_NaN();
    if (joint.accepts_motor_commands() && !get_motor().is_on()) joint.disarm();
    // Completing a profile may have refreshed the deadline inside update_service_motion().
    // Sample the clock after that write, including when it crossed a millisecond boundary.
    if (commands.update(HAL_GetTick(), service_controller.autonomous_motion_active()) && joint.accepts_motor_commands()) hold_current_position();

    if ((bootloader_reboot_pending || reset_pending) && now - reset_requested_at >= 150U) {
        if (bootloader_reboot_pending) {
            (void)HAL_FDCAN_Stop(&hfdcan1);
            __HAL_RCC_PWR_CLK_ENABLE();
            HAL_PWR_EnableBkUpAccess();
            TAMP->BKP0R = ruka3::protocol::kBootRequestMagic;
            TAMP->BKP1R = ruka3::protocol::boot_transport_1m4(profile.node_id);
            TAMP->BKP2R = 0;
            HAL_PWR_DisableBkUpAccess();
        }
        NVIC_SystemReset();
    }

    static millis last_heartbeat = 0, last_feedback = 0;
    if (now - last_heartbeat >= 1000U) { last_heartbeat = now; heartbeat(); }
    if (now - last_feedback >= 1000U / profile.communication.feedback_update_hz) {
        last_feedback = now;
        static CanardTransferID transfer_id = 0;
        auto& motor = get_motor();
        FocState::Type message{};
        message.timestamp.microsecond = micros_64();
        message.angle.radian = output_observer.snapshot().fused_angle;
        message.velocity.radian_per_second = output_observer.snapshot().fused_velocity;
        message._torque.newton_meter = direction * motor.get_torque();
        message.current.ampere = direction * motor.get_working_current();
        message.bus_voltage.volt = motor.get_voltage();
        message.mcu_temp.kelvin = unavailable;
        message.stator_temp.kelvin = unavailable;
        message.has_fault.value = faults.snapshot().latched != 0;
        interface->send_msg<FocState>(&message, profile.feedback_subject_id, &transfer_id);
    }
}
