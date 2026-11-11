//#pragma region Includes
#include "app.h"

#include <memory>
#include <cmath>
#include <type_traits>

#include "tim.h"
#include "i2c.h"
#include "adc.h"

#include "spi.h"
#include "cordic.h"

#include <voltbro/devices/stspin32g4.hpp>
#include <cyphal/node/node_info_handler.h>
#include <cyphal/node/registers_handler.hpp>
#include <cyphal/node/registers_utils.hpp>
#include <cyphal/providers/G4CAN.h>

#include <uavcan/node/Mode_1_0.hpp>
#include <uavcan/node/Health_1_0.hpp>
#include <uavcan/primitive/Empty_1_0.hpp>
#include <uavcan/primitive/array/Real32_1_0.hpp>
#include <uavcan/primitive/scalar/Real32_1_0.hpp>
#include <uavcan/si/unit/angular_velocity/Scalar_1_0.hpp>
#include <uavcan/si/unit/angle/Scalar_1_0.hpp>
#include <uavcan/si/unit/torque/Scalar_1_0.hpp>
#include <uavcan/si/unit/voltage/Scalar_1_0.hpp>
#include <voltbro/foc/command_1_0.hpp>
#include <voltbro/foc/state_simple_1_0.hpp>

#include <voltbro/eeprom/eeprom.hpp>
#include <voltbro/encoders/ASxxxx/AS5047P.hpp>
#include <voltbro/motors/bldc/vbdrive/vbdrive.hpp>
#include <voltbro/utils.hpp>
#include "robot_config.h"
#include "joint_controller.hpp"
#include "output_state_observer.hpp"
#include "parameters.hpp"
#include "calibration_service.hpp"
#include "joint_register_codec.hpp"
#include "protocol.h"
std::uint32_t joint_uptime_ms() { return millis_32(); }
//#pragma endregion

//#pragma region ExternConfiguration
#define NANOPRINTF_IMPLEMENTATION
#define NANOPRINTF_USE_FIELD_WIDTH_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_PRECISION_FORMAT_SPECIFIERS   1
#define NANOPRINTF_USE_FLOAT_FORMAT_SPECIFIERS       1 // float
#define NANOPRINTF_USE_LARGE_FORMAT_SPECIFIERS       1 // 'l' (long), 'll' (long long)
#define NANOPRINTF_USE_SMALL_FORMAT_SPECIFIERS       1 // 'hh' (char), 'h' (short)
#define NANOPRINTF_USE_BINARY_FORMAT_SPECIFIERS      0 // %b (binary)
#define NANOPRINTF_USE_WRITEBACK_FORMAT_SPECIFIERS   0 // %n
#include "nanoprintf.h"
extern "C" {
    // During setup we need tiny heap for shared_ptr of CyphalInterface for API compatibility reasons
    bool global_allocation_lock = false;
}
//#pragma endregion

void setup_cordic() {
    CORDIC_ConfigTypeDef cordic_config {
        .Function = CORDIC_FUNCTION_COSINE,
        .Scale = CORDIC_SCALE_0,
        .InSize = CORDIC_INSIZE_32BITS,
        .OutSize = CORDIC_OUTSIZE_32BITS,
        .NbWrite = CORDIC_NBWRITE_1,
        .NbRead = CORDIC_NBREAD_2,
        .Precision = CORDIC_PRECISION_6CYCLES
    };
    HAL_IMPORTANT(HAL_CORDIC_Configure(&hcordic, &cordic_config));
}

EEPROM eeprom(&hi2c2, 64, I2C_MEMADD_SIZE_16BIT);
EEPROM& get_eeprom() {
    return eeprom;
}
static STSPIN32G4 motor_gate_driver(&hi2c3, GpioPin(DRV_WAKE_GPIO_Port, DRV_WAKE_Pin));

// correct elec_offset will be set by apply_calibration
AS5047P motor_encoder(GpioPin(SPI1_CS0_GPIO_Port, SPI1_CS0_Pin), &hspi1);
InductiveSensor inductive_sensor(
    eeprom,
    IND_SENSOR_STATE_PLACEMENT,
    &hspi3,
    GpioPin(SPI3_CS_GPIO_Port, SPI3_CS_Pin)
);
VBInverter motor_inverter(&hadc1, &hadc2);
alignas(4) static CalibrationData calibration_data;  // avoid stack overflow and misalignment issues for I2C EEPROM
static std::aligned_storage_t<sizeof(VBDrive), alignof(VBDrive)> motor_storage;
static VBDrive* motor = nullptr;
static bool motor_calibration_valid = false;
static bool motor_calibration_pending = false;
static millis motor_calibration_requested_at = 0;
static bool driver_awake = false;
static constexpr float kPositionDirection = static_cast<float>(
    ruka3::joint_profile(RUKA3_JOINT_INDEX).position_direction);
static ruka3::JointController joint_controller;
static ruka3::CanFaultMonitor can_fault_monitor{hfdcan1};
static ruka3::CommandSupervisor command_supervisor;
static ruka3::FaultManager fault_manager;
static ruka3::OutputStateObserver output_observer({
    ruka3::joint_profile(RUKA3_JOINT_INDEX).observer.output_encoder_cpr,
    ruka3::joint_profile(RUKA3_JOINT_INDEX).output_encoder_zero,
    static_cast<std::int8_t>(ruka3::joint_profile(RUKA3_JOINT_INDEX).output_encoder_sign *
                             ruka3::joint_profile(RUKA3_JOINT_INDEX).position_direction),
    ruka3::joint_profile(RUKA3_JOINT_INDEX).observer.output_velocity_lpf,
    ruka3::joint_profile(RUKA3_JOINT_INDEX).observer.fusion_encoder_corridor_ticks,
    ruka3::joint_profile(RUKA3_JOINT_INDEX).observer.expected_backlash_rad,
    ruka3::joint_profile(RUKA3_JOINT_INDEX).observer.fusion_encoder_weight,
});
static ruka3::MechanicalCalibration mechanical_calibration(
    RUKA3_JOINT_INDEX,
    ruka3::joint_profile(RUKA3_JOINT_INDEX).observer.output_encoder_cpr,
    static_cast<std::int8_t>(ruka3::joint_profile(RUKA3_JOINT_INDEX).output_encoder_sign *
                             ruka3::joint_profile(RUKA3_JOINT_INDEX).position_direction),
    ruka3::kCalibrationReferenceTicksPerTurn,
    ruka3::calibration_margin_ticks(ruka3::joint_profile(RUKA3_JOINT_INDEX).observer.output_encoder_cpr));
static ruka3::ServiceMotion service_controller{ruka3::joint_profile(RUKA3_JOINT_INDEX).command,
    std::min(ruka3::joint_profile(RUKA3_JOINT_INDEX).motor.stall_current, ruka3::joint_profile(RUKA3_JOINT_INDEX).motor.nominal_current)};
static const auto& service_motion = service_controller.snapshot();
VBDrive* get_motor() {
    return motor;
}
const ruka3::CanFaultSnapshot& get_can_fault_snapshot() {
    return can_fault_monitor.snapshot();
}
const ruka3::FaultSnapshot& get_fault_snapshot() {
    return fault_manager.snapshot();
}
const ruka3::CommandSnapshot& get_command_snapshot() {
    return command_supervisor.snapshot();
}
uint32_t get_joint_state() {
    return static_cast<uint32_t>(joint_controller.state());
}
bool is_motor_enabled() {
    return joint_controller.accepts_motor_commands();
}
const ruka3::OutputObserverSnapshot& get_output_observer_snapshot() { return output_observer.snapshot(); }
const ruka3::MechanicalCalibrationSnapshot& get_mechanical_calibration_snapshot() {
    return mechanical_calibration.snapshot();
}
const ruka3::ServiceMotionSnapshot& get_service_motion_snapshot() { return service_motion; }
ruka3::ServiceMotion& joint_motion_instance() { return service_controller; }
ruka3::JointCalibration& joint_calibration_instance() { return mechanical_calibration; }
std::uint16_t calibration_board_raw() { return motor->get_shaft_encoder_value(); }
float calibration_board_angle() { return kPositionDirection * motor->get_angle(); }
float calibration_board_velocity() { return kPositionDirection * motor->get_velocity(); }
bool calibration_board_available() {
    return motor && motor_calibration_valid && !motor_calibration_pending && fault_manager.snapshot().latched == 0;
}
bool calibration_board_enable() {
    service_cancel();
    motor->set_voltage_point(0.0F);
    return motor->set_bridge_outputs_enabled(true) == HAL_OK;
}
void calibration_board_disable() {
    service_cancel();
    if (motor) { motor->set_foc_point(FOCTarget{0}); (void)motor->set_bridge_outputs_enabled(false); }
    joint_controller.disarm();
}
void calibration_board_current(float current) {
    constexpr auto& profile = ruka3::joint_profile(RUKA3_JOINT_INDEX);
    motor->set_foc_point(FOCTarget{.torque = kPositionDirection * current * profile.motor.torque_const * profile.gear_ratio});
}
void calibration_board_sync() {
    mechanical_calibration.refresh();
    output_observer.set_encoder_zero(mechanical_calibration.snapshot().encoder_zero);
    const auto& limits = mechanical_calibration.snapshot();
    if (limits.limits_valid) output_observer.set_calibrated_limits(limits.lower_limit, limits.upper_limit);
}
static void update_fault_manager();
static void update_service_motion();

static float output_outward_command_scale(float command) {
    return ruka3::outward_command_scale(command, output_observer.snapshot().fused_angle,
                                       mechanical_calibration.snapshot());
}
static float limit_output_directional_command(float command) {
    return command * output_outward_command_scale(command);
}

void service_cancel() {
    service_controller.cancel();
}

bool service_set_zero() {
    if (!motor || !output_observer.snapshot().initialized ||
        get_fault_snapshot().level == ruka3::FaultLevel::fault) return false;
    service_cancel();
    motor->set_foc_point(FOCTarget{0});
    set_motor_enabled(false);
    if (!mechanical_calibration.set_zero(static_cast<uint16_t>(motor->get_shaft_encoder_value()))) return false;
    output_observer.set_encoder_zero(mechanical_calibration.snapshot().encoder_zero);
    if (mechanical_calibration.snapshot().limits_valid) {
        output_observer.set_calibrated_limits(mechanical_calibration.snapshot().lower_limit,
                                              mechanical_calibration.snapshot().upper_limit);
    }
    output_observer.update(millis_32(), motor->get_shaft_encoder_value(),
                           kPositionDirection * motor->get_angle());
    return true;
}

bool service_set_limit(uint32_t stage) {
    if (!motor || joint_controller.accepts_motor_commands() ||
        get_fault_snapshot().level == ruka3::FaultLevel::fault) return false;
    const bool saved = mechanical_calibration.set_limit(stage,
        static_cast<uint16_t>(motor->get_shaft_encoder_value()));
    if (saved && stage == 3U && mechanical_calibration.snapshot().limits_valid) {
        output_observer.set_calibrated_limits(mechanical_calibration.snapshot().lower_limit,
                                              mechanical_calibration.snapshot().upper_limit);
    }
    return saved;
}

bool service_set_voltage(float voltage) {
    const auto now = millis_32();
    if (!motor || !joint_controller.accepts_motor_commands() ||
        !service_controller.direct(ruka3::ServiceMotionMode::voltage, voltage,
            ruka3::joint_profile(RUKA3_JOINT_INDEX).motor.max_voltage, mechanical_calibration.snapshot(), now)) return false;
    command_supervisor.note_accepted_command(now);
    update_service_motion();
    return service_motion.mode == ruka3::ServiceMotionMode::voltage;
}

bool service_set_current(float current) {
    const auto now = millis_32();
    if (!motor || !joint_controller.accepts_motor_commands() ||
        !service_controller.direct(ruka3::ServiceMotionMode::current, current,
            ruka3::joint_profile(RUKA3_JOINT_INDEX).motor.current_limit, mechanical_calibration.snapshot(), now)) return false;
    command_supervisor.note_accepted_command(now);
    update_service_motion();
    return service_motion.mode == ruka3::ServiceMotionMode::current;
}

bool service_set_position(float position) {
    const auto now = millis_32();
    if (!motor || !joint_controller.accepts_motor_commands() || !output_observer.snapshot().initialized ||
        !service_controller.position(position, output_observer.snapshot().fused_angle,
                                      mechanical_calibration.snapshot(), now)) return false;
    command_supervisor.note_accepted_command(now);
    update_service_motion();
    return service_motion.mode == ruka3::ServiceMotionMode::position;
}

bool service_set_servo(float position, float velocity, float torque, float kp, float kd) {
    const auto now = millis_32();
    if (!motor || !joint_controller.accepts_motor_commands() || !output_observer.snapshot().initialized ||
        !service_controller.servo(position, velocity, torque, kp, kd, output_observer.snapshot().fused_angle,
                                  mechanical_calibration.snapshot(), now)) return false;
    command_supervisor.note_accepted_command(now);
    update_service_motion();
    return service_motion.mode == ruka3::ServiceMotionMode::servo;
}

void service_hold() {
    service_controller.hold(output_observer.snapshot().fused_angle, millis_32());
    update_service_motion();
}

static void update_service_motion() {
    if (!motor || !joint_controller.accepts_motor_commands() ||
        get_fault_snapshot().level == ruka3::FaultLevel::fault) return;
    const auto now = millis_32();
    const auto& output = output_observer.snapshot();
    const auto& limits = mechanical_calibration.snapshot();
    const bool was_running = service_controller.autonomous_motion_active();
    if (!service_controller.update(now, output.fused_angle, limits)) return;
    if (was_running && !service_controller.autonomous_motion_active()) command_supervisor.complete_autonomous_command(now);
    constexpr auto& profile = ruka3::joint_profile(RUKA3_JOINT_INDEX);
    bool valid;
    CRITICAL_SECTION({
    if (service_motion.mode == ruka3::ServiceMotionMode::voltage) {
        valid = motor->set_voltage_point(kPositionDirection * profile.motor_command_sign *
            service_motion.target * ruka3::outward_command_scale(service_motion.target, output.fused_angle, limits));
    } else if (service_motion.mode == ruka3::ServiceMotionMode::current) {
        valid = motor->set_foc_point(FOCTarget{.torque = kPositionDirection *
            limit_output_directional_command(service_motion.target) * profile.motor.torque_const * profile.gear_ratio});
    } else {
        const float amp_to_torque = profile.motor.torque_const * profile.gear_ratio;
        const float base_current = ruka3::position_current_request(service_motion, output.fused_velocity,
            profile.motor.torque_const, profile.gear_ratio);
        const float current_limit = std::min(profile.motor.nominal_current, profile.motor.torque_limit/amp_to_torque);
        const float load_current = service_controller.compensate_load(now, output.fused_angle, base_current, current_limit, limits);
        valid = motor->set_foc_point(FOCTarget{
            .torque = kPositionDirection * (service_motion.torque + load_current*amp_to_torque),
            .angle = motor->get_angle() + kPositionDirection * service_motion.error,
            .velocity = motor->get_velocity() + kPositionDirection * (service_motion.velocity-output.fused_velocity),
            // VBDrive divides P/D by Kt, but feedforward by Kt*gear.
            .angle_kp = service_motion.kp / profile.gear_ratio,
            .velocity_kp = service_motion.kd / profile.gear_ratio,
        });
    }
    })
    if (!valid) service_cancel();
}

bool set_motor_enabled(bool enabled) {
    if (!motor) return false;
    if (!enabled) {
        service_cancel();
        const bool target_set = motor->set_foc_point(FOCTarget{
            .torque = 0.0F,
            .angle = motor->get_angle(),
            .velocity = 0.0F,
            .angle_kp = 0.0F,
            .velocity_kp = 0.0F,
        });
        const bool outputs_disabled = motor->set_bridge_outputs_enabled(false) == HAL_OK;
        joint_controller.disarm();
        return target_set && outputs_disabled;
    }
    if (joint_controller.accepts_motor_commands() && motor->is_on()) return true;
    if (mechanical_calibration.core().blocks_normal_control() || !motor_calibration_valid || !mechanical_calibration.snapshot().limits_valid ||
        !joint_controller.arm()) {
        return false;
    }
    // Keep the bridge at zero voltage until PWM is enabled; only then engage
    // position hold around the angle measured at that moment.
    motor->set_voltage_point(0.0F);
    if (motor->set_bridge_outputs_enabled(true) != HAL_OK) {
        joint_controller.disarm();
        return false;
    }
    service_hold();
    command_supervisor.note_accepted_command(millis_32());
    return true;
}

bool acknowledge_faults() {
    if (!fault_manager.acknowledge()) {
        return false;
    }
    return joint_controller.acknowledge_fault(motor_calibration_valid);
}

static void update_fault_manager() {
    // STSPIN asserts DRV_FAULT while deliberately held in standby. When it
    // is awake, keep sampling the pin even after disabling PWM so a real
    // driver fault cannot be acknowledged while it is still present.
    const bool driver_fault = driver_awake &&
        (HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin) == GPIO_PIN_RESET);
    const auto& inverter = static_cast<const VBInverter&>(motor->get_inverter());
    const ruka3::FaultInputs inputs{
        .can = can_fault_monitor.snapshot(),
        .calibration_valid = motor_calibration_valid,
        .driver_fault = driver_fault,
        .mcu_temperature_c = inverter.get_mcu_temperature(),
        .stator_temperature_c = inverter.get_stator_temperature(),
    };
    // Communication state is diagnostic only. The transport owns CAN recovery
    // and the command supervisor holds the current target when commands stop.
    // A local power-stage fault disables PWM but leaves STSPIN awake so CAN
    // communication remains available for diagnosis and recovery.
    if (fault_manager.update(inputs) && driver_fault) {
        service_cancel();
        motor->set_foc_point(FOCTarget{0});
        (void)motor->set_bridge_outputs_enabled(false);
        joint_controller.fault();
    }

    const auto level = fault_manager.snapshot().level;
    set_cyphal_health(level == ruka3::FaultLevel::fault || level == ruka3::FaultLevel::degraded
            ? uavcan_node_Health_1_0_CAUTION
            : level == ruka3::FaultLevel::warning
                ? uavcan_node_Health_1_0_ADVISORY
                : uavcan_node_Health_1_0_NOMINAL);
}

bool is_motor_calibrated() { return motor_calibration_valid; }
int32_t get_motor_calibration_offset() { return calibration_data.meas_elec_offset; }
bool is_motor_encoder_inverted() { return calibration_data.is_encoder_inverted; }
bool is_motor_calibration_pending() { return motor_calibration_pending; }
bool request_motor_calibration() {
    if (!motor || joint_controller.accepts_motor_commands() || motor_calibration_pending ||
        mechanical_calibration.core().blocks_normal_control()) return false;
    motor_calibration_pending = true;
    motor_calibration_requested_at = millis_32();
    return true;
}
bool is_driver_awake() { return driver_awake; }
bool set_driver_awake(bool awake) {
    if (!motor || motor->is_on()) return false;
    const HAL_StatusTypeDef status = awake
        ? motor_gate_driver.wake()
        : motor_gate_driver.request_standby();
    if (status != HAL_OK) return false;
    driver_awake = awake;
    return true;
}
float get_motor_electric_angle() { return motor ? motor->get_electric_angle() : 0.0F; }

void create_motor() {
    constexpr auto& profile = ruka3::joint_profile(RUKA3_JOINT_INDEX);
    constexpr auto& motor_profile = profile.motor;
    motor = new (&motor_storage) VBDrive(
        motor_profile.control_period_s,
        // Kalman filter for determining electric angle
        FiltersConfig {
            .expected_a = motor_profile.filter_expected_a,
            .g1 = motor_profile.filter_g1,
            .g2 = motor_profile.filter_g2,
            .g3 = motor_profile.filter_g3,
            .I_lpf_coefficient = motor_profile.current_lpf
        },
        // Q Regulator
        PIDConfig {
            .multiplier = 1.0f,
            .kp = motor_profile.current_pid_kp,
            .ki = motor_profile.current_pid_ki,
            .kd = motor_profile.current_pid_kd,
            .integral_error_lim = motor_profile.max_voltage,
            .max_output = motor_profile.max_voltage,
            .min_output = -motor_profile.max_voltage,
        },
        // D Regulator
        PIDConfig {
            .multiplier = 1.0f,
            .kp = motor_profile.current_pid_kp,
            .ki = motor_profile.current_pid_ki,
            .kd = motor_profile.current_pid_kd,
            .integral_error_lim = motor_profile.max_voltage,
            .max_output = motor_profile.max_voltage,
            .min_output = -motor_profile.max_voltage,
        },
        // User-defined runtime config
        DriveRuntimeConfig {
            .user_current_limit = motor_profile.current_limit,
            .user_torque_limit = motor_profile.torque_limit,
            .user_speed_limit = profile.speed_limit > 0.0F ? profile.speed_limit : NAN,
            .user_position_lower_limit = NAN,
            .user_position_upper_limit = NAN,
            .user_angle_offset = profile.position_offset,
            .user_angle_direction = 1
        },
        // Built-in constant parameters
        DriveInfo {
            .torque_const = motor_profile.torque_const,
            .max_current = motor_profile.current_limit,
            .max_torque = motor_profile.torque_limit,
            .stall_current = motor_profile.stall_current,
            .stall_timeout = motor_profile.stall_timeout_s,
            .stall_tolerance = motor_profile.stall_tolerance,
            .calibration_voltage = motor_profile.calibration_voltage,
            .en_pin = GpioPin(DRV_WAKE_GPIO_Port, DRV_WAKE_Pin),
            .common = {
                .ppairs = motor_profile.pole_pairs,
                .gear_ratio = static_cast<uint8_t>(profile.gear_ratio)
            }
        },
        &htim1,
        motor_encoder,
        motor_inverter,
        motor_gate_driver,
        inductive_sensor,
        AngleEncoderType::ROTOR
    );
    HAL_Delay(100);
    motor->init();
}

bool is_able_to_calibrate() {
    return motor != nullptr;
}

bool do_calibrate() {
    // Stop all control
    motor->set_foc_point(FOCTarget{0});
    joint_controller.calibration_started();
    if (motor->set_state(true) != HAL_OK) {
        joint_controller.calibration_finished(false);
        return false;
    }
    calibration_data.reset();
    // NOTE: see app.h lines 20-21 for details on cyphal_queue_buffer_shared
    if (!motor->calibrate(calibration_data, cyphal_queue_buffer_shared, SHARED_BUFFER_SIZE)) {
        motor->set_bridge_outputs_enabled(false);
        motor_calibration_valid = false;
        joint_controller.calibration_finished(false);
        return false;
    }
    calibration_data.was_calibrated = true;
    HAL_IMPORTANT(eeprom.write<CalibrationData>(&calibration_data, CALIBRATION_PLACEMENT))
    motor->apply_calibration(calibration_data);
    motor_calibration_valid = true;
    joint_controller.calibration_finished(true);
    motor->set_foc_point(FOCTarget{0});
    motor->set_bridge_outputs_enabled(false);

    return true;
}

void apply_calibration() {
    motor_calibration_valid = false;
    if (calibration_data.type_id == 0) {  // uninitialized, try to read from EEPROM
        HAL_IMPORTANT(eeprom.read<CalibrationData>(&calibration_data, CALIBRATION_PLACEMENT))
    }
    if (calibration_data.type_id != CalibrationData::TYPE_ID || !calibration_data.was_calibrated) {
        joint_controller.configuration_loaded(false);
        return;
    }
    motor->apply_calibration(calibration_data);
    motor_calibration_valid = true;
    joint_controller.configuration_loaded(true);
}

static void reboot_to_bootloader_if_requested();

void app() {
//#pragma region StartupConfiguration
    start_timers();
    eeprom.wait_until_available();
    constexpr auto& profile = ruka3::joint_profile(RUKA3_JOINT_INDEX);
    mechanical_calibration.load(profile.output_encoder_zero);
    output_observer.set_calibration(&mechanical_calibration.core());
    output_observer.set_encoder_zero(mechanical_calibration.snapshot().encoder_zero);
    if (mechanical_calibration.snapshot().limits_valid) {
        output_observer.set_calibrated_limits(mechanical_calibration.snapshot().lower_limit,
                                              mechanical_calibration.snapshot().upper_limit);
    }

    // Keep CAN bring-up isolated from motor, encoder, and gate-driver startup.
    // This is the ordering validated by the stable G431 firmware.
    start_cyphal();

    setup_cordic();
    create_motor();
    motor->start();
    apply_calibration();
    motor->set_foc_point(FOCTarget{0});
    // Start with the gate driver awake but PWM outputs disabled.  Entering
    // STSPIN standby here makes reset/startup depend on external SWD wiring.
    HAL_IMPORTANT(motor->set_bridge_outputs_enabled(false))
    driver_awake = true;
    command_supervisor.initialize(millis_32(),
                                  ruka3::joint_profile(RUKA3_JOINT_INDEX).safety.command_timeout_ms);
    fault_manager.initialize();
    can_fault_monitor.poll();
    update_fault_manager();

    // Warm up
    for (uint8_t i = 0; i < 32; ++i) {
        motor->update();
    }

    set_cyphal_mode(uavcan_node_Mode_1_0_OPERATIONAL);

    // Lock heap, no dynamic memory is used at runtime
    global_allocation_lock = true;
//#pragma endregion

    HAL_TIM_Base_Start_IT(&htim4);
    const uint32_t auto_arm_at = millis_32() + 250U;
    // Disabled while diagnosing J5 power-stage interference. Arming remains
    // explicitly available through arm.
    bool auto_arm_pending = false;

    while(true) {
        calibration_service_update(millis_32());
        output_observer.update(millis_32(), motor->get_shaft_encoder_value(),
                               kPositionDirection * motor->get_angle());
        update_service_motion();
        can_fault_monitor.poll();
        update_register_diagnostics(millis_32());
        if (command_supervisor.update(millis_32(), service_controller.autonomous_motion_active()) &&
            !mechanical_calibration.core().blocks_normal_control() && joint_controller.accepts_motor_commands()) {
            service_hold();
        }
        update_fault_manager();
        if (auto_arm_pending && static_cast<int32_t>(millis_32() - auto_arm_at) >= 0) {
            auto_arm_pending = false;
            if (get_fault_snapshot().level != ruka3::FaultLevel::fault) {
                set_motor_enabled(true);
            }
        }
        cyphal_loop();
        if (reset_pending &&
            static_cast<int32_t>(millis_32() - reset_requested_at) >= 250) {
            NVIC_SystemReset();
        }
        // Let the register response leave the CAN queue before calibration
        // reuses its RAM workspace. Reboot immediately afterwards so the
        // Cyphal allocator never runs on overwritten memory.
        if (motor_calibration_pending &&
            static_cast<int32_t>(millis_32() - motor_calibration_requested_at) >= 500) {
            // Calibration drives and samples the motor synchronously. The
            // regular control ISR must not access the same motor/SPI objects,
            // and the calibration workspace aliases the Cyphal queue RAM.
            HAL_TIM_Base_Stop_IT(&htim4);
            (void)HAL_FDCAN_Stop(&hfdcan1);
            (void)do_calibrate();
            NVIC_SystemReset();
        }
        reboot_to_bootloader_if_requested();

        millis current_time = millis_32();
        monitor_loop(current_time);
    }
}

#ifndef NO_CYPHAL
//#pragma region Cyphal
using FOCState = voltbro_foc_state_simple_1_0;

static constexpr CanardPortID FOC_COMMAND_PORT =
    ruka3::joint_profile(RUKA3_JOINT_INDEX).servo_subject_id;
static constexpr CanardPortID FOC_STATE_PORT =
    ruka3::joint_profile(RUKA3_JOINT_INDEX).feedback_subject_id;
static void reboot_to_bootloader_if_requested() {
    if (!bootloader_reboot_pending) {
        return;
    }
    bootloader_reboot_pending = false;
    reboot_to_bootloader();
}

void in_loop_reporting(millis current_t) {
    if (motor == nullptr) {
        return;
    }
    const auto& communication = ruka3::joint_profile(RUKA3_JOINT_INDEX).communication;
    const millis feedback_period_ms = 1000U / communication.feedback_update_hz;
    const millis temperature_period_ms = 1000U / communication.temperature_update_hz;

    static millis temperature_time = 0;
    EACH_N(current_t, temperature_time, temperature_period_ms, {
        motor_inverter.update_temperature();
    })

    static millis report_time = 0;
    EACH_N(current_t, report_time, feedback_period_ms, {
        FOCState state_msg = {};

        state_msg.timestamp.microsecond = system_time();

        const auto& output = output_observer.snapshot();
        state_msg.angle.radian = output.fused_angle;
        state_msg.velocity.radian_per_second = output.fused_velocity;
        state_msg._torque.newton_meter = kPositionDirection * motor->get_torque();

        state_msg.current.ampere = kPositionDirection * motor->get_working_current();
        state_msg.bus_voltage.volt = motor_inverter.get_busV();

        constexpr float KELVIN_OFFSET = 273.15f;
        state_msg.mcu_temp.kelvin = motor_inverter.get_mcu_temperature() + KELVIN_OFFSET;
        state_msg.stator_temp.kelvin = motor_inverter.get_stator_temperature() + KELVIN_OFFSET;

        state_msg.has_fault.value = get_fault_snapshot().latched != 0;

        static CanardTransferID state_transfer_id = 0;
        get_interface()->send_msg(&state_msg, FOC_STATE_PORT, &state_transfer_id);
    })
}

class FOCCommandSub: public AbstractSubscription<voltbro_foc_command_1_0> {
public:
    FOCCommandSub(InterfacePtr interface, CanardPortID port_id): AbstractSubscription<voltbro_foc_command_1_0>(interface, port_id) {};
    void handler(const voltbro_foc_command_1_0& msg, CanardRxTransfer*) override {
        if (!service_set_servo(msg.angle.radian, msg.velocity.radian_per_second, msg._torque.newton_meter,
                               msg.angle_kp.value, msg.velocity_kp.value)) record_invalid_command();
    }
};

// NOTE: underlying CanardRxSubscriptions are HUGE - 552 bytes each. C++ wrapper size is negligible in comparison
ReservedObject<NodeInfoReader> node_info_reader;
ReservedObject<RegistersHandler<PARAMETER_CATALOG.size(), StaticRegisters<PARAMETER_CATALOG.size()>>> registers_handler;
ReservedObject<FOCCommandSub> foc_command_sub;


void setup_subscriptions() {
    auto cyphal_interface = get_interface();

    HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1,
        FDCAN_REJECT,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE
    );

    constexpr auto node_id = ruka3::joint_profile(RUKA3_JOINT_INDEX).node_id;
    registers_handler.create(
        StaticRegisters<PARAMETER_CATALOG.size()>{PARAMETER_NAMES, handle_parameter_register},
        cyphal_interface
    );

    node_info_reader.create(
        cyphal_interface,
        "org.voltbro.vbdrive",
        uavcan_node_Version_1_0{1, 0},
        uavcan_node_Version_1_0{1, 0},
        uavcan_node_Version_1_0{1, 0},
        VBDRIVE_VCS_REVISION_ID
    );

    foc_command_sub.create(cyphal_interface, FOC_COMMAND_PORT);

    HAL_IMPORTANT(apply_filter(
        0,
        &hfdcan1,
        foc_command_sub->make_filter(node_id)
    ))

    HAL_IMPORTANT(apply_filter(
        1,
        &hfdcan1,
        registers_handler->make_filter(node_id)
    ))

    HAL_IMPORTANT(apply_filter(
        2,
        &hfdcan1,
        node_info_reader->make_filter(node_id)
    ))

    // Keep the existing message-RAM layout; disable the retired SpecificControl filter.
    FDCAN_FilterTypeDef retired_filter{};
    retired_filter.IdType = FDCAN_EXTENDED_ID;
    retired_filter.FilterIndex = 3;
    retired_filter.FilterType = FDCAN_FILTER_MASK;
    retired_filter.FilterConfig = FDCAN_FILTER_DISABLE;
    HAL_IMPORTANT(HAL_FDCAN_ConfigFilter(&hfdcan1, &retired_filter))

}
//#pragma endregion
#endif
