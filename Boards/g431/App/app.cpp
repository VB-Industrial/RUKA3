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
#include <voltbro/foc/MIT_1_0.hpp>
#include <voltbro/foc/specific_control_1_0.hpp>
#include <voltbro/foc/state_simple_1_0.hpp>

#include <voltbro/eeprom/eeprom.hpp>
#include <voltbro/encoders/ASxxxx/AS5047P.hpp>
#include <voltbro/motors/bldc/vbdrive/vbdrive.hpp>
#include <voltbro/utils.hpp>
#include "robot_config.h"
#include "joint_controller.hpp"
#include "output_state_observer.hpp"
#include "parameters.hpp"
#include "protocol.h"
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
});
static ruka3::MechanicalCalibration mechanical_calibration(
    eeprom,
    ruka3::joint_profile(RUKA3_JOINT_INDEX).observer.output_encoder_cpr,
    static_cast<std::int8_t>(ruka3::joint_profile(RUKA3_JOINT_INDEX).output_encoder_sign *
                             ruka3::joint_profile(RUKA3_JOINT_INDEX).position_direction));
static ruka3::ServiceMotionSnapshot service_motion;
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
static void update_fault_manager();
static void update_service_motion();

static float output_outward_command_scale(float command) {
    const auto& limits = mechanical_calibration.snapshot();
    const auto& output = output_observer.snapshot();
    if (!limits.limits_valid || !output.initialized) return 0.0F;
    if (command > 0.0F) {
        if (output.fused_angle >= limits.upper_limit) return 0.0F;
        if (output.fused_angle > limits.soft_upper_limit) {
            return std::fmax(0.0F, (limits.upper_limit - output.fused_angle) /
                (limits.upper_limit - limits.soft_upper_limit));
        }
    } else if (command < 0.0F) {
        if (output.fused_angle <= limits.lower_limit) return 0.0F;
        if (output.fused_angle < limits.soft_lower_limit) {
            return std::fmax(0.0F, (output.fused_angle - limits.lower_limit) /
                (limits.soft_lower_limit - limits.lower_limit));
        }
    }
    return 1.0F;
}

static float limit_output_directional_command(float command) {
    return command * output_outward_command_scale(command);
}

static float limit_direct_voltage(float voltage) {
    constexpr auto& profile = ruka3::joint_profile(RUKA3_JOINT_INDEX);
    return voltage * output_outward_command_scale(
        kPositionDirection * static_cast<float>(profile.motor_command_sign) * voltage);
}

static float output_soft_limited_target(float target) {
    const auto& limits = mechanical_calibration.snapshot();
    const float position = output_observer.snapshot().fused_angle;
    // A soft limit is a one-way gate, not a prohibited target range.  A
    // command may enter it; once already inside, only a command back towards
    // the working range may change the position target.
    if (position > limits.soft_upper_limit && target > position) {
        return position;
    }
    if (position < limits.soft_lower_limit && target < position) {
        return position;
    }
    return target;
}

void service_cancel() {
    service_motion = {};
}

bool service_set_zero() {
    if (!motor || !output_observer.snapshot().initialized ||
        get_fault_snapshot().level == ruka3::FaultLevel::fault) return false;
    service_cancel();
    motor->set_foc_point(FOCTarget{0});
    set_motor_enabled(false);
    if (!mechanical_calibration.set_zero(static_cast<uint16_t>(motor->get_shaft_encoder_value()))) return false;
    output_observer.set_encoder_zero(mechanical_calibration.snapshot().encoder_zero);
    output_observer.update(millis_32(), motor->get_shaft_encoder_value(),
                           kPositionDirection * motor->get_angle());
    return true;
}

bool service_set_limit(uint32_t stage) {
    if (!motor || joint_controller.accepts_motor_commands() ||
        get_fault_snapshot().level == ruka3::FaultLevel::fault) return false;
    return mechanical_calibration.set_limit(stage, static_cast<uint16_t>(motor->get_shaft_encoder_value()));
}

bool service_set_voltage(float voltage) {
    if (!motor || !mechanical_calibration.snapshot().limits_valid || !std::isfinite(voltage) || !joint_controller.accepts_motor_commands() ||
        std::fabs(voltage) > ruka3::joint_profile(RUKA3_JOINT_INDEX).motor.max_voltage) return false;
    if (voltage == 0.0F) {
        service_cancel();
        return motor->set_voltage_point(0.0F);
    }
    service_motion.mode = ruka3::ServiceMotionMode::voltage;
    service_motion.target = voltage;
    service_motion.error = 0.0F;
    return true;
}

bool service_set_position(float position) {
    const auto& limits = mechanical_calibration.snapshot();
    if (!motor || !std::isfinite(position) || !joint_controller.accepts_motor_commands() ||
        !limits.limits_valid || !output_observer.snapshot().initialized ||
        position < limits.lower_limit || position > limits.upper_limit) return false;
    const float start = output_observer.snapshot().fused_angle;
    const float delta = position - start;
    const float distance = std::fabs(delta);
    const auto& tuning = ruka3::joint_profile(RUKA3_JOINT_INDEX).command;
    const float max_velocity = tuning.service_position_velocity_rad_s;
    const float acceleration = tuning.service_position_acceleration_rad_s2;
    const std::uint16_t update_hz = tuning.service_position_update_hz;
    if (max_velocity <= 0.0F || acceleration <= 0.0F || update_hz == 0U || update_hz > 1000U) return false;

    const float full_acceleration_distance = max_velocity * max_velocity / acceleration;
    const bool triangular = distance <= full_acceleration_distance;
    const float peak_velocity = triangular ? std::sqrt(distance * acceleration) : max_velocity;
    const float acceleration_time = peak_velocity / acceleration;
    const float cruise_time = triangular ? 0.0F : (distance - full_acceleration_distance) / max_velocity;
    const float total_time = acceleration_time * 2.0F + cruise_time;
    const std::uint32_t started_ms = millis_32();

    service_motion.mode = ruka3::ServiceMotionMode::position;
    while (true) {
        const float elapsed = static_cast<float>(millis_32() - started_ms) / 1000.0F;
        if (elapsed >= total_time) break;
        float travelled = 0.0F;
        if (elapsed < acceleration_time) {
            travelled = 0.5F * acceleration * elapsed * elapsed;
        } else if (elapsed < acceleration_time + cruise_time) {
            travelled = 0.5F * acceleration * acceleration_time * acceleration_time +
                peak_velocity * (elapsed - acceleration_time);
        } else {
            const float braking_time = total_time - elapsed;
            travelled = distance - 0.5F * acceleration * braking_time * braking_time;
        }
        service_motion.target = start + std::copysign(travelled, delta);
        output_observer.update(millis_32(), motor->get_shaft_encoder_value(),
                               kPositionDirection * motor->get_angle());
        can_fault_monitor.poll();
        update_fault_manager();
        update_service_motion();
        if (service_motion.mode != ruka3::ServiceMotionMode::position ||
            !joint_controller.accepts_motor_commands()) return false;
        HAL_Delay(1000U / update_hz);
    }
    service_motion.target = position;
    output_observer.update(millis_32(), motor->get_shaft_encoder_value(),
                           kPositionDirection * motor->get_angle());
    update_service_motion();
    return service_motion.mode == ruka3::ServiceMotionMode::position;
}

static void update_service_motion() {
    if (service_motion.mode == ruka3::ServiceMotionMode::idle || !motor ||
        !joint_controller.accepts_motor_commands() || get_fault_snapshot().level == ruka3::FaultLevel::fault) return;
    bool valid = false;
    if (service_motion.mode == ruka3::ServiceMotionMode::voltage) {
        valid = motor->set_voltage_point(limit_direct_voltage(service_motion.target));
    } else {
        const auto& output = output_observer.snapshot();
        const float target = output_soft_limited_target(service_motion.target);
        service_motion.error = target - output.fused_angle;
        const float motor_target = motor->get_angle() + kPositionDirection * service_motion.error;
        const auto& tuning = ruka3::joint_profile(RUKA3_JOINT_INDEX).command;
        valid = motor->set_foc_point(FOCTarget{
            .torque = 0.0F, .angle = motor_target, .velocity = 0.0F,
            .angle_kp = tuning.default_position_kp,
            .velocity_kp = tuning.default_velocity_kp,
        });
    }
    if (valid) command_supervisor.note_accepted_command(millis_32());
    else service_cancel();
}

bool set_motor_enabled(bool enabled) {
    if (!motor) return false;
    const auto& tuning = ruka3::joint_profile(RUKA3_JOINT_INDEX).command;
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
    if (!motor_calibration_valid || !mechanical_calibration.snapshot().limits_valid ||
        !joint_controller.arm()) {
        return false;
    }
    if (!motor->set_foc_point(FOCTarget{
            .torque = 0.0F, .angle = motor->get_angle(), .velocity = 0.0F,
            .angle_kp = tuning.default_position_kp,
            .velocity_kp = tuning.default_velocity_kp,
        })) {
        joint_controller.disarm();
        return false;
    }
    if (motor->set_bridge_outputs_enabled(true) != HAL_OK) {
        motor->set_foc_point(FOCTarget{0});
        joint_controller.disarm();
        return false;
    }
    return true;
}

bool acknowledge_faults() {
    if (!fault_manager.acknowledge()) {
        return false;
    }
    return joint_controller.acknowledge_fault(motor_calibration_valid);
}

static void update_fault_manager() {
    // STSPIN asserts DRV_FAULT while it is deliberately held in standby.
    // A disabled bridge is therefore an expected state, not a hardware fault.
    const bool driver_fault = motor->is_on() &&
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
    // Only a local power-stage fault may put the bridge in standby.
    if (fault_manager.update(inputs) && driver_fault) {
        motor->set_foc_point(FOCTarget{0});
        motor->set_state(false);
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
    if (!motor || joint_controller.accepts_motor_commands() || motor_calibration_pending) return false;
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
    motor->calibrate(calibration_data, cyphal_queue_buffer_shared, SHARED_BUFFER_SIZE);
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
    output_observer.set_encoder_zero(mechanical_calibration.snapshot().encoder_zero);

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
    // explicitly available through set_arm.
    bool auto_arm_pending = false;

    while(true) {
        output_observer.update(millis_32(), motor->get_shaft_encoder_value(),
                               kPositionDirection * motor->get_angle());
        update_service_motion();
        can_fault_monitor.poll();
        if (command_supervisor.update(millis_32())) {
            service_cancel();
            const auto& tuning = ruka3::joint_profile(RUKA3_JOINT_INDEX).command;
            motor->set_foc_point(FOCTarget{
                .torque = 0.0F, .angle = motor->get_angle(), .velocity = 0.0F,
                .angle_kp = tuning.default_position_kp,
                .velocity_kp = 0.0F,
            });
        }
        update_fault_manager();
        if (auto_arm_pending && static_cast<int32_t>(millis_32() - auto_arm_at) >= 0) {
            auto_arm_pending = false;
            if (get_fault_snapshot().level != ruka3::FaultLevel::fault) {
                set_motor_enabled(true);
            }
        }
        cyphal_loop();
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
using SpecificControl = voltbro_foc_specific_control_1_0;

static constexpr CanardPortID FOC_COMMAND_PORT = ruka3::protocol::kMitCommandBase;
static constexpr CanardPortID FOC_STATE_PORT =
    ruka3::joint_profile(RUKA3_JOINT_INDEX).feedback_subject_id;
static constexpr CanardPortID SPECIFIC_CONTROL_PORT = ruka3::protocol::kSpecificControlBase;

static bool is_voltage_target_valid(float voltage) {
    constexpr auto& motor_profile = ruka3::joint_profile(RUKA3_JOINT_INDEX).motor;
    return std::isfinite(voltage) && std::fabs(voltage) <= motor_profile.max_voltage;
}

static bool is_output_position_target_valid(float target) {
    const auto& output = output_observer.snapshot();
    const auto& limits = mechanical_calibration.snapshot();
    return output.initialized && std::isfinite(target) &&
        limits.limits_valid && target >= limits.lower_limit && target <= limits.upper_limit;
}

static float output_target_to_motor_target(float output_target) {
    return motor->get_angle() + kPositionDirection *
        (output_target - output_observer.snapshot().fused_angle);
}

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
        state_msg.angle.radian = output.initialized ? output.fused_angle : kPositionDirection * motor->get_angle();
        state_msg.velocity.radian_per_second = output.initialized ? output.fused_velocity : kPositionDirection * motor->get_velocity();
        state_msg._torque.newton_meter = motor->get_torque();

        state_msg.current.ampere = motor->get_working_current();
        state_msg.bus_voltage.volt = motor_inverter.get_busV();

        constexpr float KELVIN_OFFSET = 273.15f;
        state_msg.mcu_temp.kelvin = motor_inverter.get_mcu_temperature() + KELVIN_OFFSET;
        state_msg.stator_temp.kelvin = motor_inverter.get_stator_temperature() + KELVIN_OFFSET;

        state_msg.has_fault.value = get_fault_snapshot().level == ruka3::FaultLevel::fault;

        static CanardTransferID state_transfer_id = 0;
        get_interface()->send_msg(&state_msg, FOC_STATE_PORT, &state_transfer_id);
    })
}

class FOCCommandSub: public AbstractSubscription<voltbro_foc_MIT_1_0> {
public:
    FOCCommandSub(InterfacePtr interface, CanardPortID port_id): AbstractSubscription<voltbro_foc_MIT_1_0>(interface, port_id) {};
    void handler(const voltbro_foc_MIT_1_0& msg, CanardRxTransfer*) override {
        if (!joint_controller.accepts_motor_commands()) {
            record_invalid_command();
            return;
        }
        if (!is_output_position_target_valid(msg.pos.radian)) {
            record_invalid_command();
            return;
        }
        const float target = output_soft_limited_target(msg.pos.radian);
        bool is_valid = motor->set_foc_point(FOCTarget {
            .torque = kPositionDirection * limit_output_directional_command(msg._torq.newton_meter),
            .angle = output_target_to_motor_target(target),
            .velocity = kPositionDirection * limit_output_directional_command(msg.vel.radian_per_second),
            .angle_kp = msg.pos_gain.value,
            .velocity_kp = msg.vel_gain.value
        });
        if (!is_valid) {
            record_invalid_command();
        } else {
            service_cancel();
            command_supervisor.note_accepted_command(millis_32());
        }
    }
};

class SpecificControlSub: public AbstractSubscription<SpecificControl> {
public:
    SpecificControlSub(InterfacePtr interface, CanardPortID port_id): AbstractSubscription<SpecificControl>(interface, port_id) {};
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-parameter"
    // NOTE: transfer parameter required by the interface, but not used in this implementation
    void handler(const SpecificControl& msg, CanardRxTransfer* _) override {
        if (!joint_controller.accepts_motor_commands()) {
            record_invalid_command();
            return;
        }
    #pragma GCC diagnostic pop
        bool is_valid = false;
        switch (msg.set_point_type){
            case voltbro_foc_specific_control_1_0_VELOCITY:
                is_valid = motor->set_velocity_point(
                    kPositionDirection * limit_output_directional_command(msg.set_point_value));
                break;
            case voltbro_foc_specific_control_1_0_TORQUE:
                is_valid = motor->set_torque_point(
                    kPositionDirection * limit_output_directional_command(msg.set_point_value));
                break;
            case voltbro_foc_specific_control_1_0_POSITION:
                if (is_output_position_target_valid(msg.set_point_value)) {
                    const auto& tuning = ruka3::joint_profile(RUKA3_JOINT_INDEX).command;
                    const float target = output_soft_limited_target(msg.set_point_value);
                    is_valid = motor->set_foc_point(FOCTarget{
                        .torque = 0.0F, .angle = output_target_to_motor_target(target),
                        .velocity = 0.0F, .angle_kp = tuning.default_position_kp,
                        .velocity_kp = tuning.default_velocity_kp,
                    });
                }
                break;
            case voltbro_foc_specific_control_1_0_VOLTAGE:
                is_valid = is_voltage_target_valid(msg.set_point_value) &&
                    motor->set_voltage_point(limit_direct_voltage(msg.set_point_value));
                break;
            default:
                break;
        }
        if (!is_valid) {
            record_invalid_command();
        } else {
            service_cancel();
            command_supervisor.note_accepted_command(millis_32());
        }
    }
};

// NOTE: underlying CanardRxSubscriptions are HUGE - 552 bytes each. C++ wrapper size is negligible in comparison
ReservedObject<NodeInfoReader> node_info_reader;
ReservedObject<RegistersHandler<PARAMETER_CATALOG.size(), StaticRegisters<PARAMETER_CATALOG.size()>>> registers_handler;
ReservedObject<FOCCommandSub> foc_command_sub;
ReservedObject<SpecificControlSub> specific_control_sub;

static void handle_parameter_register(
    size_t index,
    const uavcan_register_Value_1_0& v_in,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse& response
) {
    const auto& definition = PARAMETER_CATALOG[index];
    response.persistent = definition.is_persistent;
    response._mutable = definition.is_mutable;

    // Keep RUKA2's service convention: a read of set_zero is an explicit
    // calibration trigger, with its result carried by the same response.
    if (definition.id == ParameterId::ZERO_SET && v_in._tag_ == REGISTER_EMPTY_TAG) {
        fill_register_integer32(v_out, service_set_zero() ? 1 : 0);
        return;
    }

    if (definition.is_mutable && v_in._tag_ != REGISTER_EMPTY_TAG) {
        ParameterValue requested{};
        bool parsed = false;
        switch (definition.type) {
            case ParameterType::REAL32:
                parsed = parse_register_real32(v_in, requested.emplace<float>());
                break;
            case ParameterType::REAL32_ARRAY:
                break;
            case ParameterType::NATURAL32:
                parsed = parse_register_natural32(v_in, requested.emplace<uint32_t>());
                if (!parsed) {
                    int32_t signed_value = 0;
                    parsed = parse_register_integer32(v_in, signed_value) && signed_value >= 0;
                    if (parsed) requested = static_cast<uint32_t>(signed_value);
                }
                break;
            case ParameterType::INTEGER32:
                parsed = parse_register_integer32(v_in, requested.emplace<int32_t>());
                break;
            case ParameterType::BIT:
                parsed = parse_register_bit(v_in, requested.emplace<bool>());
                if (!parsed) {
                    int32_t signed_value = 0;
                    uint32_t unsigned_value = 0;
                    float real_value = 0;
                    if (parse_register_integer32(v_in, signed_value)) {
                        requested = signed_value != 0;
                        parsed = true;
                    } else if (parse_register_natural32(v_in, unsigned_value)) {
                        requested = unsigned_value != 0;
                        parsed = true;
                    } else if (parse_register_real32(v_in, real_value)) {
                        requested = real_value != 0;
                        parsed = true;
                    }
                }
                break;
            case ParameterType::STRING:
                break;
        }
        if (parsed) (void)write_runtime_parameter(definition.id, requested);
    }

    ParameterValue current{};
    if (!read_parameter(definition.id, current)) {
        v_out._tag_ = REGISTER_EMPTY_TAG;
        v_out.empty = {};
        return;
    }
    switch (definition.type) {
        case ParameterType::REAL32:
            fill_register_real32(v_out, std::get<float>(current));
            break;
        case ParameterType::REAL32_ARRAY: {
            const auto& values = std::get<AngleDiagnostics>(current);
            v_out._tag_ = REGISTER_REAL32_TAG;
            for (size_t i = 0; i < values.size(); ++i) v_out.real32.value.elements[i] = values[i];
            v_out.real32.value.count = values.size();
            break;
        }
        case ParameterType::NATURAL32:
            fill_register_natural32(v_out, std::get<uint32_t>(current));
            break;
        case ParameterType::INTEGER32:
            fill_register_integer32(v_out, std::get<int32_t>(current));
            break;
        case ParameterType::BIT:
            fill_register_bit(v_out, std::get<bool>(current));
            break;
        case ParameterType::STRING:
            fill_register_string(v_out, std::get<std::string_view>(current));
            break;
    }
}

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

    specific_control_sub.create(cyphal_interface, SPECIFIC_CONTROL_PORT + node_id);
    foc_command_sub.create(cyphal_interface, FOC_COMMAND_PORT + node_id);

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

    HAL_IMPORTANT(apply_filter(
        3,
        &hfdcan1,
        specific_control_sub->make_filter(node_id)
    ))
}
//#pragma endregion
#endif
