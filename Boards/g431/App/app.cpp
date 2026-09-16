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

#ifdef STACK_PROFILE
extern uint32_t __StackLimit;
extern uint32_t __StackTop;
constexpr uint32_t STACK_CANARY = 0xDEADBEEF;
volatile static size_t max_stack_usage = 0;

void mark_stack() {
    uint32_t current_sp;
    __asm__ volatile ("mov %0, sp" : "=r" (current_sp));

    volatile uint32_t *p = static_cast<volatile uint32_t*>(&__StackLimit);
    volatile uint32_t *sp = reinterpret_cast<volatile uint32_t*>(current_sp);

    while (p < sp) {
        *p = STACK_CANARY;
        p++;
    }
}

void measure_stack_usage() {
    volatile uint32_t *p = static_cast<volatile uint32_t*>(&__StackLimit);
    volatile uint32_t *top = static_cast<volatile uint32_t*>(&__StackTop);

    while (p < top && *p == STACK_CANARY) {
        p++;
    }

    size_t unused = static_cast<size_t>(p - &__StackLimit);
    size_t total = static_cast<size_t>(&__StackTop - &__StackLimit);
    size_t used = total - unused;

    if (used > max_stack_usage) {
        max_stack_usage = used;
    }
}
#endif

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
static ruka3::JointController joint_controller;
static ruka3::CanFaultMonitor can_fault_monitor{hfdcan1};
static ruka3::CommandSupervisor command_supervisor;
static ruka3::FaultManager fault_manager;
static ruka3::OutputStateObserver output_observer({
    ruka3::joint_profile(RUKA3_JOINT_INDEX).observer.output_encoder_cpr,
    ruka3::joint_profile(RUKA3_JOINT_INDEX).output_encoder_zero,
    ruka3::joint_profile(RUKA3_JOINT_INDEX).output_encoder_sign,
    ruka3::joint_profile(RUKA3_JOINT_INDEX).observer.output_velocity_lpf,
    ruka3::joint_profile(RUKA3_JOINT_INDEX).observer.fusion_encoder_corridor_ticks,
});
static ruka3::MechanicalCalibration mechanical_calibration(
    eeprom,
    ruka3::joint_profile(RUKA3_JOINT_INDEX).observer.output_encoder_cpr,
    ruka3::joint_profile(RUKA3_JOINT_INDEX).output_encoder_sign);
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
const ruka3::OutputObserverSnapshot& get_output_observer_snapshot() { return output_observer.snapshot(); }
const ruka3::MechanicalCalibrationSnapshot& get_mechanical_calibration_snapshot() {
    return mechanical_calibration.snapshot();
}
const ruka3::ServiceMotionSnapshot& get_service_motion_snapshot() { return service_motion; }

void service_cancel() {
    service_motion = {};
}

static bool apply_runtime_limits() {
    if (!motor) return false;
    const auto& limits = mechanical_calibration.snapshot();
    auto config = motor->get_runtime_config();
    config.user_position_lower_limit = limits.limits_valid ? limits.lower_limit : NAN;
    config.user_position_upper_limit = limits.limits_valid ? limits.upper_limit : NAN;
    return motor->set_runtime_config(config);
}

bool service_set_zero() {
    if (!motor || !output_observer.snapshot().initialized ||
        get_fault_snapshot().level == ruka3::FaultLevel::fault) return false;
    service_cancel();
    motor->set_foc_point(FOCTarget{0});
    set_motor_enabled(false);
    if (!mechanical_calibration.set_zero(static_cast<uint16_t>(motor->get_shaft_encoder_value()))) return false;
    output_observer.set_encoder_zero(mechanical_calibration.snapshot().encoder_zero);
    output_observer.update(millis_32(), motor->get_shaft_encoder_value(), motor->get_angle());
    return apply_runtime_limits();
}

bool service_set_limit(uint32_t stage) {
    if (!motor || motor->is_on() || get_fault_snapshot().level == ruka3::FaultLevel::fault) return false;
    const bool result = mechanical_calibration.set_limit(stage, static_cast<uint16_t>(motor->get_shaft_encoder_value()));
    return result && (stage != 3U || apply_runtime_limits());
}

bool service_set_voltage(float voltage) {
    if (!motor || !std::isfinite(voltage) || !joint_controller.accepts_motor_commands() ||
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
        !output_observer.snapshot().initialized ||
        (limits.limits_valid && (position < limits.lower_limit || position > limits.upper_limit))) return false;
    service_motion.mode = ruka3::ServiceMotionMode::position;
    service_motion.target = position;
    service_motion.error = position - output_observer.snapshot().fused_angle;
    return true;
}

static void update_service_motion() {
    if (service_motion.mode == ruka3::ServiceMotionMode::idle || !motor ||
        !joint_controller.accepts_motor_commands() || get_fault_snapshot().level == ruka3::FaultLevel::fault) return;
    bool valid = false;
    if (service_motion.mode == ruka3::ServiceMotionMode::voltage) {
        valid = motor->set_voltage_point(service_motion.target);
    } else {
        const auto& output = output_observer.snapshot();
        service_motion.error = service_motion.target - output.fused_angle;
        const float motor_target = motor->get_angle() + service_motion.error;
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
    if (!enabled) {
        service_cancel();
        motor->set_foc_point(FOCTarget{0});
        joint_controller.disarm();
        return motor->set_state(false) == HAL_OK;
    }
    if (!motor_calibration_valid || !joint_controller.arm()) {
        return false;
    }
    if (motor->set_state(true) == HAL_OK) {
        return true;
    }
    joint_controller.disarm();
    return false;
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
    if (fault_manager.update(inputs)) {
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
float get_motor_electric_angle() { return motor ? motor->get_electric_angle() : 0.0F; }

void create_motor() {
    constexpr auto& profile = ruka3::joint_profile(RUKA3_JOINT_INDEX);
    constexpr auto& motor_profile = profile.motor;
    constexpr auto angle_encoder = motor_profile.position_source == ruka3::PositionSource::shaft
        ? AngleEncoderType::SHAFT : AngleEncoderType::ROTOR;
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
            .user_position_lower_limit = mechanical_calibration.snapshot().limits_valid
                ? mechanical_calibration.snapshot().lower_limit : NAN,
            .user_position_upper_limit = mechanical_calibration.snapshot().limits_valid
                ? mechanical_calibration.snapshot().upper_limit : NAN,
            .user_angle_offset = profile.position_offset,
            .user_angle_direction = profile.position_direction
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
        angle_encoder
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
    motor->set_state(false);

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
#ifdef STACK_PROFILE
    mark_stack();
#endif
//#pragma region StartupConfiguration
    start_timers();
    eeprom.wait_until_available();
    constexpr auto& profile = ruka3::joint_profile(RUKA3_JOINT_INDEX);
    mechanical_calibration.load(profile.output_encoder_zero, profile.position_lower_limit, profile.position_upper_limit);
    output_observer.set_encoder_zero(mechanical_calibration.snapshot().encoder_zero);
    // Bring up the fixed FDCAN transport before the motor stack. This is the
    // same ordering as the communication bootstrap, so motor initialization
    // cannot alter or delay CAN startup.
    start_cyphal();

    setup_cordic();
    create_motor();
    motor->start();
    apply_calibration();
    motor->set_foc_point(FOCTarget{0});
    // Commissioned calibration makes the joint ready, but it is never armed
    // merely by booting.  A controller must explicitly write arm=true.
    motor->set_state(false);
    command_supervisor.initialize(millis_32(),
                                  ruka3::joint_profile(RUKA3_JOINT_INDEX).safety.command_timeout_ms);
    fault_manager.initialize();

    // Warm up
    for (uint8_t i = 0; i < 32; ++i) {
        motor->update();
    }

    set_cyphal_mode(uavcan_node_Mode_1_0_OPERATIONAL);

    // Lock heap, no dynamic memory is used at runtime
    global_allocation_lock = true;
//#pragma endregion

    HAL_TIM_Base_Start_IT(&htim4);

    #ifdef STACK_PROFILE
    static millis stack_measurement_time = 0;
    #endif

    while(true) {
        output_observer.update(millis_32(), motor->get_shaft_encoder_value(), motor->get_angle());
        update_service_motion();
        can_fault_monitor.poll();
        if (command_supervisor.update(millis_32())) {
            // A command source has to explicitly re-enable the bridge after a
            // supervision stop; stale torque/position commands are never reused.
            motor->set_foc_point(FOCTarget{0});
            set_motor_enabled(false);
        }
        update_fault_manager();
        cyphal_loop();
        reboot_to_bootloader_if_requested();

        millis current_time = millis_32();
        monitor_loop(current_time);
        #ifdef STACK_PROFILE
        EACH_N(current_time, stack_measurement_time, 100, {
            measure_stack_usage();
        })
        #endif

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

    static millis report_time = 0;
    EACH_N(current_t, report_time, 1, {
        FOCState state_msg = {};

        state_msg.timestamp.microsecond = system_time();

        state_msg.angle.radian = motor->get_angle();
        state_msg.velocity.radian_per_second = motor->get_velocity();
        state_msg._torque.newton_meter = motor->get_torque();

        state_msg.current.ampere = motor->get_working_current();
        state_msg.bus_voltage.volt = motor_inverter.get_busV();

        constexpr float KELVIN_OFFSET = 273.15f;
        motor_inverter.update_temperature();
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
        bool is_valid = motor->set_foc_point(FOCTarget {
            .torque = msg._torq.newton_meter,
            .angle = msg.pos.radian,
            .velocity = msg.vel.radian_per_second,
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
                is_valid = motor->set_velocity_point(msg.set_point_value);
                break;
            case voltbro_foc_specific_control_1_0_TORQUE:
                is_valid = motor->set_torque_point(msg.set_point_value);
                break;
            case voltbro_foc_specific_control_1_0_POSITION:
                is_valid = motor->set_angle_point(msg.set_point_value);
                break;
            case voltbro_foc_specific_control_1_0_VOLTAGE:
                is_valid = is_voltage_target_valid(msg.set_point_value) &&
                    motor->set_voltage_point(msg.set_point_value);
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

    // Keep RUKA2's service convention: a read of zero_set is an explicit
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
