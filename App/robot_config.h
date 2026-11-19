#pragma once

// C-compatible board timing constants consumed by CubeMX TIM initialization.
#define RUKA3_G474_PWM_PRESCALER 4U
#define RUKA3_G474_PWM_TICKS 2000U
#define RUKA3_G474_MAX_PWM_TICKS 1700U
#define RUKA3_G474_CURRENT_SAMPLE_TICKS 1800U

#ifdef __cplusplus
#include <cstdint>
#include "phase_current_sense.hpp"

namespace ruka3 {

enum class Board : std::uint8_t { g474, g431 };

#ifndef RUKA3_JOINT_INDEX
#error "RUKA3_JOINT_INDEX must be selected by the firmware build"
#endif

// The single compile-time identity of the firmware image being built.  CMake
// supplies it from the selected joint task; all board code should use this
// name rather than carry a second local joint number.
inline constexpr std::uint8_t kFlashedJointIndex =
    static_cast<std::uint8_t>(RUKA3_JOINT_INDEX);

struct MotorProfile {
    float control_period_s;
    float torque_const;
    float current_limit;
    float nominal_current;
    float torque_limit;
    float max_voltage;
    float stall_current;
    float stall_timeout_s;
    float stall_tolerance;
    float calibration_voltage;
    float current_pid_kp;
    float current_pid_ki;
    float current_pid_kd;
    float filter_expected_a;
    float filter_g1;
    float filter_g2;
    float filter_g3;
    float current_lpf;
    std::uint8_t pole_pairs;
};

struct TemperatureLimits {
    // Zero keeps the corresponding policy disabled until a measured limit is
    // commissioned for this joint.
    float mcu_warning_c;
    float mcu_fault_c;
    float stator_warning_c;
    float stator_fault_c;
};

struct SafetyProfile {
    std::uint32_t command_timeout_ms;
    TemperatureLimits temperature;
};

struct CommandTuning {
    float default_position_kp;
    float default_position_ki = 0.0F; // A / (rad*s), slow load compensation.
    float default_integral_current_limit_a = 3.0F;
    float default_velocity_kp;
    float default_torque_kp;
    float acquire_threshold_rad;
    float acquire_voltage;
    float track_max_voltage;
    float max_velocity_feedback_gain;
    float register_position_kp;
    float register_position_max_voltage;
    std::int32_t register_position_deadband_ticks;
    float service_position_velocity_rad_s;
    float service_position_acceleration_rad_s2;
    std::uint16_t service_position_update_hz;
    float calibration_current_limit_a = 0.4F;
    float calibration_velocity_gain_a = 0.8F;
};

struct ObserverProfile {
    std::uint32_t output_encoder_cpr;
    float output_velocity_lpf;
    std::uint8_t fusion_encoder_corridor_ticks;
    // Measured output-side free play.  It is diagnostic information, not a
    // coordinate correction.
    float expected_backlash_rad;
    float fusion_encoder_weight = 1.0F; // Absolute correction relative to the motor anchor, not per-tick alpha.
};

struct CommunicationProfile {
    std::uint16_t feedback_update_hz;
    std::uint16_t temperature_update_hz;
};

struct JointProfile {
    std::uint8_t index;
    Board board;
    std::uint8_t node_id;
    std::uint16_t feedback_subject_id;
    std::uint16_t servo_subject_id;
    std::uint16_t velocity_subject_id;
    float gear_ratio;
    // The first three signs describe the native mechanical coordinate.  The
    // final sign converts it to the public coordinate of the robot joint.
    std::int8_t motor_encoder_sign;   // Raw motor encoder increment to positive native motor angle.
    std::int8_t motor_command_sign;   // Raw voltage increment to positive native output motion.
    std::int8_t output_encoder_sign;  // Raw output encoder increment to positive native output angle.
    std::uint16_t motor_electrical_offset;
    std::uint16_t output_encoder_zero;
    std::int8_t dh_rotation;
    float speed_limit;
    float position_offset;
    std::int8_t position_direction;   // Native angle/velocity/torque to the public joint coordinate.
    MotorProfile motor;
    SafetyProfile safety;
    CommandTuning command;
    ObserverProfile observer;
    CommunicationProfile communication;
};

inline constexpr SafetyProfile kDefaultSafetyProfile{
    .command_timeout_ms = 1000U,
    .temperature = {0.0F, 0.0F, 0.0F, 0.0F},
};

inline constexpr CommandTuning kDefaultCommandTuning{
    .default_position_kp = 5.0F,
    .default_velocity_kp = 0.0F,
    .default_torque_kp = 0.5F,
    .acquire_threshold_rad = 0.1F,
    .acquire_voltage = 0.6F,
    .track_max_voltage = 11.5F,
    .max_velocity_feedback_gain = 1.5F,
    .register_position_kp = 0.002F,
    .register_position_max_voltage = 1.2F,
    .register_position_deadband_ticks = 8,
    .service_position_velocity_rad_s = 0.5F,
    .service_position_acceleration_rad_s2 = 1.0F,
    .service_position_update_hz = 100U,
    .calibration_current_limit_a = 1.0F,
    .calibration_velocity_gain_a = 2.0F,
};
inline constexpr CommandTuning kG474CommandTuning = [] {
    auto tuning = kDefaultCommandTuning;
    tuning.default_position_kp = 20.0F;
    tuning.default_position_ki = 2.0F;
    tuning.default_integral_current_limit_a = 1.0F;
    return tuning;
}();
// J4 uses the G474 position gains.

inline constexpr CommandTuning kJ4CommandTuning = [] {
    auto tuning = kG474CommandTuning;
    tuning.default_integral_current_limit_a = 1.0F;
    return tuning;
}();
// Common public Kp/Kd for both boards; adapters convert them to motor-side
// gains by dividing by gear_ratio. G431 keeps its own calibration-current limits.
inline constexpr CommandTuning kG431CommandTuning = [] {
    auto tuning = kDefaultCommandTuning;
    tuning.default_integral_current_limit_a = 2.0F;
    tuning.calibration_current_limit_a = 0.4F;
    tuning.calibration_velocity_gain_a = 0.8F;
    return tuning;
}();
inline constexpr ObserverProfile kDefaultObserverProfile{65536U, 0.1F, 2U, 0.0F};
inline constexpr ObserverProfile kG474ObserverProfile{16384U, 0.1F, 2U, 0.0F};
// J1 unloaded transmission must not be driven primarily by its backlash-side encoder.
inline constexpr ObserverProfile kJ1ObserverProfile = [] {
    auto observer = kG474ObserverProfile;
    observer.fusion_encoder_weight = 0.1F;
    return observer;
}();
inline constexpr ObserverProfile kJ4ObserverProfile = [] {
    auto observer = kG474ObserverProfile;
    observer.fusion_encoder_weight = 0.1F;
    return observer;
}();
inline constexpr float kCalibrationMaxVelocityRadS = 0.5F;
inline constexpr float kCalibrationCurrentSlewAS = 1.0F;
inline constexpr float kCalibrationSafeMarginRad = 0.08F;
constexpr std::uint16_t calibration_margin_ticks(std::uint32_t cpr) {
    return static_cast<std::uint16_t>(cpr * kCalibrationSafeMarginRad / 6.28318530718F);
}
inline constexpr std::uint32_t kCalibrationReferenceTicksPerTurn = 65536U;
inline constexpr CommunicationProfile kDefaultCommunicationProfile{
    .feedback_update_hz = 100U,
    .temperature_update_hz = 10U,
};
// J6: measured around the zero pose by rocking the motor with Kp=30.  The
// output encoder stayed still while the geared motor estimate traversed about
// 0.006 rad, then the output began to move.
inline constexpr ObserverProfile kJ6ObserverProfile{65536U, 0.1F, 2U, 0.006F};

inline constexpr MotorProfile kG474MotorProfile{
    // TIM1: 160 MHz / ((PSC=4)+1) / ((ARR=1999)+1) = 16 kHz.
    .control_period_s = 0.0000625F, .torque_const = 0.011F,
    // User-selected provisional limits for J1-J4; not verified nameplate ratings.
    .current_limit = 22.0F, .nominal_current = 10.0F, .torque_limit = 10.0F, .max_voltage = 50.0F,
    .stall_current = 10.0F, .stall_timeout_s = 3.0F, .stall_tolerance = 0.2F,
    .calibration_voltage = 1.0F, .current_pid_kp = 0.035F, .current_pid_ki = 75.0F,
    .current_pid_kd = 0.0F, .filter_expected_a = 0.0F,
    .filter_g1 = 0.015700989410003974F, .filter_g2 = 3.925227776360174F,
    .filter_g3 = 387.54711795263574F, .current_lpf = 0.0925F,
    .pole_pairs = 14,
};

// ADC now acquires one externally triggered four-rank scan per PWM period.
// Preserve the original 45 mV/A equivalent sensor sensitivity.
inline constexpr PhaseCurrentSense kG474CurrentSense{
    3.3F, 4096.0F, 1.0F, 0.045F, 16.0F, {1, 1, 1}, 64U,
};
// At this vector/bus ratio, SVM's highest duty is <= 0.8465, leaving all
// three PWM phases low before the sample trigger at 90% of the period.
inline constexpr float kG474ModulationVoltageRatio = 0.4F;
// Active current-loop ceiling follows the selected nominal motor current.
// These limits apply to ALL command paths, including normal CAN servo commands.
inline constexpr float kG474CommissioningCurrentLimitA = kG474MotorProfile.nominal_current;
inline constexpr float kG474CommissioningVoltageLimitV = 2.0F;
inline constexpr float kG474PhaseOvercurrentTripA = kG474MotorProfile.current_limit;
static_assert(kG474CommissioningCurrentLimitA > 0.0F &&
              kG474CommissioningCurrentLimitA <= kG474MotorProfile.current_limit);
static_assert(kG474CommissioningVoltageLimitV > 0.0F &&
              kG474CommissioningVoltageLimitV <= kG474MotorProfile.max_voltage);
static_assert(kG474PhaseOvercurrentTripA > kG474CommissioningCurrentLimitA &&
              kG474PhaseOvercurrentTripA <= kG474MotorProfile.current_limit);

// J5/J6: user-selected 6 A control ceiling, including stationary holding.
// torque_limit follows the existing Kt=1 scale so it does not impose a second
// 3 A ceiling at gear=10. These are controller settings, not rated motor data.
inline constexpr MotorProfile kG431MotorProfile{
    .control_period_s = 0.000025F, .torque_const = 1.0F,
    .current_limit = 6.0F, .nominal_current = 6.0F, .torque_limit = 60.0F, .max_voltage = 50.0F,
    .stall_current = 6.0F, .stall_timeout_s = 3.0F, .stall_tolerance = 0.2F,
    .calibration_voltage = 0.2F, .current_pid_kp = 4.0F, .current_pid_ki = 1600.0F,
    .current_pid_kd = 0.0F, .filter_expected_a = 0.0F,
    .filter_g1 = 0.015700989410003974F, .filter_g2 = 3.925227776360174F,
    .filter_g3 = 387.54711795263574F, .current_lpf = 0.0925F,
    .pole_pairs = 14,
};

// This table is the only CAN identity authority for all six joints.
// Calibration offsets remain in the board-specific persistent record.
inline constexpr JointProfile kJointProfiles[] = {
    {1, Board::g474, 101, 1001, 1121, 1131, 72.0F, -1, -1, -1, 1205,  1206, -1, 0.0F, 0.0F,  1, kG474MotorProfile, kDefaultSafetyProfile, kG474CommandTuning, kJ1ObserverProfile, kDefaultCommunicationProfile},
    {2, Board::g474, 102, 1001, 1122, 1132, 36.0F, -1, -1,  1,  555, 11000, -1, 0.0F, 0.0F,  1, kG474MotorProfile, kDefaultSafetyProfile, kG474CommandTuning, kG474ObserverProfile, kDefaultCommunicationProfile},
    {3, Board::g474, 103, 1001, 1123, 1133, 36.0F, -1, -1,  1, 1320,  9956,  1, 0.0F, 0.0F,  1, kG474MotorProfile, kDefaultSafetyProfile, kG474CommandTuning, kG474ObserverProfile, kDefaultCommunicationProfile},
    {4, Board::g474, 104, 1001, 1124, 1134, 36.0F, -1,  1, -1,16179,  5000,  1, 0.0F, 0.0F,  1, kG474MotorProfile, kDefaultSafetyProfile, kJ4CommandTuning, kJ4ObserverProfile, kDefaultCommunicationProfile},
    {5, Board::g431, 105, 1001, 1125, 1135, 10.0F,  1, -1,  1,    0,     0, -1, 0.0F, 0.0F, -1, kG431MotorProfile, kDefaultSafetyProfile, kG431CommandTuning, kDefaultObserverProfile, kDefaultCommunicationProfile},
    {6, Board::g431, 106, 1001, 1126, 1136, 10.0F,  1,  1,  1,    0, 52076, -1, 0.0F, 0.0F,  1, kG431MotorProfile, kDefaultSafetyProfile, kG431CommandTuning, kJ6ObserverProfile, kDefaultCommunicationProfile},
};

constexpr bool motor_profile_is_valid(const MotorProfile& profile) {
    return profile.control_period_s > 0.0F &&
           profile.torque_const > 0.0F &&
           profile.current_limit > 0.0F &&
           profile.nominal_current > 0.0F && profile.nominal_current <= profile.current_limit &&
           profile.torque_limit > 0.0F &&
           profile.max_voltage > 0.0F &&
           profile.stall_current > 0.0F &&
           profile.stall_timeout_s >= 0.0F &&
           profile.pole_pairs > 0U;
}

constexpr bool joint_profiles_are_valid() {
    for (const auto& profile : kJointProfiles) {
        if (profile.index == 0U || profile.node_id == 0U || profile.gear_ratio <= 0.0F ||
            profile.speed_limit < 0.0F ||
            !(profile.observer.fusion_encoder_weight >= 0 && profile.observer.fusion_encoder_weight <= 1) ||
            profile.communication.feedback_update_hz == 0U ||
            profile.communication.feedback_update_hz > 1000U ||
            profile.communication.temperature_update_hz == 0U ||
            profile.communication.temperature_update_hz > 1000U ||
            !(profile.command.calibration_current_limit_a > 0) ||
            profile.command.calibration_current_limit_a > profile.motor.nominal_current ||
            !(profile.command.calibration_velocity_gain_a > 0) ||
            !(profile.command.default_position_ki >= 0) ||
            !(profile.command.default_integral_current_limit_a >= 0) ||
            profile.command.default_integral_current_limit_a > profile.motor.stall_current ||
            profile.command.default_integral_current_limit_a > profile.motor.nominal_current ||
            profile.command.service_position_velocity_rad_s <= 0 ||
            profile.command.service_position_acceleration_rad_s2 <= 0 ||
            profile.command.service_position_update_hz == 0 ||
            profile.command.service_position_update_hz > 1000U ||
            (profile.position_direction != -1 && profile.position_direction != 1) ||
            !motor_profile_is_valid(profile.motor)) {
            return false;
        }
    }
    return true;
}

static_assert(joint_profiles_are_valid());

constexpr const JointProfile& joint_profile(const std::uint8_t index) {
    return kJointProfiles[index - 1U];
}

}  // namespace ruka3
#endif // __cplusplus
