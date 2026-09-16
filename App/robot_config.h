#pragma once

#include <cstdint>

namespace ruka3 {

enum class Board : std::uint8_t { g474, g431 };
enum class PositionSource : std::uint8_t { rotor, shaft };

struct MotorProfile {
    float control_period_s;
    float torque_const;
    float current_limit;
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
    PositionSource position_source;
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
    float default_velocity_kp;
    float default_torque_kp;
    float acquire_threshold_rad;
    float acquire_voltage;
    float track_max_voltage;
    float max_velocity_feedback_gain;
    float register_position_kp;
    float register_position_max_voltage;
    std::int32_t register_position_deadband_ticks;
};

struct ObserverProfile {
    std::uint32_t output_encoder_cpr;
    float output_velocity_lpf;
    std::uint8_t fusion_encoder_corridor_ticks;
};

struct JointProfile {
    std::uint8_t index;
    Board board;
    std::uint8_t node_id;
    std::uint16_t feedback_subject_id;
    std::uint16_t servo_subject_id;
    std::uint16_t velocity_subject_id;
    float gear_ratio;
    std::int8_t motor_encoder_sign;
    std::int8_t motor_command_sign;
    std::int8_t output_encoder_sign;
    std::uint16_t motor_electrical_offset;
    std::uint16_t output_encoder_zero;
    std::int8_t dh_rotation;
    float speed_limit;
    float position_offset;
    float position_lower_limit;
    float position_upper_limit;
    std::int8_t position_direction;
    MotorProfile motor;
    SafetyProfile safety;
    CommandTuning command;
    ObserverProfile observer;
};

inline constexpr SafetyProfile kDefaultSafetyProfile{
    .command_timeout_ms = 1000U,
    .temperature = {0.0F, 0.0F, 0.0F, 0.0F},
};

inline constexpr CommandTuning kDefaultCommandTuning{
    .default_position_kp = 0.5F,
    .default_velocity_kp = 0.02F,
    .default_torque_kp = 0.5F,
    .acquire_threshold_rad = 0.5F,
    .acquire_voltage = 0.6F,
    .track_max_voltage = 11.5F,
    .max_velocity_feedback_gain = 1.5F,
    .register_position_kp = 0.002F,
    .register_position_max_voltage = 1.2F,
    .register_position_deadband_ticks = 8,
};
inline constexpr ObserverProfile kDefaultObserverProfile{65536U, 0.1F, 2U};

inline constexpr MotorProfile kG474MotorProfile{
    .control_period_s = 0.00005F, .torque_const = 0.011F,
    .current_limit = 22.0F, .torque_limit = 10.0F, .max_voltage = 50.0F,
    .stall_current = 22.0F, .stall_timeout_s = 3.0F, .stall_tolerance = 0.2F,
    .calibration_voltage = 1.0F, .current_pid_kp = 2.0F, .current_pid_ki = 600.0F,
    .current_pid_kd = 0.0F, .filter_expected_a = 0.0F,
    .filter_g1 = 0.015700989410003974F, .filter_g2 = 3.925227776360174F,
    .filter_g3 = 387.54711795263574F, .current_lpf = 0.0925F,
    .pole_pairs = 14, .position_source = PositionSource::rotor,
};

// J5/J6 preserve the presently commissioned 2 A limit.  The remaining values
// are the prior VBDrive defaults, now fixed in the robot profile rather than
// read from EEPROM.
inline constexpr MotorProfile kG431MotorProfile{
    .control_period_s = 0.000025F, .torque_const = 1.0F,
    .current_limit = 2.0F, .torque_limit = 30.0F, .max_voltage = 50.0F,
    .stall_current = 6.0F, .stall_timeout_s = 3.0F, .stall_tolerance = 0.2F,
    .calibration_voltage = 0.2F, .current_pid_kp = 4.0F, .current_pid_ki = 1600.0F,
    .current_pid_kd = 0.0F, .filter_expected_a = 0.0F,
    .filter_g1 = 0.015700989410003974F, .filter_g2 = 3.925227776360174F,
    .filter_g3 = 387.54711795263574F, .current_lpf = 0.0925F,
    .pole_pairs = 14, .position_source = PositionSource::rotor,
};

// This table is the only CAN identity authority for all six joints.
// Calibration offsets remain in the board-specific persistent record.
inline constexpr JointProfile kJointProfiles[] = {
    {1, Board::g474, 101, 1001, 1121, 1131, 72.0F, -1, -1, -1, 1205,  1206, -1, 0.0F, 0.0F, 0.0F, 0.0F,  1, kG474MotorProfile, kDefaultSafetyProfile, kDefaultCommandTuning, kDefaultObserverProfile},
    {2, Board::g474, 102, 1001, 1122, 1132, 36.0F, -1, -1,  1,  555, 11000, -1, 0.0F, 0.0F, 0.0F, 0.0F,  1, kG474MotorProfile, kDefaultSafetyProfile, kDefaultCommandTuning, kDefaultObserverProfile},
    {3, Board::g474, 103, 1001, 1123, 1133, 36.0F, -1, -1,  1, 1320,  9956,  1, 0.0F, 0.0F, 0.0F, 0.0F,  1, kG474MotorProfile, kDefaultSafetyProfile, kDefaultCommandTuning, kDefaultObserverProfile},
    {4, Board::g474, 104, 1001, 1124, 1134, 36.0F, -1,  1, -1,16179,  5000,  1, 0.0F, 0.0F, 0.0F, 0.0F,  1, kG474MotorProfile, kDefaultSafetyProfile, kDefaultCommandTuning, kDefaultObserverProfile},
    {5, Board::g431, 105, 1001, 1125, 1135, 10.0F,  1, -1,  1,    0,     0, -1, 0.0F, 0.0F, 0.0F, 0.0F,  1, kG431MotorProfile, kDefaultSafetyProfile, kDefaultCommandTuning, kDefaultObserverProfile},
    {6, Board::g431, 106, 1001, 1126, 1136, 10.0F,  1, -1,  1,    0, 52076, -1, 0.0F, 0.0F, -3.12413936F, 3.12413936F, 1, kG431MotorProfile, kDefaultSafetyProfile, kDefaultCommandTuning, kDefaultObserverProfile},
};

constexpr bool motor_profile_is_valid(const MotorProfile& profile) {
    return profile.control_period_s > 0.0F &&
           profile.torque_const > 0.0F &&
           profile.current_limit > 0.0F &&
           profile.torque_limit > 0.0F &&
           profile.max_voltage > 0.0F &&
           profile.stall_current > 0.0F &&
           profile.stall_timeout_s >= 0.0F &&
           profile.pole_pairs > 0U;
}

constexpr bool joint_profiles_are_valid() {
    for (const auto& profile : kJointProfiles) {
        const bool position_limits_disabled =
            profile.position_lower_limit == 0.0F && profile.position_upper_limit == 0.0F;
        const bool position_limits_valid = position_limits_disabled ||
            profile.position_lower_limit < profile.position_upper_limit;
        if (profile.index == 0U || profile.node_id == 0U || profile.gear_ratio <= 0.0F ||
            profile.speed_limit < 0.0F ||
            (profile.position_direction != -1 && profile.position_direction != 1) ||
            !position_limits_valid ||
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
