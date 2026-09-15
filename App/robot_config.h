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
    float speed_limit;
    float max_voltage;
    float position_offset;
    float position_lower_limit;
    float position_upper_limit;
    std::int8_t position_direction;
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
    MotorProfile motor;
};

inline constexpr MotorProfile kG474MotorProfile{
    .control_period_s = 0.00005F, .torque_const = 0.011F,
    .current_limit = 22.0F, .torque_limit = 10.0F, .speed_limit = 0.0F, .max_voltage = 50.0F,
    .position_offset = 0.0F, .position_lower_limit = 0.0F, .position_upper_limit = 0.0F, .position_direction = 1,
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
    .current_limit = 2.0F, .torque_limit = 30.0F, .speed_limit = 0.0F, .max_voltage = 50.0F,
    .position_offset = 0.0F, .position_lower_limit = 0.0F, .position_upper_limit = 0.0F, .position_direction = 1,
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
    {1, Board::g474, 101, 1001, 1121, 1131, 72.0F, -1, -1, -1, 1205,  1206, -1, kG474MotorProfile},
    {2, Board::g474, 102, 1001, 1122, 1132, 36.0F, -1, -1,  1,  555, 11000, -1, kG474MotorProfile},
    {3, Board::g474, 103, 1001, 1123, 1133, 36.0F, -1, -1,  1, 1320,  9956,  1, kG474MotorProfile},
    {4, Board::g474, 104, 1001, 1124, 1134, 36.0F, -1,  1, -1,16179,  5000,  1, kG474MotorProfile},
    {5, Board::g431, 105, 1001, 1125, 1135, 36.0F,  1, -1,  1,    0,     0, -1, kG431MotorProfile},
    {6, Board::g431, 106, 1001, 1126, 1136, 36.0F,  1, -1,  1,    0,     0, -1, kG431MotorProfile},
};

constexpr const JointProfile& joint_profile(const std::uint8_t index) {
    return kJointProfiles[index - 1U];
}

}  // namespace ruka3
