#pragma once

#include <cstdint>

namespace ruka3 {

enum class Board : std::uint8_t { g474, g431 };

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
};

// This table is the only CAN identity authority for all six joints.
// Calibration offsets remain in the board-specific persistent record.
inline constexpr JointProfile kJointProfiles[] = {
    {1, Board::g474, 101, 1001, 1121, 1131, 72.0F, -1, -1, -1},
    {2, Board::g474, 102, 1001, 1122, 1132, 36.0F, -1, -1,  1},
    {3, Board::g474, 103, 1001, 1123, 1133, 36.0F, -1, -1,  1},
    {4, Board::g474, 104, 1001, 1124, 1134, 36.0F, -1,  1, -1},
    {5, Board::g431, 105, 1001, 1125, 1135, 36.0F,  1, -1,  1},
    {6, Board::g431, 106, 1001, 1126, 1136, 36.0F,  1, -1,  1},
};

constexpr const JointProfile& joint_profile(const std::uint8_t index) {
    return kJointProfiles[index - 1U];
}

}  // namespace ruka3
