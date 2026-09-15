#pragma once

#include "../../../App/robot_config.h"

#ifndef JOINT_N
#error "JOINT_N must be provided by the RUKA3_JOINT_INDEX CMake setting"
#endif

static_assert(JOINT_N >= 1 && JOINT_N <= 4);
inline constexpr auto& active_joint_profile = ruka3::joint_profile(JOINT_N);
static_assert(active_joint_profile.board == ruka3::Board::g474);

#define AGENT_JS_SUB_PORT 1001
#define JS_SUB_PORT_ID (active_joint_profile.servo_subject_id)

constexpr bool js_control_enabled = true;
constexpr int8_t start_angle_according_to_DH = 0;
constexpr int8_t DH_rotation = active_joint_profile.dh_rotation;
constexpr uint8_t gear_ratio = static_cast<uint8_t>(active_joint_profile.gear_ratio);
constexpr encoder_data electric_offset = static_cast<encoder_data>(active_joint_profile.motor_electrical_offset);
constexpr encoder_data gear_encoder_zero = static_cast<encoder_data>(active_joint_profile.output_encoder_zero);
constexpr int8_t motor_encoder_sign = active_joint_profile.motor_encoder_sign;
constexpr int8_t motor_command_sign = active_joint_profile.motor_command_sign;
constexpr int8_t gear_encoder_sign = active_joint_profile.output_encoder_sign;
