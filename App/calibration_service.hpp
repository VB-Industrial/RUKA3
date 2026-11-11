#pragma once
#include "joint_calibration.hpp"

// Board hooks keep STSPIN/DRV8328 startup and current FOC implementations local.
ruka3::JointCalibration& joint_calibration_instance();
std::uint16_t calibration_board_raw();
float calibration_board_angle();
float calibration_board_velocity();
bool calibration_board_available();
bool calibration_board_enable();
void calibration_board_disable();
void calibration_board_current(float public_current_a);
void calibration_board_sync();

bool calibration_manual_command(std::int32_t command, std::uint32_t now);
bool calibration_manual_move(std::int32_t reference_velocity, std::uint32_t now);
bool calibration_begin_backlash(std::uint32_t now);
void calibration_service_update(std::uint32_t now);
void calibration_cancel();

// Applied calibration request after the current clamp and slew limiter, public A.
float calibration_current_target();
