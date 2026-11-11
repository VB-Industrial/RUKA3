#pragma once

#include "main.h"
#include "stm32g4xx_hal.h"

#include <cyphal/cyphal.h>
#include <voltbro/utils.hpp>
#include <voltbro/motors/bldc/vbdrive/vbdrive.hpp>

#include "state_manager.hpp"
#include "can_fault_monitor.hpp"
#include "command_supervisor.hpp"
#include "fault_manager.hpp"
#include "output_state_observer.hpp"
#include "mechanical_calibration.hpp"

#include "service_motion.hpp"

// state_manager.hpp
// CALIBRATION_PLACEMENT
// IND_SENSOR_STATE_PLACEMENT

// communications.cpp
inline constexpr size_t CYPHAL_QUEUE_SIZE = 50;

// NOTE: due to RAM constraints, this buffer is first used for calibration, then reused for Cyphal queue.
//       It should be big enough for both purposes. For calibration, it should be >=(2048+1)*4 for guaranteed alignment
constexpr size_t SHARED_BUFFER_SIZE = std::max(
    (CALIBRATION_BUFF_SIZE + 1) * sizeof(int),
    static_cast<size_t>(CYPHAL_QUEUE_SIZE * sizeof(CanardTxQueueItem) * QUEUE_SIZE_MULT)
);
extern std::byte cyphal_queue_buffer_shared[SHARED_BUFFER_SIZE];

inline constexpr millis DELAY_ON_ERROR_MS = 500;
std::shared_ptr<CyphalInterface> get_interface();
void in_loop_reporting(millis);
void setup_subscriptions();
void cyphal_loop();
void start_cyphal();
void monitor_loop(millis);
void set_cyphal_mode(uint8_t mode);
void set_cyphal_health(uint8_t health);

// common.cpp
micros system_time();
micros micros_64();
millis millis_32();
void start_timers();

// app.cpp
VBDrive* get_motor();
const ruka3::CanFaultSnapshot& get_can_fault_snapshot();
const ruka3::FaultSnapshot& get_fault_snapshot();
const ruka3::CommandSnapshot& get_command_snapshot();
uint32_t get_joint_state();
bool is_motor_enabled();
const ruka3::OutputObserverSnapshot& get_output_observer_snapshot();
const ruka3::MechanicalCalibrationSnapshot& get_mechanical_calibration_snapshot();
const ruka3::ServiceMotionSnapshot& get_service_motion_snapshot();
bool set_motor_enabled(bool enabled);
bool acknowledge_faults();
bool service_set_zero();
bool service_set_limit(uint32_t stage);
bool service_set_voltage(float voltage);
bool service_set_current(float current);
bool service_set_position(float position);
bool service_set_servo(float position, float velocity, float torque, float kp, float kd);
void service_hold();
void service_cancel();
EEPROM& get_eeprom();
bool is_motor_calibrated();
int32_t get_motor_calibration_offset();
bool is_motor_encoder_inverted();
bool is_motor_calibration_pending();
bool request_motor_calibration();
bool is_driver_awake();
bool set_driver_awake(bool awake);
float get_motor_electric_angle();
// actions (in app.cpp)
bool is_able_to_calibrate();
bool do_calibrate();

//fdcan.cpp
extern FDCAN_HandleTypeDef hfdcan1;

// state_manager.cpp
void reboot_to_bootloader();

//command_mode.cpp
