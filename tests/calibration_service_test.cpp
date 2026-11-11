#include <array>
#include <cstdint>
#define private public
#include "encoder_calibration.hpp"
#undef private
#include "calibration_service.hpp"
#include "robot_config.h"
#include <cassert>
#include <cmath>
static ruka3::JointCalibration cal(4,16384,1,65536,208);
static float speed, current;
static bool enabled;
ruka3::JointCalibration& joint_calibration_instance() { return cal; }
std::uint16_t calibration_board_raw() { return 500; }
float calibration_board_angle() { return 0; }
float calibration_board_velocity() { return speed; }
bool calibration_board_available() { return true; }
bool calibration_board_enable() { enabled=true; return true; }
void calibration_board_disable() { enabled=false; }
void calibration_board_current(float value) { current=value; }
void calibration_board_sync() {}
extern "C" bool encoder_calibration_storage_load(uint8_t,encoder_calibration_data*) { return false; }
extern "C" bool encoder_calibration_storage_save(uint8_t,encoder_calibration_data*) { return true; }
int main() {
 assert(calibration_manual_command(1,0));
 assert(calibration_manual_move(1000,0)); assert(enabled);
 calibration_service_update(1); assert(current > 0);
 calibration_service_update(1001); assert(current == 0); // Jog lease expires; zero-speed braking remains.
 calibration_cancel();
 assert(calibration_manual_command(1,0));
 assert(calibration_manual_move(1000,0));
 auto& core=cal.core();
 core.state_=EncoderCalibrationState::SettleAtB;
 core.data_.manual_span_ticks=1000;
 core.previous_raw_=500;
 core.position_ticks_=500;
 speed=.2F;
 constexpr auto& tuning=ruka3::joint_profile(RUKA3_JOINT_INDEX).command;
 for (unsigned now=1; now<=600; ++now) { core.state_started_ms_=now; calibration_service_update(now); }
 assert(std::fabs(calibration_current_target()-current)<.00001F);
 assert(std::fabs(current+tuning.calibration_velocity_gain_a*.2F)<.00001F); // Zero velocity must brake, not freewheel.
 speed=-100;
 calibration_service_update(601);
 assert(std::fabs(current-tuning.calibration_current_limit_a)<.00001F); // Oppose overspeed immediately.
 for (unsigned now=602; now<2200; ++now) { core.state_started_ms_=now; calibration_service_update(now); }
 assert(std::fabs(current-tuning.calibration_current_limit_a)<.00001F);
 speed=0;
 calibration_service_update(2200); assert(current==0); // Stop driving immediately.
 calibration_cancel();
 assert(calibration_manual_command(1,3000));
 assert(calibration_manual_move(5000,3000));
 calibration_service_update(3001); assert(current>0 && current<=.02001F); // Motoring starts softly.
 for(unsigned now=3002;now<3250;++now) calibration_service_update(now);
 assert(current>.1F);
 speed=1;
 calibration_service_update(3250); assert(current<0); // A freed rotor brakes without a one-second torque tail.
 calibration_cancel(); assert(!enabled);
 assert(calibration_current_target()==0);
 assert(core.state()==EncoderCalibrationState::Aborted);
 calibration_service_update(2200); assert(!enabled);
}
