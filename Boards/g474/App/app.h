#pragma once

#include "main.h"

#include <cyphal/cyphal.h>

#include "voltbro/utils.hpp"
#include "voltbro/motors/bldc/foc/foc.hpp"

#include "joint_config.h"
#include "robot_config.h"


micros micros_64();

void setup_cordic();
void setup_timers();

void setup_can();
void setup_cyphal_comms();
void heartbeat();
std::shared_ptr<CyphalInterface>& get_interface();

void comms_loop(millis);
FOC& get_motor();
GenericEncoder& get_gear_encoder();
