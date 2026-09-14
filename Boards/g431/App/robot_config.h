#pragma once

#include "../../../App/robot_config.h"

static_assert(RUKA3_JOINT_INDEX == 5 || RUKA3_JOINT_INDEX == 6,
              "STM32G431 is reserved for RUKA3 joints 5 and 6");
