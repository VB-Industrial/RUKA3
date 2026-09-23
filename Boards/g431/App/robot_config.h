#pragma once

#include "../../../App/robot_config.h"

static_assert(ruka3::kFlashedJointIndex == 5 || ruka3::kFlashedJointIndex == 6,
              "STM32G431 is reserved for RUKA3 joints 5 and 6");
