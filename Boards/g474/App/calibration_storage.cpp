#include "i2c.h"
#include "calibration_eeprom_hal.hpp"

I2C_HandleTypeDef* calibration_eeprom_bus() { return &hi2c4; }
extern "C" void calibration_eeprom_initialize() { MX_I2C4_Init(); }
