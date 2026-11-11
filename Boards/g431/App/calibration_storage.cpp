#include "i2c.h"
#include "calibration_eeprom_hal.hpp"

I2C_HandleTypeDef* calibration_eeprom_bus() { return &hi2c2; }
extern "C" void calibration_eeprom_initialize() {} // Initialized by board main.
