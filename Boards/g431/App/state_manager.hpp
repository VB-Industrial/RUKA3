#pragma once

#include <cstddef>

#include <voltbro/motors/bldc/vbdrive/vbdrive.hpp>

// The old mutable VBDriveConfig occupied EEPROM bytes [0, 73]. It is no
// longer read or written, but calibration stays at its commissioned address
// so existing J5/J6 calibration records remain valid.
constexpr size_t LEGACY_CONFIG_RESERVED_SIZE = 74;
constexpr size_t CALIBRATION_PLACEMENT = LEGACY_CONFIG_RESERVED_SIZE + 1;
constexpr size_t IND_SENSOR_STATE_PLACEMENT = CALIBRATION_PLACEMENT + sizeof(CalibrationData) + 1;

void reboot_to_bootloader();
