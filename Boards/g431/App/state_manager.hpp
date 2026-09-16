#pragma once

#include <cstddef>
#include <cstdint>

#include <voltbro/motors/bldc/vbdrive/vbdrive.hpp>

// The old mutable VBDriveConfig occupied EEPROM bytes [0, 73]. It is no
// longer read or written, but calibration stays at its commissioned address
// so existing J5/J6 calibration records remain valid.
constexpr size_t LEGACY_CONFIG_RESERVED_SIZE = 74;
constexpr size_t CALIBRATION_PLACEMENT = LEGACY_CONFIG_RESERVED_SIZE + 1;
constexpr size_t IND_SENSOR_STATE_PLACEMENT = CALIBRATION_PLACEMENT + sizeof(CalibrationData) + 1;

// The retired VBDrive configuration area is deliberately reused only below
// byte 64.  Bytes [64, 74] remain reserved, and the commissioned FOC record
// still starts at byte 75.
constexpr std::uint16_t MECHANICAL_CALIBRATION_SLOT_SIZE = 32U;
constexpr std::uint16_t MECHANICAL_CALIBRATION_SLOT_A = 0U;
constexpr std::uint16_t MECHANICAL_CALIBRATION_SLOT_B = MECHANICAL_CALIBRATION_SLOT_A + MECHANICAL_CALIBRATION_SLOT_SIZE;
static_assert(MECHANICAL_CALIBRATION_SLOT_B + MECHANICAL_CALIBRATION_SLOT_SIZE <= CALIBRATION_PLACEMENT);

void reboot_to_bootloader();
