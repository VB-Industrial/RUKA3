#pragma once
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
void calibration_eeprom_initialize(void);
uint8_t at24_isConnected(void);
uint8_t at24_read(uint16_t address, uint8_t* data, uint8_t size, uint32_t timeout_ms);
uint8_t at24_write(uint16_t address, uint8_t* data, uint8_t size, uint32_t timeout_ms);
#ifdef __cplusplus
}
#endif
