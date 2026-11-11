#pragma once
#include <algorithm>
#include "stm32g4xx_hal.h"
#include "calibration_eeprom.h"

// One board-owned bus; no heap, retries/fallback buses, or unbounded waits.
I2C_HandleTypeDef* calibration_eeprom_bus();
extern "C" uint8_t at24_isConnected() {
    return HAL_I2C_IsDeviceReady(calibration_eeprom_bus(), 0xA0, 1, 10) == HAL_OK;
}
extern "C" uint8_t at24_read(uint16_t address, uint8_t* bytes, uint8_t size, uint32_t timeout) {
    if (static_cast<unsigned>(address) + size > 16384) return 0;
    return HAL_I2C_Mem_Read(calibration_eeprom_bus(), 0xA0, address, I2C_MEMADD_SIZE_16BIT, bytes, size, timeout) == HAL_OK;
}
extern "C" uint8_t at24_write(uint16_t address, uint8_t* bytes, uint8_t size, uint32_t timeout) {
    if (static_cast<unsigned>(address) + size > 16384) return 0;
    unsigned done = 0;
    while (done < size) {
        const auto chunk = std::min<unsigned>(size - done, 64U - ((address + done) % 64U));
        if (HAL_I2C_Mem_Write(calibration_eeprom_bus(), 0xA0, address + done, I2C_MEMADD_SIZE_16BIT,
                bytes + done, chunk, timeout) != HAL_OK) return 0;
        const auto started = HAL_GetTick();
        while (HAL_I2C_IsDeviceReady(calibration_eeprom_bus(), 0xA0, 1, 5) != HAL_OK) {
            if (HAL_GetTick() - started >= timeout) return 0;
        }
        done += chunk;
    }
    return 1;
}
