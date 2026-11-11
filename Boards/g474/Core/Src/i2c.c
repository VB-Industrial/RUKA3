#include "i2c.h"

I2C_HandleTypeDef hi2c4;

// Same pins/clock/timing as RUKA2. EEPROM uses bounded polling only: ADC owns
// DMA1_Channel1, so the RUKA2 I2C DMA setup must not be copied here.
void MX_I2C4_Init(void) {
    hi2c4.Instance = I2C4;
    hi2c4.Init.Timing = 0x00702991;
    hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c4) != HAL_OK ||
        HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK ||
        HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK) Error_Handler();
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* handle) {
    if (handle->Instance != I2C4) return;
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C4_CONFIG(RCC_I2C4CLKSOURCE_PCLK1);
    __HAL_RCC_I2C4_CLK_ENABLE();
    GPIO_InitTypeDef pin = {0};
    pin.Pin = GPIO_PIN_6;
    pin.Mode = GPIO_MODE_AF_OD;
    pin.Pull = GPIO_NOPULL;
    pin.Speed = GPIO_SPEED_FREQ_LOW;
    pin.Alternate = GPIO_AF8_I2C4;
    HAL_GPIO_Init(GPIOC, &pin);
    pin.Pin = GPIO_PIN_7;
    pin.Alternate = GPIO_AF3_I2C4;
    HAL_GPIO_Init(GPIOB, &pin);
}
