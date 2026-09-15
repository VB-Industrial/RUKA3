#include "app.h"

void reboot_to_bootloader() {
    if (auto* motor = get_motor()) {
        motor->set_foc_point(FOCTarget{0});
        motor->stop();
    }

    constexpr uint32_t boot_request_magic = 0xB00710ADUL;
    (void)HAL_FDCAN_Stop(&hfdcan1);
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    TAMP->BKP0R = boot_request_magic;
    TAMP->BKP1R = 0U;
    TAMP->BKP2R = 0U;
    HAL_PWR_DisableBkUpAccess();
    HAL_Delay(50);
    NVIC_SystemReset();
}
