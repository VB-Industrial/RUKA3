#include "app.h"
#include "protocol.h"
#include "robot_config.h"

void reboot_to_bootloader() {
    if (auto* motor = get_motor()) {
        motor->set_foc_point(FOCTarget{0});
        // Keep STSPIN awake while rebooting: standby can disrupt CAN before
        // VBBoot takes over. TIM1 MOE gates off all bridge PWM outputs.
        (void)motor->set_bridge_outputs_enabled(false);
    }

    constexpr uint32_t boot_request_magic = 0xB00710ADUL;
    (void)HAL_FDCAN_Stop(&hfdcan1);
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    TAMP->BKP0R = boot_request_magic;
    TAMP->BKP1R = ruka3::protocol::boot_transport_1m4(
        ruka3::joint_profile(RUKA3_JOINT_INDEX).node_id);
    TAMP->BKP2R = 0U;
    HAL_PWR_DisableBkUpAccess();
    HAL_Delay(50);
    NVIC_SystemReset();
}
