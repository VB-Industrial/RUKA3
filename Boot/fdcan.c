/* RUKA3 VBBoot integration. Both board boot clocks feeding FDCAN are 160 MHz:
   G431: PCLK1 (APB1 /1); G474: PLLQ (independent of APB1 /2).
   This initializer replaces VBBoot's mutable/legacy 1M/8M initializer. */
#include "app.h"

void configure_fdcan(FDCAN_HandleTypeDef* hfdcan, const BootTransportConfig* config) {
    hfdcan->Instance = FDCAN1;
    hfdcan->Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan->Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    hfdcan->Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan->Init.AutoRetransmission = ENABLE;
    hfdcan->Init.TransmitPause = ENABLE;
    hfdcan->Init.ProtocolException = DISABLE;
    /* 160 MHz / (4 * 40) = 1 Mbit/s, sample point 75%, SJW 25 ns. */
    hfdcan->Init.NominalPrescaler = 4U;
    hfdcan->Init.NominalSyncJumpWidth = 1U;
    hfdcan->Init.NominalTimeSeg1 = 29U;
    hfdcan->Init.NominalTimeSeg2 = 10U;
    /* 160 MHz / (2 * 20) = 4 Mbit/s, sample point 80%, SJW 50 ns. */
    hfdcan->Init.DataPrescaler = 2U;
    hfdcan->Init.DataSyncJumpWidth = 4U;
    hfdcan->Init.DataTimeSeg1 = 15U;
    hfdcan->Init.DataTimeSeg2 = 4U;
    hfdcan->Init.StdFiltersNbr = BOOTLOADER_FDCAN_STD_FILTERS;
    hfdcan->Init.ExtFiltersNbr = BOOTLOADER_FDCAN_EXT_FILTERS;
    hfdcan->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(hfdcan) != HAL_OK) Error_Handler();

    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = BOOTLOADER_FDCAN_ID_TYPE;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = config->can_id & BOOTLOADER_FDCAN_ID_MASK;
    filter.FilterID2 = BOOTLOADER_FDCAN_ID_MASK;
    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) Error_Handler();
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                    FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
        Error_Handler();
    }
}
