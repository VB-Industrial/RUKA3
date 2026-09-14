#include "fdcan.h"

FDCAN_HandleTypeDef hfdcan1;

void MX_FDCAN1_Init(void) {
    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = ENABLE;
    hfdcan1.Init.TransmitPause = ENABLE;
    hfdcan1.Init.ProtocolException = DISABLE;
    hfdcan1.Init.NominalPrescaler = 4U;
    hfdcan1.Init.NominalSyncJumpWidth = 1U;
    hfdcan1.Init.NominalTimeSeg1 = 29U;
    hfdcan1.Init.NominalTimeSeg2 = 10U;
    hfdcan1.Init.DataPrescaler = 4U;
    hfdcan1.Init.DataSyncJumpWidth = 1U;
    hfdcan1.Init.DataTimeSeg1 = 2U;
    hfdcan1.Init.DataTimeSeg2 = 2U;
    hfdcan1.Init.StdFiltersNbr = 0U;
    hfdcan1.Init.ExtFiltersNbr = 8U;
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
}
