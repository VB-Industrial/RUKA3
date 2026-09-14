#pragma once

#include <cstdint>

#include "stm32g4xx_hal.h"

namespace ruka3 {

struct CanFaultSnapshot {
    std::uint32_t session_error_count{};
    std::uint32_t rx_error_count{};
    std::uint32_t tx_error_count{};
    std::uint32_t error_logging_count{};
    std::uint32_t arbitration_last_error{};
    std::uint32_t data_last_error{};
    bool warning{};
    bool error_passive{};
    bool bus_off{};
};

class CanFaultMonitor {
public:
    explicit CanFaultMonitor(FDCAN_HandleTypeDef& fdcan) : fdcan_(fdcan) {}

    void poll() {
        FDCAN_ErrorCountersTypeDef counters{};
        FDCAN_ProtocolStatusTypeDef protocol{};
        if ((HAL_FDCAN_GetErrorCounters(&fdcan_, &counters) != HAL_OK) ||
            (HAL_FDCAN_GetProtocolStatus(&fdcan_, &protocol) != HAL_OK)) {
            return;
        }

        const std::uint32_t logging = counters.ErrorLogging;
        if (!initialized_) {
            initialized_ = true;
            snapshot_.session_error_count = logging;
        } else if (logging >= previous_error_logging_) {
            snapshot_.session_error_count += logging - previous_error_logging_;
        } else {
            // The FDCAN peripheral was reinitialized without restarting the MCU.
            snapshot_.session_error_count += logging;
        }
        previous_error_logging_ = logging;

        snapshot_.rx_error_count = counters.RxErrorCnt;
        snapshot_.tx_error_count = counters.TxErrorCnt;
        snapshot_.error_logging_count = logging;
        snapshot_.arbitration_last_error = protocol.LastErrorCode;
        snapshot_.data_last_error = protocol.DataLastErrorCode;
        snapshot_.warning = protocol.Warning != 0U;
        snapshot_.error_passive = protocol.ErrorPassive != 0U;
        snapshot_.bus_off = protocol.BusOff != 0U;
    }

    [[nodiscard]] const CanFaultSnapshot& snapshot() const { return snapshot_; }

private:
    FDCAN_HandleTypeDef& fdcan_;
    CanFaultSnapshot snapshot_{};
    std::uint32_t previous_error_logging_{};
    bool initialized_{};
};

}  // namespace ruka3
