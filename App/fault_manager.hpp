#pragma once

#include <cstdint>

#include "can_fault_monitor.hpp"

namespace ruka3 {

// Values exposed via Cyphal registers; keep their numeric representation stable.
enum class FaultLevel : std::uint32_t { nominal = 0, warning = 1, degraded = 2, fault = 3 };
enum class StopReason : std::uint32_t { none = 0, driver_fault = 1, can_bus_off = 2, current_loop_fault = 3 };
enum class NetworkState : std::uint32_t { starting = 0, online = 1, degraded = 2, offline = 3 };

enum FaultBit : std::uint32_t {
    fault_none = 0,
    fault_driver = 1U << 0U,
    fault_can_bus_off = 1U << 1U,
    fault_can_passive = 1U << 2U,
    fault_calibration_invalid = 1U << 3U,
    fault_current_loop = 1U << 4U,
};

struct FaultSnapshot {
    std::uint32_t active{};
    std::uint32_t latched{};
    FaultLevel level{FaultLevel::nominal};
    StopReason stop_reason{StopReason::none};
    NetworkState network_state{NetworkState::starting};
    float mcu_temperature_c{};
    float stator_temperature_c{};
};

// Hardware-specific code samples these inputs; the common manager owns all
// fault classification and publishes the resulting snapshot.
struct FaultInputs {
    CanFaultSnapshot can{};
    bool calibration_valid{};
    bool driver_fault{};
    float mcu_temperature_c{};
    float stator_temperature_c{};
    bool current_loop_fault{};
};

// Application-level policy only. app.cpp owns the physical motor stop action.
class FaultManager {
public:
    void initialize() { initialized_ = true; }

    // Returns true once when a new motion-blocking condition is seen.
    bool update(const FaultInputs& inputs) {
        std::uint32_t active = fault_none;
        if (inputs.driver_fault) active |= fault_driver;
        if (!inputs.calibration_valid) active |= fault_calibration_invalid;
        if (inputs.current_loop_fault) active |= fault_current_loop;

        const std::uint32_t blocking = active & kBlockingFaults;
        const bool entered_blocking = initialized_ && blocking != 0U && previous_blocking_ == 0U;
        previous_blocking_ = blocking;

        snapshot_.active = active;
        snapshot_.mcu_temperature_c = inputs.mcu_temperature_c;
        snapshot_.stator_temperature_c = inputs.stator_temperature_c;
        if (blocking != 0U) {
            snapshot_.latched |= blocking;
            snapshot_.stop_reason = (blocking & fault_driver) ? StopReason::driver_fault : StopReason::current_loop_fault;
            snapshot_.level = FaultLevel::fault;
        } else if ((active & fault_calibration_invalid) != 0U) {
            snapshot_.level = FaultLevel::warning;
        } else {
            snapshot_.level = FaultLevel::nominal;
        }
        if (snapshot_.latched == 0U) {
            snapshot_.stop_reason = StopReason::none;
        }
        snapshot_.network_state = inputs.can.bus_off ? NetworkState::offline
            : inputs.can.error_passive ? NetworkState::degraded : NetworkState::online;
        return entered_blocking;
    }

    // Only clears a latch once its physical/network cause disappeared.
    bool acknowledge() {
        if ((snapshot_.active & kBlockingFaults) != 0U) return false;
        snapshot_.latched &= ~kBlockingFaults;
        if (snapshot_.latched == 0U) snapshot_.stop_reason = StopReason::none;
        return true;
    }

    [[nodiscard]] const FaultSnapshot& snapshot() const { return snapshot_; }

private:
    static constexpr std::uint32_t kBlockingFaults = fault_driver | fault_current_loop;
    FaultSnapshot snapshot_{};
    std::uint32_t previous_blocking_{};
    bool initialized_{};
};

}  // namespace ruka3
