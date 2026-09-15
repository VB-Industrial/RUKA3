#pragma once

#include <cstdint>

namespace ruka3 {

enum class JointState : std::uint8_t {
    boot,
    unconfigured,
    calibrating,
    ready,
    armed,
    fault,
};

class JointController {
public:
    void configuration_loaded(bool valid) {
        state_ = valid ? JointState::ready : JointState::unconfigured;
    }

    void calibration_started() { state_ = JointState::calibrating; }
    void calibration_finished(bool valid) { state_ = valid ? JointState::ready : JointState::fault; }
    void fault() { state_ = JointState::fault; }
    bool acknowledge_fault(bool configuration_valid) {
        if (state_ != JointState::fault || !configuration_valid) return false;
        state_ = JointState::ready;
        return true;
    }

    bool arm() {
        if (state_ != JointState::ready) return false;
        state_ = JointState::armed;
        return true;
    }

    void disarm() {
        if (state_ == JointState::armed) state_ = JointState::ready;
    }

    [[nodiscard]] bool accepts_motor_commands() const {
        return state_ == JointState::ready || state_ == JointState::armed;
    }

    [[nodiscard]] JointState state() const { return state_; }

private:
    JointState state_{JointState::boot};
};

}  // namespace ruka3
