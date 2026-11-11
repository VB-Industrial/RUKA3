#pragma once

#include <cstdint>

namespace ruka3 {

struct CommandSnapshot {
    std::uint32_t accepted_count{};
    std::uint32_t age_ms{};
    bool timed_out{};
};

class CommandSupervisor {
public:
    void initialize(std::uint32_t now_ms, std::uint32_t timeout_ms) {
        last_command_ms_ = now_ms;
        timeout_ms_ = timeout_ms;
        initialized_ = true;
        snapshot_ = {};
    }

    void note_accepted_command(std::uint32_t now_ms) {
        last_command_ms_ = now_ms;
        snapshot_.accepted_count += 1U;
        received_command_ = true;
    }

    // Returns true once on the transition into timeout.
    void complete_autonomous_command(std::uint32_t now_ms) { last_command_ms_ = now_ms; }

    bool update(std::uint32_t now_ms, bool autonomous_motion = false) {
        if (!initialized_) initialize(now_ms, timeout_ms_);
        snapshot_.age_ms = now_ms - last_command_ms_;
        const bool timed_out = received_command_ && !autonomous_motion && snapshot_.age_ms > timeout_ms_;
        const bool entered_timeout = timed_out && !snapshot_.timed_out;
        snapshot_.timed_out = timed_out;
        return entered_timeout;
    }

    [[nodiscard]] const CommandSnapshot& snapshot() const { return snapshot_; }

private:
    CommandSnapshot snapshot_{};
    std::uint32_t last_command_ms_{};
    bool initialized_{};
    bool received_command_{};
    std::uint32_t timeout_ms_{1000U};
};

}  // namespace ruka3
