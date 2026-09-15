#pragma once

#include <cstdint>

namespace ruka3 {

struct CommandSnapshot {
    std::uint32_t accepted_count{};
    std::uint32_t age_ms{};
    bool timed_out{};
};

// Phase 1 is deliberately diagnostic-only.  The application reads the
// timeout state but does not alter the motor target or drive enable yet.
class CommandSupervisor {
public:
    static constexpr std::uint32_t kTimeoutMs = 1000U;

    void initialize(std::uint32_t now_ms) {
        last_command_ms_ = now_ms;
        initialized_ = true;
        snapshot_ = {};
    }

    void note_accepted_command(std::uint32_t now_ms) {
        last_command_ms_ = now_ms;
        snapshot_.accepted_count += 1U;
        received_command_ = true;
    }

    void update(std::uint32_t now_ms) {
        if (!initialized_) initialize(now_ms);
        snapshot_.age_ms = now_ms - last_command_ms_;
        snapshot_.timed_out = received_command_ && snapshot_.age_ms > kTimeoutMs;
    }

    [[nodiscard]] const CommandSnapshot& snapshot() const { return snapshot_; }

private:
    CommandSnapshot snapshot_{};
    std::uint32_t last_command_ms_{};
    bool initialized_{};
    bool received_command_{};
};

}  // namespace ruka3
