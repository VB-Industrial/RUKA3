#pragma once

#include <cstdint>

namespace ruka3 {

struct OutputObserverConfig {
    std::uint32_t encoder_cpr{};
    std::uint32_t encoder_zero{};
    std::int8_t encoder_sign{1};
    float velocity_lpf{0.1F};
};

struct OutputObserverSnapshot {
    float encoder_angle{};
    float output_velocity{};
    float motor_residual{};
    bool initialized{};
};

class OutputStateObserver {
public:
    explicit constexpr OutputStateObserver(OutputObserverConfig config) : config_(config) {}

    void update(std::uint32_t now_ms, std::uint32_t raw_ticks, float motor_angle) {
        if (config_.encoder_cpr == 0U) return;
        const float angle = ticks_to_angle(raw_ticks);
        if (!snapshot_.initialized) {
            snapshot_.encoder_angle = angle;
            previous_angle_ = angle;
            previous_ms_ = now_ms;
            snapshot_.initialized = true;
        } else if (now_ms != previous_ms_) {
            const float dt = static_cast<float>(now_ms - previous_ms_) / 1000.0F;
            const float velocity = wrap_pi(angle - previous_angle_) / dt;
            snapshot_.output_velocity += config_.velocity_lpf * (velocity - snapshot_.output_velocity);
            previous_angle_ = angle;
            previous_ms_ = now_ms;
            snapshot_.encoder_angle = angle;
        }
        snapshot_.motor_residual = wrap_pi(snapshot_.encoder_angle - motor_angle);
    }

    [[nodiscard]] const OutputObserverSnapshot& snapshot() const { return snapshot_; }

private:
    static constexpr float kTwoPi = 6.28318530718F;
    static float wrap_pi(float value) {
        while (value > 3.14159265359F) value -= kTwoPi;
        while (value < -3.14159265359F) value += kTwoPi;
        return value;
    }
    float ticks_to_angle(std::uint32_t raw_ticks) const {
        const std::int32_t cpr = static_cast<std::int32_t>(config_.encoder_cpr);
        std::int32_t delta = static_cast<std::int32_t>(raw_ticks) - static_cast<std::int32_t>(config_.encoder_zero);
        while (delta > cpr / 2) delta -= cpr;
        while (delta < -cpr / 2) delta += cpr;
        return static_cast<float>(config_.encoder_sign) * kTwoPi * static_cast<float>(delta) / static_cast<float>(cpr);
    }
    OutputObserverConfig config_;
    OutputObserverSnapshot snapshot_{};
    float previous_angle_{};
    std::uint32_t previous_ms_{};
};

}  // namespace ruka3
