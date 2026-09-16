#pragma once

#include <cstdint>

namespace ruka3 {

struct OutputObserverConfig {
    std::uint32_t encoder_cpr{};
    std::uint32_t encoder_zero{};
    std::int8_t encoder_sign{1};
    float velocity_lpf{0.1F};
    std::uint8_t fusion_encoder_corridor_ticks{2U};
};

struct OutputObserverSnapshot {
    float encoder_angle{};
    float output_velocity{};
    float motor_residual{};
    float fused_angle{};
    float fused_velocity{};
    float fusion_offset{};
    float encoder_error{};
    bool initialized{};
};

class OutputStateObserver {
public:
    explicit constexpr OutputStateObserver(OutputObserverConfig config) : config_(config) {}

    void set_encoder_zero(std::uint32_t encoder_zero) {
        config_.encoder_zero = encoder_zero;
        snapshot_ = {};
        previous_angle_ = 0.0F;
        previous_motor_angle_ = 0.0F;
        motor_alignment_offset_ = 0.0F;
        encoder_unwrapped_angle_ = 0.0F;
        previous_ms_ = 0U;
    }

    void update(std::uint32_t now_ms, std::uint32_t raw_ticks, float motor_angle) {
        if (config_.encoder_cpr == 0U) return;
        const float angle = ticks_to_angle(raw_ticks);
        if (!snapshot_.initialized) {
            snapshot_.encoder_angle = angle;
            encoder_unwrapped_angle_ = angle;
            motor_alignment_offset_ = angle - motor_angle;
            snapshot_.fused_angle = angle;
            previous_angle_ = angle;
            previous_motor_angle_ = motor_angle;
            previous_ms_ = now_ms;
            snapshot_.initialized = true;
        } else if (now_ms != previous_ms_) {
            const float dt = static_cast<float>(now_ms - previous_ms_) / 1000.0F;
            const float encoder_delta = wrap_pi(angle - previous_angle_);
            encoder_unwrapped_angle_ += encoder_delta;
            const float velocity = encoder_delta / dt;
            snapshot_.output_velocity += config_.velocity_lpf * (velocity - snapshot_.output_velocity);

            const float previous_fused_angle = snapshot_.fused_angle;
            float predicted_angle = previous_fused_angle + (motor_angle - previous_motor_angle_);
            const float encoder_error = encoder_unwrapped_angle_ - predicted_angle;
            const float corridor = kTwoPi * static_cast<float>(config_.fusion_encoder_corridor_ticks) /
                                   static_cast<float>(config_.encoder_cpr);
            if (absolute(encoder_error) > corridor) {
                predicted_angle += encoder_error - signed_value(corridor, encoder_error);
            }
            snapshot_.fused_angle = predicted_angle;
            snapshot_.encoder_error = encoder_unwrapped_angle_ - predicted_angle;
            snapshot_.fusion_offset = predicted_angle - (motor_angle + motor_alignment_offset_);
            const float fused_velocity = (predicted_angle - previous_fused_angle) / dt;
            snapshot_.fused_velocity += config_.velocity_lpf * (fused_velocity - snapshot_.fused_velocity);
            previous_angle_ = angle;
            previous_motor_angle_ = motor_angle;
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
    static float absolute(float value) { return value < 0.0F ? -value : value; }
    static float signed_value(float magnitude, float sign_source) {
        return sign_source < 0.0F ? -magnitude : magnitude;
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
    float previous_motor_angle_{};
    float motor_alignment_offset_{};
    float encoder_unwrapped_angle_{};
    std::uint32_t previous_ms_{};
};

}  // namespace ruka3
