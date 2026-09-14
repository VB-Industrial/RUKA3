#pragma once

#include <cstdint>

namespace voltbro::foc {

constexpr float kPi = 3.14159265359f;
constexpr float kTwoPi = 2.0f * kPi;

inline float wrap_pi(float angle) {
    while (angle > kPi) {
        angle -= kTwoPi;
    }
    while (angle < -kPi) {
        angle += kTwoPi;
    }
    return angle;
}

inline int32_t shortest_tick_error(int32_t current_ticks, int32_t target_ticks, int32_t cpr) {
    int32_t error = target_ticks - current_ticks;
    const int32_t half_cpr = cpr / 2;
    while (error > half_cpr) {
        error -= cpr;
    }
    while (error < -half_cpr) {
        error += cpr;
    }
    return error;
}

inline float gear_angle_from_ticks(int32_t current_ticks, int32_t zero_ticks, int32_t cpr) {
    const int32_t error_ticks = shortest_tick_error(zero_ticks, current_ticks, cpr);
    const float angle = (kTwoPi * static_cast<float>(error_ticks)) / static_cast<float>(cpr);
    return wrap_pi(angle);
}

inline float fuse_angle(float motor_angle, float gear_angle, float blend) {
    const float delta = wrap_pi(gear_angle - motor_angle);
    return wrap_pi(motor_angle + blend * delta);
}

inline float fuse_velocity(float motor_velocity, float gear_velocity, float blend) {
    return motor_velocity + blend * (gear_velocity - motor_velocity);
}

}  // namespace voltbro::foc
