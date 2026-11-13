#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include "joint_calibration.hpp"
#include "robot_config.h"

namespace ruka3 {

enum class ServiceMotionMode : std::uint8_t { idle, position, voltage, current, hold, servo };
struct ServiceMotionSnapshot {
    ServiceMotionMode mode{ServiceMotionMode::idle};
    float target{}, error{}, velocity{}, goal{}, torque{}, kp{}, kd{};
};

// Kp/Kd and feedforward are output-shaft Nm in the public joint coordinate.
inline float position_current_request(const ServiceMotionSnapshot& command, float measured_velocity,
                                      float torque_const, float gear_ratio) {
    return (command.kp*command.error + command.kd*(command.velocity-measured_velocity) +
            command.torque)/(torque_const*gear_ratio);
}

struct WrappedPositionTarget { float angle_error, torque; };
inline WrappedPositionTarget wrapped_position_target(const ServiceMotionSnapshot& command) {
    // G474's existing FOC wraps its local angle error. Keep that local error
    // away from the +/-pi branch and carry the remaining unwrapped P term as
    // torque. The total PD law is unchanged, including targets >pi away.
    constexpr float half_pi = 1.5707963267948966F;
    const float local = std::clamp(command.error, -half_pi, half_pi);
    return {local, command.torque + command.kp*(command.error-local)};
}

// Public joint coordinates throughout. Hardware sign conversion happens once
// in each board's motor adapter. Kp is never reduced in a limit zone.
inline float outward_command_scale(float command, float position, const MechanicalCalibrationSnapshot& limits) {
    if (!limits.limits_valid) return 0;
    if (command > 0) {
        if (position >= limits.upper_limit) return 0;
        if (position > limits.soft_upper_limit)
            return (limits.upper_limit-position)/(limits.upper_limit-limits.soft_upper_limit);
    } else if (command < 0) {
        if (position <= limits.lower_limit) return 0;
        if (position < limits.soft_lower_limit)
            return (position-limits.lower_limit)/(limits.soft_lower_limit-limits.lower_limit);
    }
    return 1;
}
inline float soft_limited_target(float target, float position, const MechanicalCalibrationSnapshot& limits) {
    if ((position > limits.soft_upper_limit && target > position) ||
        (position < limits.soft_lower_limit && target < position)) return position;
    return target;
}

// Shared G431/G474 application motion state. Trajectory ticks never block CAN.
// A finite set_pos owns its trajectory and final hold; incoming commands replace it.
class ServiceMotion {
public:
    explicit ServiceMotion(const CommandTuning& tuning, float holding_current_limit = 0) : tuning_(tuning),
        position_kp_(tuning.default_position_kp), position_kd_(tuning.default_velocity_kp),
        ki_(tuning.default_position_ki), integral_limit_(std::min(tuning.default_integral_current_limit_a, holding_current_limit)),
        max_integral_(holding_current_limit) {}
    const ServiceMotionSnapshot& snapshot() const { return state_; }
    float position_kp() const { return position_kp_; }
    float position_kd() const { return position_kd_; }
    bool set_position_gains(float kp, float kd) {
        if (!std::isfinite(kp) || !std::isfinite(kd) || kp < 0 || kd < 0) return false;
        position_kp_ = kp; position_kd_ = kd;
        if (state_.mode == ServiceMotionMode::position || state_.mode == ServiceMotionMode::hold) {
            state_.kp = kp; state_.kd = kd;
        }
        return true;
    }
    float position_ki() const { return ki_; }
    float integral_limit() const { return integral_limit_; }
    float integral_current() const { return integral_current_; }
    bool set_integral(float ki, float limit) {
        if (!std::isfinite(ki) || !std::isfinite(limit) || ki < 0 || limit < 0 || limit > max_integral_) return false;
        ki_ = ki; integral_limit_ = limit;
        integral_current_ = ki == 0 ? 0 : std::clamp(integral_current_, -limit, limit);
        return true;
    }
    // Called once per applied board setpoint. base_current is the public-sign
    // PD + feedforward request in amperes, before the motor's current limiter.
    float compensate_load(std::uint32_t now, float position, float base_current,
                          float current_limit, const MechanicalCalibrationSnapshot& limits) {
        const float dt = integral_clock_valid_ ? (now-integral_updated_)*0.001F : 0;
        integral_updated_ = now; integral_clock_valid_ = true;
        if (ki_ == 0) return 0;
        // Preserve the learned load while moving; learn only with a stationary reference.
        const bool stationary = !trajectory_active_ && state_.velocity == 0;
        if (stationary) {
            const float delta = ki_*state_.error*dt;
            const float candidate = std::clamp(integral_current_+delta, -integral_limit_, integral_limit_);
            const float change = candidate-integral_current_;
            const float total = base_current+candidate;
            // Conditional integration: permit unwinding an already saturated request.
            const bool saturated = (total > current_limit && change > 0) || (total < -current_limit && change < 0);
            const bool blocked = (position >= limits.upper_limit && change > 0) ||
                                 (position <= limits.lower_limit && change < 0);
            if (!saturated && !blocked) integral_current_ = candidate;
        }
        // No learned torque may push farther through a hard boundary.
        if ((position >= limits.upper_limit && integral_current_ > 0) ||
            (position <= limits.lower_limit && integral_current_ < 0)) integral_current_ = 0;
        return integral_current_*outward_command_scale(integral_current_, position, limits);
    }
    bool trajectory_active() const { return trajectory_active_; }
    // A register set_pos remains autonomous while holding its final target.
    bool autonomous_motion_active() const { return state_.mode == ServiceMotionMode::position; }
    void cancel(bool reset_integral = true) {
        state_ = {}; trajectory_active_ = false;
        if (reset_integral) { integral_current_ = 0; integral_clock_valid_ = false; }
    }
    void hold(float position, std::uint32_t now) {
        cancel(false); state_.mode = ServiceMotionMode::hold;
        state_.target = state_.goal = position;
        state_.kp = position_kp_; state_.kd = position_kd_;
        last_update_ = now - period_ms();
    }
    bool position(float target, float current, const MechanicalCalibrationSnapshot& limits, std::uint32_t now) {
        if (!valid_position(target, limits)) return false;
        target = soft_limited_target(target, current, limits);
        if (state_.mode == ServiceMotionMode::position && state_.goal == target) return true;
        cancel(false);
        state_.mode = ServiceMotionMode::position;
        state_.target = start_ = current; state_.goal = target;
        state_.kp = position_kp_; state_.kd = position_kd_;
        distance_ = std::fabs(target-current);
        sign_ = target >= current ? 1.0F : -1.0F;
        const float v = tuning_.service_position_velocity_rad_s;
        const float a = tuning_.service_position_acceleration_rad_s2;
        peak_velocity_ = std::min(v, std::sqrt(distance_*a));
        acceleration_time_ = peak_velocity_/a;
        cruise_time_ = peak_velocity_ > 0 ? std::max(0.0F, (distance_-peak_velocity_*acceleration_time_)/peak_velocity_) : 0;
        duration_ = 2*acceleration_time_ + cruise_time_;
        started_ = now; last_update_ = now - period_ms();
        trajectory_active_ = distance_ > 0;
        return true;
    }
    bool direct(ServiceMotionMode mode, float value, float bound, const MechanicalCalibrationSnapshot& limits, std::uint32_t now) {
        if (!limits.limits_valid || !std::isfinite(value) || std::fabs(value) > bound) return false;
        cancel(); state_.mode = mode; state_.goal = state_.target = value;
        last_update_ = now - period_ms();
        return true;
    }
    bool servo(float target, float velocity, float torque, float kp, float kd,
               float current, const MechanicalCalibrationSnapshot& limits, std::uint32_t now) {
        if (!valid_position(target, limits) || !std::isfinite(velocity) || !std::isfinite(torque) ||
            !std::isfinite(kp) || !std::isfinite(kd) || kp < 0 || kd < 0) return false;
        target = soft_limited_target(target, current, limits);
        const bool acquiring = state_.mode == ServiceMotionMode::servo && trajectory_active_;
        if (!acquiring) {
            const bool new_goal = state_.mode != ServiceMotionMode::servo ||
                std::fabs(target-state_.goal) > tuning_.acquire_threshold_rad;
            if (new_goal && std::fabs(target-current) > tuning_.acquire_threshold_rad) {
                // Capture only initial/discontinuous far references. Residual
                // tracking error must retain PD torque after capture, rather
                // than restarting a ramp from the measured position.
                position(target, current, limits, now);
            } else {
                cancel(false); state_.target = target;
            }
        }
        state_.mode = ServiceMotionMode::servo;
        state_.goal = target; // Latest packet; the capture endpoint stays in start_/distance_.
        requested_velocity_ = velocity; requested_torque_ = torque;
        state_.kp = kp; state_.kd = kd;
        // Direct packets apply immediately; an active capture retains its tick clock.
        if (!trajectory_active_) last_update_ = now - period_ms();
        return true;
    }
    // Returns true only when a new board setpoint is due.
    bool update(std::uint32_t now, float position, const MechanicalCalibrationSnapshot& limits) {
        if (state_.mode == ServiceMotionMode::idle || now-last_update_ < period_ms()) return false;
        last_update_ = now;
        if (state_.mode == ServiceMotionMode::servo) state_.torque = requested_torque_;
        if (state_.mode == ServiceMotionMode::position || trajectory_active_) {
            const float t = (now-started_)*0.001F;
            const float a = tuning_.service_position_acceleration_rad_s2;
            float travelled{}, velocity{};
            if (!trajectory_active_ || t >= duration_) {
                travelled = distance_; trajectory_active_ = false;
            } else if (t < acceleration_time_) {
                travelled = 0.5F*a*t*t; velocity = a*t;
            } else if (t < acceleration_time_+cruise_time_) {
                travelled = 0.5F*a*acceleration_time_*acceleration_time_ + peak_velocity_*(t-acceleration_time_);
                velocity = peak_velocity_;
            } else {
                const float remaining = duration_-t;
                travelled = distance_-0.5F*a*remaining*remaining; velocity = a*remaining;
            }
            state_.target = start_ + sign_*travelled;
            state_.velocity = sign_*velocity;
        } else if (state_.mode == ServiceMotionMode::servo) {
            state_.velocity = requested_velocity_;
            state_.torque = requested_torque_;
        }
        if (state_.mode == ServiceMotionMode::position || state_.mode == ServiceMotionMode::hold || state_.mode == ServiceMotionMode::servo) {
            const float limited = soft_limited_target(state_.target, position, limits);
            if (limited != state_.target) {
                state_.target = limited;
                state_.velocity = 0;
                // Keep the inward profile alive if the mechanism has moved
                // ahead of it. Subsequent ticks can still retreat to its goal.
                if (state_.mode == ServiceMotionMode::hold) state_.goal = limited;
            }
            state_.error = state_.target-position;
            state_.velocity *= outward_command_scale(state_.velocity, position, limits);
            state_.torque *= outward_command_scale(state_.torque, position, limits);
        }
        return true;
    }
private:
    bool valid_position(float target, const MechanicalCalibrationSnapshot& limits) const {
        return limits.limits_valid && std::isfinite(target) && target >= limits.lower_limit && target <= limits.upper_limit;
    }
    std::uint32_t period_ms() const { return 1000U/tuning_.service_position_update_hz; }
    const CommandTuning& tuning_;
    float position_kp_, position_kd_;
    float ki_, integral_limit_, max_integral_, integral_current_{};
    std::uint32_t integral_updated_{};
    bool integral_clock_valid_{};
    ServiceMotionSnapshot state_{};
    float requested_velocity_{}, requested_torque_{};
    float start_{}, distance_{}, sign_{1}, peak_velocity_{}, acceleration_time_{}, cruise_time_{}, duration_{};
    std::uint32_t started_{}, last_update_{};
    bool trajectory_active_{};
};
} // namespace ruka3

// Each board owns one instance; common register handlers configure the same controller.
ruka3::ServiceMotion& joint_motion_instance();
