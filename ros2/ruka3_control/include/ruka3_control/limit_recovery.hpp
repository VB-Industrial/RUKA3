#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include "ruka3_control/cyphal_contract.hpp"

namespace ruka3_control
{
// One recovery owns the whole arm until an explicit controller restart.
// Other axes hold their measured positions; the offending axes retreat at
// 0.1 rad/s, 0.2 rad/s^2. Gains are unchanged throughout.
class LimitRecovery
{
public:
  bool latched{};
  std::array<double, kJointCount> target{}, velocity{}, goal{};

  template<class Joints>
  bool update(const Joints & joints, double dt)
  {
    bool entered = false;
    if (!latched) {
      for (const auto & j : joints) {
        if (j.position < j.soft_lower || j.position > j.soft_upper) entered = true;
      }
      if (!entered) return false;
      latched = true;
      velocity.fill(0);
      for (size_t i = 0; i < kJointCount; ++i) {
        const auto & j = joints[i];
        start_[i] = target[i] = goal[i] = j.position;
        if (j.position < j.soft_lower) goal[i] = j.soft_lower + 0.01;
        if (j.position > j.soft_upper) goal[i] = j.soft_upper - 0.01;
      }
    }
    elapsed_ += std::clamp(dt, 0.0, 0.02);
    for (size_t i = 0; i < kJointCount; ++i) {
      const double distance = std::fabs(goal[i]-start_[i]);
      const double peak = std::min(0.1, std::sqrt(distance*0.2));
      const double acceleration_time = peak/0.2;
      const double cruise_time = peak > 0 ? std::max(0., (distance-peak*acceleration_time)/peak) : 0;
      const double duration = 2*acceleration_time+cruise_time;
      double travelled{}, speed{};
      if (elapsed_ >= duration) {
        travelled = distance;
      } else if (elapsed_ < acceleration_time) {
        travelled = 0.5*0.2*elapsed_*elapsed_;
        speed = 0.2*elapsed_;
      } else if (elapsed_ < acceleration_time+cruise_time) {
        travelled = 0.5*peak*acceleration_time + peak*(elapsed_-acceleration_time);
        speed = peak;
      } else {
        const double remaining = duration-elapsed_;
        travelled = distance-0.5*0.2*remaining*remaining;
        speed = 0.2*remaining;
      }
      const double sign = goal[i] >= start_[i] ? 1. : -1.;
      target[i] = start_[i]+sign*travelled;
      velocity[i] = sign*speed;
    }
    return entered;
  }

  template<class Joints>
  bool settled(const Joints & joints) const
  {
    for (size_t i = 0; i < kJointCount; ++i) {
      const auto & j = joints[i];
      if (j.position < j.soft_lower || j.position > j.soft_upper ||
        std::fabs(j.position-goal[i]) > 0.01 || std::fabs(j.velocity) > 0.02) return false;
    }
    return true;
  }
private:
  std::array<double, kJointCount> start_{};
  double elapsed_{};
};
}  // namespace ruka3_control
