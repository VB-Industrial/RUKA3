#pragma once

#include <array>
#include <memory>
#include <string>
#include "hardware_interface/system_interface.hpp"
#include "ruka3_control/cyphal_contract.hpp"
#include "ruka3_control/limit_recovery.hpp"

namespace ruka3_control
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
namespace detail { class CyphalRuntime; }

class Ruka3System : public hardware_interface::SystemInterface
{
public:
  ~Ruka3System() override;
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  void hold();
  std::array<double, kJointCount> position_{}, velocity_{}, command_{}, command_velocity_{};
  std::unique_ptr<detail::CyphalRuntime> bus_;
  std::string can_interface_{"can0"};
  double activation_timeout_{5.0}, feedback_timeout_{0.25}, heartbeat_timeout_{2.5};
  bool active_{};
  LimitRecovery recovery_;
};
}  // namespace ruka3_control
