#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>

TEST(Ruka3Plugin, LoadsAndReportsMissingCanInterface)
{
  rclcpp::init(0, nullptr);
  {
    pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "hardware_interface", "hardware_interface::SystemInterface");
    auto system = loader.createUniqueInstance("ruka3_control/Ruka3System");
    hardware_interface::HardwareInfo info;
    info.name = "Ruka3PluginProbe";
    info.hardware_parameters["can_interface"] = "ruka3_missing";
    using Result = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    ASSERT_EQ(system->on_init(info), Result::SUCCESS);
    // Exercise diagnostic publishers and dynamic dependencies without opening real CAN.
    EXPECT_EQ(system->on_configure(rclcpp_lifecycle::State()), Result::ERROR);
  }
  rclcpp::shutdown();
}
