#ifndef MIRACULOUS_DRIVER__MIRACULOUS_SYSTEM_HPP_
#define MIRACULOUS_DRIVER__MIRACULOUS_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "miraculous_driver/miraculous_arm.hpp"

namespace miraculous_driver
{

/// ros2_control SystemInterface that drives the ARM manipulator through the
/// miraculous_sdk. Mirrors the IsaacTopicSystem structure but talks to real
/// CANopen motors in CSP mode.
class MiraculousSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  double parse_double_param(const std::string & name, double default_value) const;
  std::string parse_string_param(const std::string & name, const std::string & default_value) const;
  int parse_int_param(const std::string & name, int default_value) const;
  std::vector<int> parse_int_list_param(const std::string & name, const std::vector<int> & default_value) const;
  std::vector<double> parse_double_list_param(
    const std::string & name, const std::vector<double> & default_value) const;

  std::vector<std::string> joint_names_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  std::vector<double> position_commands_;

  std::unique_ptr<MiraculousArm> arm_;
  bool active_{false};
};

}  // namespace miraculous_driver

#endif  // MIRACULOUS_DRIVER__MIRACULOUS_SYSTEM_HPP_
