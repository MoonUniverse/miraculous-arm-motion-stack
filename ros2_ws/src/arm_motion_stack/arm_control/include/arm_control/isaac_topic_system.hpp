#ifndef ARM_CONTROL__ISAAC_TOPIC_SYSTEM_HPP_
#define ARM_CONTROL__ISAAC_TOPIC_SYSTEM_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace arm_control
{

class IsaacTopicSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  double parse_double_param(const std::string & name, double default_value) const;
  std::string parse_string_param(const std::string & name, const std::string & default_value) const;

  std::vector<std::string> joint_names_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  std::vector<double> effort_states_;
  std::vector<double> position_commands_;
  std::vector<bool> has_effort_state_;

  std::string joint_commands_topic_;
  std::string joint_states_topic_;
  double state_timeout_sec_{1.0};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Time last_state_time_;
  bool received_state_{false};
  bool active_{false};
  std::mutex state_mutex_;
};

}  // namespace arm_control

#endif  // ARM_CONTROL__ISAAC_TOPIC_SYSTEM_HPP_
