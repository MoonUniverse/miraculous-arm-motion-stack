#include "arm_control/isaac_topic_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace arm_control
{

hardware_interface::CallbackReturn IsaacTopicSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.clear();
  joint_names_.reserve(info_.joints.size());
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IsaacTopicSystem"),
        "Joint '%s' must expose exactly one position command interface.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const bool has_position_state = std::any_of(
      joint.state_interfaces.begin(), joint.state_interfaces.end(),
      [](const auto & interface) { return interface.name == hardware_interface::HW_IF_POSITION; });
    const bool has_velocity_state = std::any_of(
      joint.state_interfaces.begin(), joint.state_interfaces.end(),
      [](const auto & interface) { return interface.name == hardware_interface::HW_IF_VELOCITY; });
    if (!has_position_state || !has_velocity_state) {
      RCLCPP_FATAL(
        rclcpp::get_logger("IsaacTopicSystem"),
        "Joint '%s' must expose position and velocity state interfaces.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_names_.push_back(joint.name);
  }

  position_states_.assign(joint_names_.size(), 0.0);
  velocity_states_.assign(joint_names_.size(), 0.0);
  effort_states_.assign(joint_names_.size(), 0.0);
  position_commands_.assign(joint_names_.size(), 0.0);
  has_effort_state_.assign(joint_names_.size(), false);

  for (size_t index = 0; index < info_.joints.size(); ++index) {
    for (const auto & interface : info_.joints[index].state_interfaces) {
      if (interface.name == hardware_interface::HW_IF_EFFORT) {
        has_effort_state_[index] = true;
      }
      if (interface.name != hardware_interface::HW_IF_POSITION) {
        continue;
      }
      if (!interface.initial_value.empty()) {
        try {
          position_states_[index] = std::stod(interface.initial_value);
          position_commands_[index] = position_states_[index];
        } catch (const std::exception &) {
          RCLCPP_WARN(
            rclcpp::get_logger("IsaacTopicSystem"),
            "Ignoring invalid initial_value '%s' for joint '%s'.",
            interface.initial_value.c_str(), info_.joints[index].name.c_str());
        }
      }
    }
  }

  joint_commands_topic_ = parse_string_param("joint_commands_topic", "/isaac_joint_commands");
  joint_states_topic_ = parse_string_param("joint_states_topic", "/isaac_joint_states");
  state_timeout_sec_ = parse_double_param("state_timeout_sec", 1.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IsaacTopicSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  node_ = std::make_shared<rclcpp::Node>("arm_isaac_topic_system");
  command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    joint_commands_topic_, rclcpp::SystemDefaultsQoS());
  state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      joint_state_callback(msg);
    });
  last_state_time_ = node_->now();

  RCLCPP_INFO(
    node_->get_logger(),
    "Isaac topic hardware configured: commands=%s states=%s joints=%zu",
    joint_commands_topic_.c_str(), joint_states_topic_.c_str(), joint_names_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IsaacTopicSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  active_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IsaacTopicSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> IsaacTopicSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(joint_names_.size() * 3);
  for (size_t index = 0; index < joint_names_.size(); ++index) {
    interfaces.emplace_back(joint_names_[index], hardware_interface::HW_IF_POSITION, &position_states_[index]);
    interfaces.emplace_back(joint_names_[index], hardware_interface::HW_IF_VELOCITY, &velocity_states_[index]);
    if (has_effort_state_[index]) {
      interfaces.emplace_back(joint_names_[index], hardware_interface::HW_IF_EFFORT, &effort_states_[index]);
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> IsaacTopicSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(joint_names_.size());
  for (size_t index = 0; index < joint_names_.size(); ++index) {
    interfaces.emplace_back(joint_names_[index], hardware_interface::HW_IF_POSITION, &position_commands_[index]);
  }
  return interfaces;
}

hardware_interface::return_type IsaacTopicSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (node_) {
    rclcpp::spin_some(node_);
  }

  if (active_ && received_state_ && state_timeout_sec_ > 0.0) {
    const double age = (node_->now() - last_state_time_).seconds();
    if (age > state_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "No Isaac joint state received for %.3f seconds.", age);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IsaacTopicSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!command_pub_) {
    return hardware_interface::return_type::ERROR;
  }

  sensor_msgs::msg::JointState command;
  command.header.stamp = node_->now();
  command.name = joint_names_;
  command.position = position_commands_;
  command_pub_->publish(command);
  return hardware_interface::return_type::OK;
}

void IsaacTopicSystem::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (size_t joint_index = 0; joint_index < joint_names_.size(); ++joint_index) {
    const auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[joint_index]);
    if (it == msg->name.end()) {
      continue;
    }
    const size_t msg_index = static_cast<size_t>(std::distance(msg->name.begin(), it));
    if (msg_index < msg->position.size() &&
      std::isfinite(msg->position[msg_index]))
    {
      position_states_[joint_index] = msg->position[msg_index];
    }
    if (msg_index < msg->velocity.size() &&
      std::isfinite(msg->velocity[msg_index]))
    {
      velocity_states_[joint_index] = msg->velocity[msg_index];
    }
    if (msg_index < msg->effort.size() &&
      std::isfinite(msg->effort[msg_index]))
    {
      effort_states_[joint_index] = msg->effort[msg_index];
    }
  }
  received_state_ = true;
  last_state_time_ = node_->now();
}

double IsaacTopicSystem::parse_double_param(const std::string & name, double default_value) const
{
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end()) {
    return default_value;
  }
  try {
    return std::stod(it->second);
  } catch (const std::exception &) {
    return default_value;
  }
}

std::string IsaacTopicSystem::parse_string_param(
  const std::string & name, const std::string & default_value) const
{
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end() || it->second.empty()) {
    return default_value;
  }
  return it->second;
}

}  // namespace arm_control

PLUGINLIB_EXPORT_CLASS(arm_control::IsaacTopicSystem, hardware_interface::SystemInterface)
