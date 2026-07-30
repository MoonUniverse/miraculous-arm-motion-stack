#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "arm_planning_examples/moveit_demo_utils.hpp"

namespace
{
constexpr char kGroupName[] = "single_arm";

double durationSeconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1.0e-9;
}

bool finiteAndPositive(double value)
{
  return std::isfinite(value) && value > 0.0;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("moveit_joint_smoke_test");
  arm_planning_examples::ensureMoveItRobotDescriptionParameters(node);

  const bool execute = node->declare_parameter<bool>("execute", false);
  const std::string joint_states_topic =
    node->declare_parameter<std::string>("joint_states_topic", "/arm_joint_states");
  const std::string target_joint =
    node->declare_parameter<std::string>("joint", "J1");
  const double delta_rad = node->declare_parameter<double>("delta_rad", 0.02);
  const double max_abs_delta =
    node->declare_parameter<double>("max_abs_delta", 0.05);
  const double velocity_scaling =
    node->declare_parameter<double>("velocity_scaling", 0.20);
  const double acceleration_scaling =
    node->declare_parameter<double>("acceleration_scaling", 0.20);
  const double start_tolerance =
    node->declare_parameter<double>("start_tolerance", 0.01);
  const double target_tolerance =
    node->declare_parameter<double>("target_tolerance", 0.005);
  const double hold_tolerance =
    node->declare_parameter<double>("hold_tolerance", 0.005);
  const double max_planned_velocity =
    node->declare_parameter<double>("max_planned_velocity", 0.05);
  const double min_duration_sec =
    node->declare_parameter<double>("min_duration_sec", 1.0);

  if (!std::isfinite(delta_rad) || !finiteAndPositive(max_abs_delta) ||
    std::abs(delta_rad) > max_abs_delta ||
    !finiteAndPositive(velocity_scaling) || velocity_scaling > 1.0 ||
    !finiteAndPositive(acceleration_scaling) || acceleration_scaling > 1.0 ||
    !finiteAndPositive(start_tolerance) ||
    !finiteAndPositive(target_tolerance) ||
    !finiteAndPositive(hold_tolerance) ||
    !finiteAndPositive(max_planned_velocity) ||
    !finiteAndPositive(min_duration_sec))
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Unsafe parameters rejected (delta must be finite and <= max_abs_delta; "
      "scales in (0,1]; tolerances, velocity, and duration > 0).");
    rclcpp::shutdown();
    return 2;
  }

  moveit::planning_interface::MoveGroupInterface move_group(node, kGroupName);
  move_group.setMaxVelocityScalingFactor(velocity_scaling);
  move_group.setMaxAccelerationScalingFactor(acceleration_scaling);

  sensor_msgs::msg::JointState joint_state_message;
  if (!rclcpp::wait_for_message(
      joint_state_message, node, joint_states_topic, std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(
      node->get_logger(), "No joint state received on %s within 5 seconds.",
      joint_states_topic.c_str());
    rclcpp::shutdown();
    return 3;
  }
  const rclcpp::Time message_stamp(joint_state_message.header.stamp);
  const double message_age = (node->now() - message_stamp).seconds();
  if (message_stamp.nanoseconds() == 0 || !std::isfinite(message_age) ||
    message_age < -0.1 || message_age > 1.0)
  {
    RCLCPP_ERROR(
      node->get_logger(), "Joint state timestamp is missing or stale (age %.3f s).",
      message_age);
    rclcpp::shutdown();
    return 3;
  }

  const moveit::core::RobotModelConstPtr robot_model = move_group.getRobotModel();
  if (!robot_model || !robot_model->hasJointModel(target_joint)) {
    RCLCPP_ERROR(
      node->get_logger(), "No robot model or unknown target joint '%s'.",
      target_joint.c_str());
    rclcpp::shutdown();
    return 3;
  }

  const std::vector<std::string> joint_names =
    move_group.getJointNames();
  if (joint_state_message.name.size() != joint_state_message.position.size()) {
    RCLCPP_ERROR(node->get_logger(), "Joint state name/position lengths differ.");
    rclcpp::shutdown();
    return 4;
  }
  std::map<std::string, double> message_positions;
  for (size_t index = 0; index < joint_state_message.name.size(); ++index) {
    if (!message_positions.emplace(
        joint_state_message.name[index], joint_state_message.position[index]).second)
    {
      RCLCPP_ERROR(node->get_logger(), "Joint state contains a duplicate name.");
      rclcpp::shutdown();
      return 4;
    }
  }

  moveit::core::RobotState current_state(robot_model);
  current_state.setToDefaultValues();
  std::map<std::string, double> current_positions;
  std::map<std::string, double> target_positions;
  for (const auto & name : joint_names) {
    const auto value_it = message_positions.find(name);
    if (value_it == message_positions.end() || !std::isfinite(value_it->second)) {
      RCLCPP_ERROR(
        node->get_logger(), "Missing or non-finite current state for %s.",
        name.c_str());
      rclcpp::shutdown();
      return 4;
    }
    const double value = value_it->second;
    current_state.setVariablePosition(name, value);
    current_positions[name] = value;
    target_positions[name] = value;
  }
  current_state.update();
  if (target_positions.find(target_joint) == target_positions.end()) {
    RCLCPP_ERROR(
      node->get_logger(), "Joint '%s' is not part of planning group '%s'.",
      target_joint.c_str(), kGroupName);
    rclcpp::shutdown();
    return 5;
  }
  target_positions[target_joint] += delta_rad;

  move_group.setStartState(current_state);
  if (!move_group.setJointValueTarget(target_positions)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Current-relative target violates the MoveIt joint model bounds.");
    rclcpp::shutdown();
    return 6;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (!static_cast<bool>(move_group.plan(plan))) {
    RCLCPP_ERROR(node->get_logger(), "MoveIt failed to plan the guarded joint step.");
    rclcpp::shutdown();
    return 7;
  }

  const auto & trajectory = plan.trajectory_.joint_trajectory;
  if (trajectory.joint_names.size() != joint_names.size() ||
    trajectory.points.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "Planned trajectory is empty or has wrong joint count.");
    rclcpp::shutdown();
    return 8;
  }

  std::map<std::string, size_t> trajectory_index;
  for (size_t index = 0; index < trajectory.joint_names.size(); ++index) {
    trajectory_index[trajectory.joint_names[index]] = index;
  }
  for (const auto & name : joint_names) {
    if (trajectory_index.find(name) == trajectory_index.end()) {
      RCLCPP_ERROR(node->get_logger(), "Trajectory omitted joint %s.", name.c_str());
      rclcpp::shutdown();
      return 9;
    }
  }

  const auto & first = trajectory.points.front();
  const auto & last = trajectory.points.back();
  if (first.positions.size() != trajectory.joint_names.size() ||
    last.positions.size() != trajectory.joint_names.size())
  {
    RCLCPP_ERROR(node->get_logger(), "Trajectory position vectors have wrong length.");
    rclcpp::shutdown();
    return 10;
  }

  for (const auto & name : joint_names) {
    const size_t index = trajectory_index.at(name);
    if (!std::isfinite(first.positions[index]) ||
      std::abs(first.positions[index] - current_positions.at(name)) > start_tolerance)
    {
      RCLCPP_ERROR(
        node->get_logger(), "Trajectory start mismatch for %s.", name.c_str());
      rclcpp::shutdown();
      return 11;
    }
  }

  for (const auto & point : trajectory.points) {
    if (point.positions.size() != trajectory.joint_names.size()) {
      RCLCPP_ERROR(node->get_logger(), "A trajectory point has wrong position count.");
      rclcpp::shutdown();
      return 12;
    }
    for (const auto & name : joint_names) {
      const size_t index = trajectory_index.at(name);
      if (!std::isfinite(point.positions[index])) {
        RCLCPP_ERROR(node->get_logger(), "Non-finite trajectory position.");
        rclcpp::shutdown();
        return 13;
      }
      if (name != target_joint &&
        std::abs(point.positions[index] - current_positions.at(name)) > hold_tolerance)
      {
        RCLCPP_ERROR(
          node->get_logger(), "Trajectory moves held joint %s by more than %.4f rad.",
          name.c_str(), hold_tolerance);
        rclcpp::shutdown();
        return 14;
      }
    }
    if (!point.velocities.empty()) {
      if (point.velocities.size() != trajectory.joint_names.size()) {
        RCLCPP_ERROR(node->get_logger(), "A trajectory point has wrong velocity count.");
        rclcpp::shutdown();
        return 15;
      }
      for (const double velocity : point.velocities) {
        if (!std::isfinite(velocity) ||
          std::abs(velocity) > max_planned_velocity + 1.0e-9)
        {
          RCLCPP_ERROR(node->get_logger(), "Trajectory velocity guard failed.");
          rclcpp::shutdown();
          return 16;
        }
      }
    }
  }

  const size_t target_index = trajectory_index.at(target_joint);
  const double planned_delta =
    last.positions[target_index] - current_positions.at(target_joint);
  const double duration_sec = durationSeconds(last.time_from_start);
  if (std::abs(planned_delta - delta_rad) > target_tolerance ||
    std::abs(planned_delta) > max_abs_delta + target_tolerance ||
    !std::isfinite(duration_sec) || duration_sec < min_duration_sec)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Final trajectory guard failed: requested=%.6f planned=%.6f duration=%.3f.",
      delta_rad, planned_delta, duration_sec);
    rclcpp::shutdown();
    return 17;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Guarded plan accepted: %s delta=%.6f rad, duration=%.3f s, execute=%s.",
    target_joint.c_str(), planned_delta, duration_sec, execute ? "true" : "false");

  if (execute) {
    const auto result = move_group.execute(plan);
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Execution failed with code %d.", result.val);
      rclcpp::shutdown();
      return 18;
    }
  }

  rclcpp::shutdown();
  return 0;
}
