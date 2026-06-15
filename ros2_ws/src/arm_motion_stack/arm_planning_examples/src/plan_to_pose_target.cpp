#include <memory>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

namespace
{
constexpr char kGroupName[] = "single_arm";
constexpr char kToolLink[] = "tool0";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "plan_to_pose_target", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  const bool execute = node->declare_parameter<bool>("execute", false);

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = node->declare_parameter<double>("x", 0.15);
  target_pose.position.y = node->declare_parameter<double>("y", -0.05);
  target_pose.position.z = node->declare_parameter<double>("z", 0.25);
  target_pose.orientation.x = node->declare_parameter<double>("qx", 0.0);
  target_pose.orientation.y = node->declare_parameter<double>("qy", 0.0);
  target_pose.orientation.z = node->declare_parameter<double>("qz", 0.0);
  target_pose.orientation.w = node->declare_parameter<double>("qw", 1.0);

  moveit::planning_interface::MoveGroupInterface move_group(node, kGroupName);
  move_group.setEndEffectorLink(kToolLink);
  move_group.setPoseTarget(target_pose, kToolLink);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool success = static_cast<bool>(move_group.plan(plan));
  RCLCPP_INFO(node->get_logger(), "Pose target plan %s", success ? "succeeded" : "failed");
  if (success && execute) {
    const auto result = move_group.execute(plan);
    RCLCPP_INFO(node->get_logger(), "Execute result code: %d", result.val);
  }

  rclcpp::shutdown();
  return success ? 0 : 2;
}
