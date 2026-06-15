#include <memory>
#include <string>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

namespace
{
constexpr char kGroupName[] = "single_arm";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "execute_named_pose", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  const bool execute = node->declare_parameter<bool>("execute", false);
  const std::string pose_name = node->declare_parameter<std::string>("pose", "ready");

  moveit::planning_interface::MoveGroupInterface move_group(node, kGroupName);
  move_group.setNamedTarget(pose_name);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool success = static_cast<bool>(move_group.plan(plan));
  RCLCPP_INFO(node->get_logger(), "Named pose '%s' plan %s", pose_name.c_str(), success ? "succeeded" : "failed");
  if (success && execute) {
    const auto result = move_group.execute(plan);
    RCLCPP_INFO(node->get_logger(), "Execute result code: %d", result.val);
  }

  rclcpp::shutdown();
  return success ? 0 : 2;
}
