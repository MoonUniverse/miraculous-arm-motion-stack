#include <map>
#include <memory>
#include <string>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

#include "arm_planning_examples/moveit_demo_utils.hpp"

namespace
{
constexpr char kGroupName[] = "single_arm";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("plan_to_joint_target");
  arm_planning_examples::ensureMoveItRobotDescriptionParameters(node);

  const bool execute = node->declare_parameter<bool>("execute", false);
  std::map<std::string, double> target{
    {"J1", node->declare_parameter<double>("J1", 0.0)},
    {"J2", node->declare_parameter<double>("J2", -0.5)},
    {"J3", node->declare_parameter<double>("J3", 0.8)},
    {"J4", node->declare_parameter<double>("J4", 0.0)},
    {"J5", node->declare_parameter<double>("J5", 0.6)},
    {"J6", node->declare_parameter<double>("J6", 0.0)},
  };

  moveit::planning_interface::MoveGroupInterface move_group(node, kGroupName);
  move_group.setJointValueTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool success = static_cast<bool>(move_group.plan(plan));
  RCLCPP_INFO(node->get_logger(), "Joint target plan %s", success ? "succeeded" : "failed");
  if (success && execute) {
    const auto result = move_group.execute(plan);
    RCLCPP_INFO(node->get_logger(), "Execute result code: %d", result.val);
  }

  rclcpp::shutdown();
  return success ? 0 : 2;
}
