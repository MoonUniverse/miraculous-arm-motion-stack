#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

namespace
{
constexpr char kGroupName[] = "single_arm";
constexpr char kToolLink[] = "tool0";
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "ik_demo", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = node->declare_parameter<double>("x", 0.15);
  target_pose.position.y = node->declare_parameter<double>("y", -0.05);
  target_pose.position.z = node->declare_parameter<double>("z", 0.25);
  target_pose.orientation.x = node->declare_parameter<double>("qx", 0.0);
  target_pose.orientation.y = node->declare_parameter<double>("qy", 0.0);
  target_pose.orientation.z = node->declare_parameter<double>("qz", 0.0);
  target_pose.orientation.w = node->declare_parameter<double>("qw", 1.0);

  moveit::planning_interface::MoveGroupInterface move_group(node, kGroupName);
  auto state = move_group.getCurrentState(5.0);
  if (!state) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get current robot state.");
    rclcpp::shutdown();
    return 1;
  }

  const auto * joint_model_group = state->getJointModelGroup(kGroupName);
  const bool found_ik = state->setFromIK(joint_model_group, target_pose, kToolLink, 0.1);
  if (!found_ik) {
    RCLCPP_ERROR(node->get_logger(), "IK failed for target pose.");
    rclcpp::shutdown();
    return 2;
  }

  std::vector<double> solution;
  state->copyJointGroupPositions(joint_model_group, solution);
  const auto & joint_names = joint_model_group->getVariableNames();
  RCLCPP_INFO(node->get_logger(), "IK solution:");
  for (std::size_t i = 0; i < solution.size(); ++i) {
    RCLCPP_INFO(node->get_logger(), "  %s: %.6f", joint_names[i].c_str(), solution[i]);
  }

  rclcpp::shutdown();
  return 0;
}
