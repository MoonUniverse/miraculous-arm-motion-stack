#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>

#include "arm_kinematics/moveit_demo_utils.hpp"

namespace
{
constexpr char kGroupName[] = "single_arm";
constexpr char kToolLink[] = "tool0";
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ik_demo");
  arm_kinematics::ensureMoveItRobotDescriptionParameters(node);

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = node->declare_parameter<double>("x", 0.020061);
  target_pose.position.y = node->declare_parameter<double>("y", 0.000397);
  target_pose.position.z = node->declare_parameter<double>("z", 0.549780);
  target_pose.orientation.x = node->declare_parameter<double>("qx", 0.500000);
  target_pose.orientation.y = node->declare_parameter<double>("qy", 0.500000);
  target_pose.orientation.z = node->declare_parameter<double>("qz", 0.500002);
  target_pose.orientation.w = node->declare_parameter<double>("qw", 0.499998);

  robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
  const moveit::core::RobotModelPtr robot_model = robot_model_loader.getModel();
  if (!robot_model) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load robot model.");
    rclcpp::shutdown();
    return 2;
  }

  moveit::core::RobotState state(robot_model);
  state.setToDefaultValues();
  const auto * joint_model_group = robot_model->getJointModelGroup(kGroupName);
  const bool found_ik = state.setFromIK(joint_model_group, target_pose, kToolLink, 0.1);
  if (!found_ik) {
    RCLCPP_ERROR(node->get_logger(), "IK failed for target pose.");
    rclcpp::shutdown();
    return 2;
  }

  std::vector<double> solution;
  state.copyJointGroupPositions(joint_model_group, solution);
  const auto & joint_names = joint_model_group->getVariableNames();
  RCLCPP_INFO(node->get_logger(), "IK solution:");
  for (std::size_t i = 0; i < solution.size(); ++i) {
    RCLCPP_INFO(node->get_logger(), "  %s: %.6f", joint_names[i].c_str(), solution[i]);
  }

  rclcpp::shutdown();
  return 0;
}
