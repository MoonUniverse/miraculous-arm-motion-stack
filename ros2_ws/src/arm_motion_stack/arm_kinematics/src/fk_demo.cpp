#include <array>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>

#include "arm_kinematics/moveit_demo_utils.hpp"

namespace
{
constexpr char kGroupName[] = "single_arm";
constexpr char kToolLink[] = "tool0";
const std::array<const char *, 6> kJointNames = {"J1", "J2", "J3", "J4", "J5", "J6"};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("fk_demo");
  arm_kinematics::ensureMoveItRobotDescriptionParameters(node);

  std::vector<double> joint_positions =
    node->declare_parameter<std::vector<double>>("joint_positions", {0.0, -0.5, 0.8, 0.0, 0.6, 0.0});

  if (joint_positions.size() != kJointNames.size()) {
    RCLCPP_ERROR(node->get_logger(), "joint_positions must contain exactly 6 values.");
    rclcpp::shutdown();
    return 1;
  }

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
  state.setJointGroupPositions(joint_model_group, joint_positions);
  state.update();

  const Eigen::Isometry3d & transform = state.getGlobalLinkTransform(kToolLink);
  const Eigen::Quaterniond quat(transform.rotation());

  RCLCPP_INFO(node->get_logger(), "FK for joints [J1..J6]:");
  for (std::size_t i = 0; i < kJointNames.size(); ++i) {
    RCLCPP_INFO(node->get_logger(), "  %s: %.6f", kJointNames[i], joint_positions[i]);
  }
  RCLCPP_INFO(
    node->get_logger(), "tool0 position: [%.6f, %.6f, %.6f]",
    transform.translation().x(), transform.translation().y(), transform.translation().z());
  RCLCPP_INFO(
    node->get_logger(), "tool0 orientation xyzw: [%.6f, %.6f, %.6f, %.6f]",
    quat.x(), quat.y(), quat.z(), quat.w());

  rclcpp::shutdown();
  return 0;
}
