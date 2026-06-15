#include <array>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

namespace
{
constexpr char kGroupName[] = "single_arm";
constexpr char kToolLink[] = "tool0";
const std::array<const char *, 6> kJointNames = {"J1", "J2", "J3", "J4", "J5", "J6"};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "fk_demo", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  std::vector<double> joint_positions =
    node->declare_parameter<std::vector<double>>("joint_positions", {0.0, -0.5, 0.8, 0.0, 0.6, 0.0});

  if (joint_positions.size() != kJointNames.size()) {
    RCLCPP_ERROR(node->get_logger(), "joint_positions must contain exactly 6 values.");
    rclcpp::shutdown();
    return 1;
  }

  moveit::planning_interface::MoveGroupInterface move_group(node, kGroupName);
  auto state = move_group.getCurrentState(5.0);
  if (!state) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get current robot state.");
    rclcpp::shutdown();
    return 1;
  }

  const auto * joint_model_group = state->getJointModelGroup(kGroupName);
  state->setJointGroupPositions(joint_model_group, joint_positions);
  state->update();

  const Eigen::Isometry3d & transform = state->getGlobalLinkTransform(kToolLink);
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
