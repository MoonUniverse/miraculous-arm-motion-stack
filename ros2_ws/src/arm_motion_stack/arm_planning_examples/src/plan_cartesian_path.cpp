#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
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
    "plan_cartesian_path", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  const bool execute = node->declare_parameter<bool>("execute", false);
  const double dx = node->declare_parameter<double>("dx", 0.03);
  const double dz = node->declare_parameter<double>("dz", 0.03);

  moveit::planning_interface::MoveGroupInterface move_group(node, kGroupName);
  move_group.setEndEffectorLink(kToolLink);

  std::vector<geometry_msgs::msg::Pose> waypoints;
  auto pose = move_group.getCurrentPose(kToolLink).pose;
  waypoints.push_back(pose);
  pose.position.x += dx;
  pose.position.z += dz;
  waypoints.push_back(pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  RCLCPP_INFO(node->get_logger(), "Cartesian path fraction: %.3f", fraction);

  if (fraction > 0.9 && execute) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    const auto result = move_group.execute(plan);
    RCLCPP_INFO(node->get_logger(), "Execute result code: %d", result.val);
  }

  rclcpp::shutdown();
  return fraction > 0.0 ? 0 : 2;
}
