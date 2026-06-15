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
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "cartesian_path_demo", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  const bool execute = node->declare_parameter<bool>("execute", false);
  moveit::planning_interface::MoveGroupInterface move_group(node, kGroupName);
  move_group.setEndEffectorLink(kToolLink);

  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose(kToolLink).pose;
  waypoints.push_back(start_pose);

  geometry_msgs::msg::Pose target_pose = start_pose;
  target_pose.position.z += 0.03;
  target_pose.position.x += 0.03;
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  RCLCPP_INFO(node->get_logger(), "Cartesian path fraction: %.3f", fraction);

  if (execute && fraction > 0.9) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    const auto result = move_group.execute(plan);
    RCLCPP_INFO(node->get_logger(), "Execute result code: %d", result.val);
  }

  rclcpp::shutdown();
  return 0;
}
