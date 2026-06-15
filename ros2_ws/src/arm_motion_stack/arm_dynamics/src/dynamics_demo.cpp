#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "arm_dynamics/dynamics_model.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("dynamics_demo");
  const auto urdf_path = node->declare_parameter<std::string>("urdf_path", "");

  arm_dynamics::DynamicsModel model;
  if (!model.loadModelFromUrdf(urdf_path)) {
    RCLCPP_WARN(node->get_logger(), "Using placeholder dynamics without a URDF path.");
  }

  Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd dq = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd ddq = Eigen::VectorXd::Zero(6);

  RCLCPP_INFO_STREAM(node->get_logger(), "gravity: " << model.computeGravity(q).transpose());
  RCLCPP_INFO_STREAM(node->get_logger(), "mass matrix:\n" << model.computeMassMatrix(q));
  RCLCPP_INFO_STREAM(
    node->get_logger(),
    "inverse dynamics: " << model.computeInverseDynamics(q, dq, ddq).transpose());

  rclcpp::shutdown();
  return 0;
}
