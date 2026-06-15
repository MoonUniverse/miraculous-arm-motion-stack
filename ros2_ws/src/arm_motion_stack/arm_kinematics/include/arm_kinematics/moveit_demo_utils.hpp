#pragma once

#include <array>
#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

namespace arm_kinematics
{

inline std::string shellQuote(const std::string & value)
{
  std::string quoted = "'";
  for (const char character : value) {
    if (character == '\'') {
      quoted += "'\\''";
    } else {
      quoted += character;
    }
  }
  quoted += "'";
  return quoted;
}

inline std::string readTextFile(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("Failed to open file: " + path);
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

inline std::string runCommand(const std::string & command)
{
  std::array<char, 4096> buffer{};
  std::string output;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command.c_str(), "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("Failed to run command: " + command);
  }
  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
    output += buffer.data();
  }
  if (output.empty()) {
    throw std::runtime_error("Command produced empty output: " + command);
  }
  return output;
}

inline void ensureMoveItRobotDescriptionParameters(const rclcpp::Node::SharedPtr & node)
{
  const auto description_share = ament_index_cpp::get_package_share_directory("arm_description");
  const auto moveit_share = ament_index_cpp::get_package_share_directory("arm_moveit_config");
  const auto xacro_path = description_share + "/urdf/arm.urdf.xacro";
  const auto srdf_path = moveit_share + "/srdf/arm.srdf";

  auto robot_description = node->declare_parameter<std::string>("robot_description", "");
  if (robot_description.empty()) {
    robot_description = runCommand("xacro " + shellQuote(xacro_path) + " hardware_type:=fake");
    node->set_parameter(rclcpp::Parameter("robot_description", robot_description));
  }

  auto robot_description_semantic =
    node->declare_parameter<std::string>("robot_description_semantic", "");
  if (robot_description_semantic.empty()) {
    robot_description_semantic = readTextFile(srdf_path);
    node->set_parameter(
      rclcpp::Parameter("robot_description_semantic", robot_description_semantic));
  }

  node->declare_parameter<std::string>(
    "robot_description_kinematics.single_arm.kinematics_solver",
    "kdl_kinematics_plugin/KDLKinematicsPlugin");
  node->declare_parameter<double>(
    "robot_description_kinematics.single_arm.kinematics_solver_search_resolution", 0.005);
  node->declare_parameter<double>(
    "robot_description_kinematics.single_arm.kinematics_solver_timeout", 0.05);
  node->declare_parameter<int>(
    "robot_description_kinematics.single_arm.kinematics_solver_attempts", 3);
}

}  // namespace arm_kinematics
