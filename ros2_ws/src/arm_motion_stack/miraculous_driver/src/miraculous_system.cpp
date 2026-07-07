#include "miraculous_driver/miraculous_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace miraculous_driver
{

namespace
{
constexpr double kSeedWaitSec = 0.25;  // wait for the read thread cache on configure

bool expand_single_value(std::vector<double> & values, size_t size)
{
  if (values.size() == 1 && size > 1) {
    values.assign(size, values[0]);
  }
  return values.size() == size;
}

std::vector<int> default_joint_indices(size_t size)
{
  std::vector<int> out;
  out.reserve(size);
  for (size_t i = 0; i < size; ++i) {
    out.push_back(static_cast<int>(i));
  }
  return out;
}

bool validate_joint_indices(const std::vector<int> & indices, size_t max_size)
{
  std::vector<bool> seen(max_size, false);
  for (const int index : indices) {
    if (index < 0 || static_cast<size_t>(index) >= max_size || seen[static_cast<size_t>(index)]) {
      return false;
    }
    seen[static_cast<size_t>(index)] = true;
  }
  return true;
}

bool validate_limit_values(
  const std::vector<double> & values, size_t active_count, size_t all_count)
{
  return values.size() == 1 || values.size() == active_count || values.size() == all_count;
}

double limit_value_for_joint(
  const std::vector<double> & values, size_t active_i, size_t joint_index)
{
  if (values.size() == 1) {
    return values.front();
  }
  if (values.size() == kArmJoints) {
    return values[joint_index];
  }
  return values[active_i];
}
}

hardware_interface::CallbackReturn MiraculousSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.clear();
  joint_names_.reserve(info_.joints.size());
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MiraculousSystem"),
        "Joint '%s' must expose exactly one position command interface.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    const bool has_position_state = std::any_of(
      joint.state_interfaces.begin(), joint.state_interfaces.end(),
      [](const auto & interface) { return interface.name == hardware_interface::HW_IF_POSITION; });
    const bool has_velocity_state = std::any_of(
      joint.state_interfaces.begin(), joint.state_interfaces.end(),
      [](const auto & interface) { return interface.name == hardware_interface::HW_IF_VELOCITY; });
    if (!has_position_state || !has_velocity_state) {
      RCLCPP_FATAL(
        rclcpp::get_logger("MiraculousSystem"),
        "Joint '%s' must expose position and velocity state interfaces.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_names_.push_back(joint.name);
  }

  const size_t n = joint_names_.size();
  position_states_.assign(n, 0.0);
  velocity_states_.assign(n, 0.0);
  position_commands_.assign(n, 0.0);

  // Honor initial_value from the URDF state interface so command/state start aligned.
  for (size_t index = 0; index < info_.joints.size(); ++index) {
    for (const auto & interface : info_.joints[index].state_interfaces) {
      if (interface.name != hardware_interface::HW_IF_POSITION) {
        continue;
      }
      if (!interface.initial_value.empty()) {
        try {
          position_states_[index] = std::stod(interface.initial_value);
          position_commands_[index] = position_states_[index];
        } catch (const std::exception &) {
          RCLCPP_WARN(
            rclcpp::get_logger("MiraculousSystem"),
            "Ignoring invalid initial_value '%s' for joint '%s'.",
            interface.initial_value.c_str(), info_.joints[index].name.c_str());
        }
      }
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MiraculousSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  ArmConfig config;
  config.can_interface = parse_string_param("can_interface", "can0");
  config.baudrate = static_cast<CiaBaudrate_t>(parse_int_param("baudrate", 1000));
  config.sync_period_us = static_cast<uint32_t>(parse_int_param("sync_period_us", 0));
  config.read_rate_hz = parse_double_param("read_rate_hz", 100.0);
  config.state_poll_rate_hz = parse_double_param("state_poll_rate_hz", 0.0);

  const std::vector<int> default_node_ids = {1, 2, 3, 4, 5, 6};
  const std::vector<int> node_ids =
    parse_int_list_param("node_ids", default_node_ids);
  const std::vector<int> joint_indices = parse_int_list_param(
    "joint_indices", default_joint_indices(node_ids.size()));

  std::vector<double> position_min =
    parse_double_list_param("position_min", std::vector<double>(joint_names_.size(), 0.0));
  std::vector<double> position_max =
    parse_double_list_param("position_max", std::vector<double>(joint_names_.size(), 0.0));

  if (node_ids.empty() || node_ids.size() > joint_names_.size()) {
    RCLCPP_FATAL(
      rclcpp::get_logger("MiraculousSystem"),
      "node_ids must contain 1..%zu values.", joint_names_.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (joint_indices.size() != node_ids.size() ||
    !validate_joint_indices(joint_indices, joint_names_.size()))
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("MiraculousSystem"),
      "joint_indices must contain %zu unique values in range 0..%zu.",
      node_ids.size(), joint_names_.size() - 1);
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!validate_limit_values(position_min, node_ids.size(), joint_names_.size()) ||
    !validate_limit_values(position_max, node_ids.size(), joint_names_.size()))
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("MiraculousSystem"),
      "position_min and position_max must each contain 1, %zu, or %zu comma-separated values.",
      node_ids.size(), joint_names_.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < node_ids.size(); ++i) {
    const size_t joint_index = static_cast<size_t>(joint_indices[i]);
    JointConfig jc;
    jc.name = joint_names_[joint_index];
    jc.joint_index = joint_index;
    jc.node_id = static_cast<uint8_t>(node_ids[i]);
    jc.position_min = limit_value_for_joint(position_min, i, joint_index);
    jc.position_max = limit_value_for_joint(position_max, i, joint_index);
    config.joints.push_back(jc);
  }

  arm_ = std::make_unique<MiraculousArm>();
  if (!arm_->init(config)) {
    RCLCPP_FATAL(rclcpp::get_logger("MiraculousSystem"), "Failed to open motors.");
    arm_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }

  arm_->set_emcy_callback([](uint8_t node_id, uint16_t error_code, uint8_t error_reg) {
      RCLCPP_ERROR(
        rclcpp::get_logger("MiraculousSystem"),
        "EMCY from node %u: code=0x%04X reg=0x%02X", node_id, error_code, error_reg);
    });

  // Wait briefly for the background read thread to populate the encoder cache,
  // then seed commands from the real positions to avoid a jump on activate.
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(kSeedWaitSec));
  std::array<double, kArmJoints> pos{};
  while (std::chrono::steady_clock::now() < deadline) {
    if (arm_->get_positions_rad(pos)) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (arm_->get_positions_rad(pos)) {
    const size_t n = joint_names_.size();
    for (size_t i = 0; i < n && i < kArmJoints; ++i) {
      position_states_[i] = pos[i];
      position_commands_[i] = pos[i];
    }
    RCLCPP_INFO(
      rclcpp::get_logger("MiraculousSystem"),
      "Seeded initial positions from encoders.");
  }

  RCLCPP_INFO(
    rclcpp::get_logger("MiraculousSystem"),
    "Configured: can=%s active_joints=%zu total_joints=%zu sync_period_us=%u "
    "read_rate_hz=%.1f state_poll_rate_hz=%.1f",
    config.can_interface.c_str(), config.joints.size(), joint_names_.size(),
    config.sync_period_us, config.read_rate_hz, config.state_poll_rate_hz);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MiraculousSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!arm_) {
    RCLCPP_FATAL(rclcpp::get_logger("MiraculousSystem"), "on_activate called before on_configure.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!arm_->enable_csp()) {
    RCLCPP_FATAL(rclcpp::get_logger("MiraculousSystem"), "enable_csp failed.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  active_ = true;

  RCLCPP_INFO(rclcpp::get_logger("MiraculousSystem"), "Activated (CSP enabled).");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MiraculousSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  if (arm_) {
    arm_->disable();
  }
  RCLCPP_INFO(rclcpp::get_logger("MiraculousSystem"), "Deactivated (motors disabled).");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MiraculousSystem::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  if (arm_) {
    arm_->shutdown();
    arm_.reset();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MiraculousSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(joint_names_.size() * 2);
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &position_states_[i]);
    interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> MiraculousSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &position_commands_[i]);
  }
  return interfaces;
}

hardware_interface::return_type MiraculousSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!arm_) {
    return hardware_interface::return_type::OK;
  }
  std::array<double, kArmJoints> pos{};
  std::array<double, kArmJoints> vel{};
  if (arm_->get_positions_rad(pos) && arm_->get_velocities_rad(vel)) {
    for (size_t i = 0; i < joint_names_.size() && i < kArmJoints; ++i) {
      position_states_[i] = pos[i];
      velocity_states_[i] = vel[i];
    }
  }
  if (arm_->has_fault()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("MiraculousSystem"),
      "Motor fault detected. Use fault_reset before reactivating.");
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MiraculousSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_ || !arm_) {
    return hardware_interface::return_type::OK;
  }
  std::array<double, kArmJoints> targets{};
  for (size_t i = 0; i < joint_names_.size() && i < kArmJoints; ++i) {
    targets[i] = position_commands_[i];
  }
  arm_->set_targets_rad(targets);
  return hardware_interface::return_type::OK;
}

// ============================ parameter helpers ===========================

double MiraculousSystem::parse_double_param(const std::string & name, double default_value) const
{
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end()) {
    return default_value;
  }
  try {
    return std::stod(it->second);
  } catch (const std::exception &) {
    return default_value;
  }
}

std::string MiraculousSystem::parse_string_param(
  const std::string & name, const std::string & default_value) const
{
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end() || it->second.empty()) {
    return default_value;
  }
  return it->second;
}

int MiraculousSystem::parse_int_param(const std::string & name, int default_value) const
{
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end()) {
    return default_value;
  }
  try {
    return std::stoi(it->second);
  } catch (const std::exception &) {
    return default_value;
  }
}

std::vector<int> MiraculousSystem::parse_int_list_param(
  const std::string & name, const std::vector<int> & default_value) const
{
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end() || it->second.empty()) {
    return default_value;
  }
  std::vector<int> result;
  std::stringstream ss(it->second);
  std::string token;
  while (std::getline(ss, token, ',')) {
    try {
      result.push_back(std::stoi(token));
    } catch (const std::exception &) {
      // skip malformed token
    }
  }
  return result.empty() ? default_value : result;
}

std::vector<double> MiraculousSystem::parse_double_list_param(
  const std::string & name, const std::vector<double> & default_value) const
{
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end() || it->second.empty()) {
    return default_value;
  }
  std::vector<double> result;
  std::stringstream ss(it->second);
  std::string token;
  while (std::getline(ss, token, ',')) {
    try {
      result.push_back(std::stod(token));
    } catch (const std::exception &) {
      // skip malformed token
    }
  }
  return result.empty() ? default_value : result;
}

}  // namespace miraculous_driver

PLUGINLIB_EXPORT_CLASS(miraculous_driver::MiraculousSystem, hardware_interface::SystemInterface)
