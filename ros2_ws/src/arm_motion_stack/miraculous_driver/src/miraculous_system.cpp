#include "miraculous_driver/miraculous_system.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace miraculous_driver
{

namespace
{
constexpr double kSeedWaitSec = 0.25;

std::string trim(const std::string & input)
{
  const auto first = std::find_if_not(
    input.begin(), input.end(),
    [](unsigned char ch) {return std::isspace(ch) != 0;});
  const auto last = std::find_if_not(
    input.rbegin(), input.rend(),
    [](unsigned char ch) {return std::isspace(ch) != 0;}).base();
  return first < last ? std::string(first, last) : std::string();
}

template<typename T, typename Parse>
T parse_scalar_strict(const std::string & text, const std::string & name, Parse parse)
{
  const std::string value = trim(text);
  if (value.empty()) {
    throw std::invalid_argument(name + " must not be empty");
  }
  size_t consumed = 0;
  T result = parse(value, &consumed);
  if (consumed != value.size()) {
    throw std::invalid_argument(name + " contains trailing characters: " + value);
  }
  return result;
}

std::vector<int> default_joint_indices(size_t size)
{
  std::vector<int> result;
  result.reserve(size);
  for (size_t index = 0; index < size; ++index) {
    result.push_back(static_cast<int>(index));
  }
  return result;
}

bool validate_limit_vector_size(
  const std::vector<double> & values, size_t active_count, size_t all_count)
{
  return values.size() == 1 || values.size() == active_count ||
         values.size() == all_count;
}

double limit_for_joint(
  const std::vector<double> & values, size_t active_index, size_t joint_index)
{
  if (values.size() == 1) {
    return values.front();
  }
  return values.size() == kArmJoints ? values[joint_index] : values[active_index];
}
}  // namespace

MiraculousSystem::MiraculousSystem()
: MiraculousSystem([]() {return std::make_unique<MiraculousArm>();})
{
}

MiraculousSystem::MiraculousSystem(ArmFactory arm_factory)
: arm_factory_(std::move(arm_factory))
{
}

hardware_interface::CallbackReturn MiraculousSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.joints.size() != kArmJoints) {
    RCLCPP_FATAL(
      rclcpp::get_logger("MiraculousSystem"),
      "The hardware model must expose exactly %zu joints.", kArmJoints);
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.clear();
  joint_names_.reserve(kArmJoints);
  for (size_t index = 0; index < info_.joints.size(); ++index) {
    const auto & joint = info_.joints[index];
    const std::string expected_name = "J" + std::to_string(index + 1);
    if (joint.name != expected_name) {
      RCLCPP_FATAL(
        rclcpp::get_logger("MiraculousSystem"),
        "Joint %zu must be named '%s', got '%s'.", index, expected_name.c_str(),
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
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
      [](const auto & interface) {
        return interface.name == hardware_interface::HW_IF_POSITION;
      });
    const bool has_velocity_state = std::any_of(
      joint.state_interfaces.begin(), joint.state_interfaces.end(),
      [](const auto & interface) {
        return interface.name == hardware_interface::HW_IF_VELOCITY;
      });
    if (!has_position_state || !has_velocity_state) {
      RCLCPP_FATAL(
        rclcpp::get_logger("MiraculousSystem"),
        "Joint '%s' must expose position and velocity state interfaces.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_names_.push_back(joint.name);
  }

  position_states_.assign(kArmJoints, 0.0);
  velocity_states_.assign(kArmJoints, 0.0);
  position_commands_.assign(kArmJoints, 0.0);
  for (size_t index = 0; index < info_.joints.size(); ++index) {
    for (const auto & interface : info_.joints[index].state_interfaces) {
      if (interface.name != hardware_interface::HW_IF_POSITION ||
        interface.initial_value.empty())
      {
        continue;
      }
      try {
        const double value = parse_scalar_strict<double>(
          interface.initial_value, "initial_value",
          [](const std::string & input, size_t * consumed) {
            return std::stod(input, consumed);
          });
        if (!std::isfinite(value)) {
          throw std::invalid_argument("initial_value must be finite");
        }
        position_states_[index] = value;
        position_commands_[index] = value;
      } catch (const std::exception & error) {
        RCLCPP_FATAL(
          rclcpp::get_logger("MiraculousSystem"),
          "Invalid initial_value for joint '%s': %s",
          info_.joints[index].name.c_str(), error.what());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MiraculousSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (fault_latched_) {
    RCLCPP_FATAL(
      rclcpp::get_logger("MiraculousSystem"),
      "A hardware fault is latched. Restart controller_manager after external "
      "drive reset; in-process reconfiguration is intentionally blocked.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  ArmConfig config;
  try {
    config.can_interface = parse_string_param("can_interface", "can1");
    const int baudrate = parse_int_param("baudrate", 0);
    const int sync_period_us = parse_int_param("sync_period_us", 0);
    const int encoder_bw = parse_int_param("encoder_bw", 19);
    const int manual_timeout = parse_int_param("manual_feedback_timeout_ms", 15);
    const int stale_timeout = parse_int_param("feedback_stale_timeout_ms", 30);
    const int following_error_cycles = parse_int_param("following_error_cycles", 0);
    config.read_rate_hz = parse_double_param("read_rate_hz", 50.0);
    config.state_poll_rate_hz = parse_double_param("state_poll_rate_hz", 0.0);
    config.reduction_ratio = parse_double_param("reduction_ratio", 100.0);
    config.enable_emcy_monitor = parse_bool_param("enable_emcy_monitor", true);
    max_command_step_rad_ = parse_double_param("max_command_step_rad", 0.0);
    max_following_error_rad_ = parse_double_param("max_following_error_rad", 0.0);
    const bool require_full_arm = parse_bool_param("require_full_arm", false);
    const bool require_limits = parse_bool_param("require_position_limits", false);

    if (baudrate < 0 || sync_period_us < 0 || encoder_bw < 1 || encoder_bw > 31 ||
      manual_timeout <= 0 || stale_timeout <= manual_timeout ||
      !std::isfinite(config.read_rate_hz) || config.read_rate_hz <= 0.0 ||
      !std::isfinite(config.state_poll_rate_hz) || config.state_poll_rate_hz < 0.0 ||
      !std::isfinite(config.reduction_ratio) || config.reduction_ratio <= 0.0 ||
      !std::isfinite(max_command_step_rad_) || max_command_step_rad_ < 0.0 ||
      !std::isfinite(max_following_error_rad_) || max_following_error_rad_ < 0.0 ||
      following_error_cycles < 0 ||
      ((max_following_error_rad_ > 0.0) != (following_error_cycles > 0)))
    {
      throw std::invalid_argument(
              "invalid hardware rates, timeouts, encoder, baudrate, or reduction ratio");
    }
    if (require_full_arm && sync_period_us != 0) {
      throw std::invalid_argument(
              "require_full_arm=true requires sync_period_us=0 so six-axis target "
              "writes remain under one driver-owned Manual SYNC transaction");
    }
    if (require_full_arm && !config.enable_emcy_monitor) {
      throw std::invalid_argument(
              "require_full_arm=true requires enable_emcy_monitor=true");
    }
    if (require_full_arm &&
      (max_command_step_rad_ <= 0.0 || max_following_error_rad_ <= 0.0 ||
      following_error_cycles == 0))
    {
      throw std::invalid_argument(
              "require_full_arm=true requires positive command-step and following-error "
              "watchdogs");
    }
    following_error_cycles_ = static_cast<size_t>(following_error_cycles);
    config.baudrate = static_cast<CiaBaudrate_t>(baudrate);
    config.sync_period_us = static_cast<uint32_t>(sync_period_us);
    config.encoder_bw = static_cast<uint8_t>(encoder_bw);
    config.manual_feedback_timeout_ms = static_cast<uint32_t>(manual_timeout);
    config.max_command_step_rad = max_command_step_rad_;
    config.max_following_error_rad = max_following_error_rad_;
    config.following_error_cycles = static_cast<uint32_t>(following_error_cycles);
    feedback_stale_timeout_ = std::chrono::milliseconds(stale_timeout);

    const std::vector<int> node_ids = parse_int_list_param(
      "node_ids", {1, 2, 3, 4, 5, 6});
    const std::vector<int> joint_indices = parse_int_list_param(
      "joint_indices", default_joint_indices(node_ids.size()));
    const std::vector<double> position_min = parse_double_list_param(
      "position_min", std::vector<double>(kArmJoints, 0.0));
    const std::vector<double> position_max = parse_double_list_param(
      "position_max", std::vector<double>(kArmJoints, 0.0));

    if (node_ids.empty() || node_ids.size() > kArmJoints ||
      joint_indices.size() != node_ids.size() ||
      !validate_limit_vector_size(position_min, node_ids.size(), kArmJoints) ||
      !validate_limit_vector_size(position_max, node_ids.size(), kArmJoints))
    {
      throw std::invalid_argument("node, index, or limit list has the wrong length");
    }
    std::set<int> unique_nodes;
    std::set<int> unique_indices;
    for (size_t active_index = 0; active_index < node_ids.size(); ++active_index) {
      const int node_id = node_ids[active_index];
      const int joint_index = joint_indices[active_index];
      if (node_id < 1 || node_id > 127 || joint_index < 0 ||
        joint_index >= static_cast<int>(kArmJoints) ||
        !unique_nodes.insert(node_id).second ||
        !unique_indices.insert(joint_index).second)
      {
        throw std::invalid_argument("node_ids and joint_indices must be unique and in range");
      }
    }
    if (require_full_arm &&
      (node_ids.size() != kArmJoints ||
      unique_indices != std::set<int>({0, 1, 2, 3, 4, 5})))
    {
      throw std::invalid_argument(
              "require_full_arm=true requires one mapping for each of J1..J6");
    }

    configured_joints_.fill(false);
    position_min_.fill(0.0);
    position_max_.fill(0.0);
    for (size_t active_index = 0; active_index < node_ids.size(); ++active_index) {
      const size_t joint_index = static_cast<size_t>(joint_indices[active_index]);
      const double lower = limit_for_joint(position_min, active_index, joint_index);
      const double upper = limit_for_joint(position_max, active_index, joint_index);
      if (!std::isfinite(lower) || !std::isfinite(upper) ||
        (require_limits && lower >= upper))
      {
        throw std::invalid_argument(
                "position limits must be finite and strictly ordered when required");
      }
      JointConfig joint;
      joint.name = joint_names_[joint_index];
      joint.joint_index = joint_index;
      joint.node_id = static_cast<uint8_t>(node_ids[active_index]);
      joint.position_min = lower;
      joint.position_max = upper;
      config.joints.push_back(joint);
      configured_joints_[joint_index] = true;
      position_min_[joint_index] = lower;
      position_max_[joint_index] = upper;
    }

    if (!arm_factory_) {
      throw std::runtime_error("arm factory is empty");
    }
    arm_ = arm_factory_();
    if (!arm_ || !arm_->init(config)) {
      shutdown_arm();
      throw std::runtime_error("failed to initialize motors");
    }
    emcy_latched_.store(false, std::memory_order_release);
    arm_->set_emcy_callback(
      [this](uint8_t node_id, uint16_t error_code, uint8_t error_reg) {
        emcy_latched_.store(true, std::memory_order_release);
        RCLCPP_ERROR(
          rclcpp::get_logger("MiraculousSystem"),
          "EMCY latched from node %u: code=0x%04X reg=0x%02X",
          node_id, error_code, error_reg);
      });

    FeedbackSnapshot snapshot;
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(kSeedWaitSec));
    while (std::chrono::steady_clock::now() < deadline &&
      !arm_->get_feedback_snapshot(snapshot))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::string unsafe_reason;
    if (!arm_->get_feedback_snapshot(snapshot) ||
      !snapshot_is_safe(snapshot, true, unsafe_reason))
    {
      shutdown_arm();
      throw std::runtime_error(
              "safe encoder seed unavailable: " +
              (unsafe_reason.empty() ? std::string("no complete feedback") : unsafe_reason));
    }
    apply_snapshot(snapshot);
    last_sent_commands_ = snapshot.positions_rad;
    last_command_valid_ = true;
    following_error_streak_ = 0;
    last_following_sequence_ = snapshot.sequence;
    stop_issued_ = false;
    active_ = false;

    RCLCPP_INFO(
      rclcpp::get_logger("MiraculousSystem"),
      "Configured inactive: can=%s mapped_joints=%zu stale_timeout_ms=%d. "
      "No drive was enabled.",
      config.can_interface.c_str(), config.joints.size(), stale_timeout);
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("MiraculousSystem"),
      "Configuration rejected: %s", error.what());
    shutdown_arm();
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn MiraculousSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!arm_ || fault_latched_ || emcy_latched_.load(std::memory_order_acquire)) {
    RCLCPP_FATAL(
      rclcpp::get_logger("MiraculousSystem"),
      "Activation rejected: hardware is missing or a fault is latched.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  FeedbackSnapshot before_enable;
  std::string unsafe_reason;
  if (!arm_->get_feedback_snapshot(before_enable) ||
    !snapshot_is_safe(before_enable, true, unsafe_reason))
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("MiraculousSystem"),
      "Activation rejected before enable: %s", unsafe_reason.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  apply_snapshot(before_enable);

  if (!arm_->enable_csp()) {
    fail_safe_stop("enable_csp transaction failed");
    return hardware_interface::CallbackReturn::ERROR;
  }

  FeedbackSnapshot after_enable;
  if (!arm_->get_feedback_snapshot(after_enable) ||
    !snapshot_is_safe(after_enable, true, unsafe_reason))
  {
    fail_safe_stop("unsafe feedback after enable: " + unsafe_reason);
    return hardware_interface::CallbackReturn::ERROR;
  }
  apply_snapshot(after_enable);
  last_sent_commands_ = after_enable.positions_rad;
  last_command_valid_ = true;
  following_error_streak_ = 0;
  last_following_sequence_ = after_enable.sequence;
  active_ = true;
  RCLCPP_INFO(
    rclcpp::get_logger("MiraculousSystem"),
    "Activated in CSP with commands seeded from fresh encoder feedback.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MiraculousSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  const bool was_active = active_;
  active_ = false;
  last_command_valid_ = false;
  following_error_streak_ = 0;
  if (arm_ && was_active && !arm_->disable()) {
    fail_safe_stop("drive disable failed");
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("MiraculousSystem"), "Deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MiraculousSystem::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  shutdown_arm();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MiraculousSystem::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  if (active_ && arm_ && !stop_issued_) {
    arm_->quick_stop();
  }
  active_ = false;
  shutdown_arm();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MiraculousSystem::on_error(
  const rclcpp_lifecycle::State &)
{
  if (active_ || fault_latched_) {
    fail_safe_stop("ros2_control entered the hardware error transition");
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("MiraculousSystem"),
      "Hardware error transition occurred before activation. Shutting down "
      "the SDK without quick-stop or a runtime fault latch because no drive "
      "was enabled.");
  }
  shutdown_arm();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MiraculousSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(joint_names_.size() * 2);
  for (size_t index = 0; index < joint_names_.size(); ++index) {
    interfaces.emplace_back(
      joint_names_[index], hardware_interface::HW_IF_POSITION,
      &position_states_[index]);
    interfaces.emplace_back(
      joint_names_[index], hardware_interface::HW_IF_VELOCITY,
      &velocity_states_[index]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
MiraculousSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(joint_names_.size());
  for (size_t index = 0; index < joint_names_.size(); ++index) {
    interfaces.emplace_back(
      joint_names_[index], hardware_interface::HW_IF_POSITION,
      &position_commands_[index]);
  }
  return interfaces;
}

hardware_interface::return_type MiraculousSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!arm_) {
    return active_ ? hardware_interface::return_type::ERROR :
           hardware_interface::return_type::OK;
  }
  if (emcy_latched_.load(std::memory_order_acquire) || arm_->has_fault()) {
    fail_safe_stop("motor fault or EMCY detected");
    return hardware_interface::return_type::ERROR;
  }

  FeedbackSnapshot snapshot;
  std::string unsafe_reason;
  if (!arm_->get_feedback_snapshot(snapshot) ||
    !snapshot_is_safe(snapshot, active_, unsafe_reason))
  {
    if (active_) {
      fail_safe_stop("feedback rejected: " + unsafe_reason);
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }
  if (active_ && !following_error_is_safe(snapshot, unsafe_reason)) {
    fail_safe_stop("feedback rejected: " + unsafe_reason);
    return hardware_interface::return_type::ERROR;
  }
  apply_snapshot(snapshot);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MiraculousSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_ || !arm_) {
    return hardware_interface::return_type::OK;
  }
  if (emcy_latched_.load(std::memory_order_acquire) || arm_->has_fault()) {
    fail_safe_stop("fault detected before command write");
    return hardware_interface::return_type::ERROR;
  }

  std::array<double, kArmJoints> targets{};
  for (size_t index = 0; index < kArmJoints; ++index) {
    if (!std::isfinite(position_commands_[index])) {
      fail_safe_stop("non-finite position command for " + joint_names_[index]);
      return hardware_interface::return_type::ERROR;
    }
    if (configured_joints_[index] &&
      position_max_[index] > position_min_[index] &&
      (position_commands_[index] < position_min_[index] ||
      position_commands_[index] > position_max_[index]))
    {
      fail_safe_stop("out-of-limit position command for " + joint_names_[index]);
      return hardware_interface::return_type::ERROR;
    }
    if (configured_joints_[index] && last_command_valid_ &&
      max_command_step_rad_ > 0.0 &&
      std::abs(position_commands_[index] - last_sent_commands_[index]) >
      max_command_step_rad_)
    {
      fail_safe_stop("single-cycle command step exceeded for " + joint_names_[index]);
      return hardware_interface::return_type::ERROR;
    }
    targets[index] = position_commands_[index];
  }
  if (!arm_->set_targets_rad(targets)) {
    fail_safe_stop("target write or fresh-feedback transaction failed");
    return hardware_interface::return_type::ERROR;
  }
  last_sent_commands_ = targets;
  last_command_valid_ = true;
  return hardware_interface::return_type::OK;
}

bool MiraculousSystem::snapshot_is_safe(
  const FeedbackSnapshot & snapshot, bool require_fresh,
  std::string & reason) const
{
  if (snapshot.sequence == 0) {
    reason = "feedback sequence is zero";
    return false;
  }
  const auto now = std::chrono::steady_clock::now();
  if (snapshot.stamp.time_since_epoch().count() <= 0 || snapshot.stamp > now) {
    reason = "feedback timestamp is invalid";
    return false;
  }
  if (require_fresh && now - snapshot.stamp > feedback_stale_timeout_) {
    reason = "feedback exceeded stale watchdog";
    return false;
  }
  for (size_t index = 0; index < kArmJoints; ++index) {
    if (!configured_joints_[index]) {
      continue;
    }
    if (!std::isfinite(snapshot.positions_rad[index]) ||
      !std::isfinite(snapshot.velocities_rad_s[index]))
    {
      reason = "non-finite feedback for " + joint_names_[index];
      return false;
    }
    if (position_max_[index] > position_min_[index] &&
      (snapshot.positions_rad[index] < position_min_[index] ||
      snapshot.positions_rad[index] > position_max_[index]))
    {
      reason = "feedback outside configured position limits for " + joint_names_[index];
      return false;
    }
  }
  return true;
}

bool MiraculousSystem::following_error_is_safe(
  const FeedbackSnapshot & snapshot, std::string & reason)
{
  if (!last_command_valid_ || max_following_error_rad_ <= 0.0 ||
    following_error_cycles_ == 0 || snapshot.sequence == last_following_sequence_)
  {
    return true;
  }
  last_following_sequence_ = snapshot.sequence;

  double worst_error = 0.0;
  size_t worst_joint = 0;
  for (size_t index = 0; index < kArmJoints; ++index) {
    if (!configured_joints_[index]) {
      continue;
    }
    const double error = std::abs(last_sent_commands_[index] - snapshot.positions_rad[index]);
    if (error > worst_error) {
      worst_error = error;
      worst_joint = index;
    }
  }

  if (worst_error <= max_following_error_rad_) {
    following_error_streak_ = 0;
    return true;
  }
  ++following_error_streak_;
  if (following_error_streak_ < following_error_cycles_) {
    return true;
  }

  std::ostringstream message;
  message << "following error exceeded for " << joint_names_[worst_joint]
          << ": " << worst_error << " rad > " << max_following_error_rad_
          << " rad for " << following_error_streak_ << " fresh cycles";
  reason = message.str();
  return false;
}

void MiraculousSystem::apply_snapshot(const FeedbackSnapshot & snapshot)
{
  for (size_t index = 0; index < kArmJoints; ++index) {
    if (!configured_joints_[index]) {
      continue;
    }
    position_states_[index] = snapshot.positions_rad[index];
    velocity_states_[index] = snapshot.velocities_rad_s[index];
    position_commands_[index] = snapshot.positions_rad[index];
  }
}

void MiraculousSystem::fail_safe_stop(const std::string & reason)
{
  active_ = false;
  last_command_valid_ = false;
  fault_latched_ = true;
  if (!stop_issued_) {
    stop_issued_ = true;
    const bool stopped = arm_ && arm_->quick_stop();
    RCLCPP_ERROR(
      rclcpp::get_logger("MiraculousSystem"),
      "FAULT LATCHED: %s. Verified quick-stop result: %s. External drive reset "
      "and controller_manager restart are required.",
      reason.c_str(), stopped ? "success" : "failure");
  }
}

void MiraculousSystem::shutdown_arm()
{
  if (arm_) {
    arm_->set_emcy_callback({});
    arm_->shutdown();
    arm_.reset();
  }
}

double MiraculousSystem::parse_double_param(
  const std::string & name, double default_value) const
{
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end()) {
    return default_value;
  }
  const double value = parse_scalar_strict<double>(
    it->second, name,
    [](const std::string & input, size_t * consumed) {
      return std::stod(input, consumed);
    });
  if (!std::isfinite(value)) {
    throw std::invalid_argument(name + " must be finite");
  }
  return value;
}

std::string MiraculousSystem::parse_string_param(
  const std::string & name, const std::string & default_value) const
{
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end()) {
    return default_value;
  }
  const std::string value = trim(it->second);
  if (value.empty() || value.find(',') != std::string::npos ||
    std::any_of(
      value.begin(), value.end(),
      [](unsigned char character) {return std::isspace(character) != 0;}))
  {
    throw std::invalid_argument(
            name + " must be non-empty and contain no whitespace or comma");
  }
  return value;
}

int MiraculousSystem::parse_int_param(
  const std::string & name, int default_value) const
{
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end()) {
    return default_value;
  }
  return parse_scalar_strict<int>(
    it->second, name,
    [](const std::string & input, size_t * consumed) {
      return std::stoi(input, consumed);
    });
}

bool MiraculousSystem::parse_bool_param(
  const std::string & name, bool default_value) const
{
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end()) {
    return default_value;
  }
  std::string value = trim(it->second);
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char character) {return static_cast<char>(std::tolower(character));});
  if (value == "true" || value == "1") {
    return true;
  }
  if (value == "false" || value == "0") {
    return false;
  }
  throw std::invalid_argument(name + " must be true, false, 1, or 0");
}

std::vector<int> MiraculousSystem::parse_int_list_param(
  const std::string & name, const std::vector<int> & default_value) const
{
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end()) {
    return default_value;
  }
  std::vector<int> result;
  std::stringstream stream(it->second);
  std::string token;
  while (std::getline(stream, token, ',')) {
    result.push_back(parse_scalar_strict<int>(
        token, name,
        [](const std::string & input, size_t * consumed) {
          return std::stoi(input, consumed);
        }));
  }
  if (result.empty() || (!it->second.empty() && it->second.back() == ',')) {
    throw std::invalid_argument(name + " contains an empty list item");
  }
  return result;
}

std::vector<double> MiraculousSystem::parse_double_list_param(
  const std::string & name, const std::vector<double> & default_value) const
{
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end()) {
    return default_value;
  }
  std::vector<double> result;
  std::stringstream stream(it->second);
  std::string token;
  while (std::getline(stream, token, ',')) {
    const double value = parse_scalar_strict<double>(
      token, name,
      [](const std::string & input, size_t * consumed) {
        return std::stod(input, consumed);
      });
    if (!std::isfinite(value)) {
      throw std::invalid_argument(name + " must contain only finite values");
    }
    result.push_back(value);
  }
  if (result.empty() || (!it->second.empty() && it->second.back() == ',')) {
    throw std::invalid_argument(name + " contains an empty list item");
  }
  return result;
}

}  // namespace miraculous_driver

PLUGINLIB_EXPORT_CLASS(miraculous_driver::MiraculousSystem, hardware_interface::SystemInterface)
