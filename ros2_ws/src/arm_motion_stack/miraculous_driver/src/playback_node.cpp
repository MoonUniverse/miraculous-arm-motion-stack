#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "miraculous_driver/miraculous_arm.hpp"
#include "playback_csv_header.hpp"
#include "strict_parameter_lists.hpp"

using miraculous_driver::ArmConfig;
using miraculous_driver::CsvHeaderFormat;
using miraculous_driver::JointConfig;
using miraculous_driver::MiraculousArm;
using miraculous_driver::classify_playback_csv_header;
using miraculous_driver::kArmJoints;

namespace
{
struct TrajectoryPoint
{
  double t = 0.0;
  uint64_t sample_index = 0;
  std::array<double, miraculous_driver::kArmJoints> pos{};
};

struct LoadedTrajectory
{
  std::vector<TrajectoryPoint> points;
  std::array<bool, miraculous_driver::kArmJoints> joint_mask{};
  bool is_v2 = false;
};

std::vector<int> default_joint_indices(size_t size)
{
  std::vector<int> out;
  out.reserve(size);
  for (size_t i = 0; i < size; ++i) {
    out.push_back(static_cast<int>(i));
  }
  return out;
}

bool validate_joint_indices(const std::vector<int> & indices)
{
  std::array<bool, miraculous_driver::kArmJoints> seen{};
  for (const int index : indices) {
    if (index < 0 ||
      static_cast<size_t>(index) >= miraculous_driver::kArmJoints ||
      seen[static_cast<size_t>(index)])
    {
      return false;
    }
    seen[static_cast<size_t>(index)] = true;
  }
  return true;
}

bool validate_node_ids(const std::vector<int> & node_ids)
{
  std::array<bool, 128> seen{};
  for (const int node_id : node_ids) {
    if (node_id < 1 || node_id > 127 || seen[static_cast<size_t>(node_id)]) {
      return false;
    }
    seen[static_cast<size_t>(node_id)] = true;
  }
  return true;
}

bool valid_limit_size(const std::vector<double> & values, size_t active_count)
{
  return values.size() == 1 || values.size() == active_count ||
         values.size() == miraculous_driver::kArmJoints;
}

double limit_value_for_joint(
  const std::vector<double> & values, size_t active_i, size_t joint_index)
{
  if (values.size() == 1) {
    return values.front();
  }
  if (values.size() == miraculous_driver::kArmJoints) {
    return values[joint_index];
  }
  return values[active_i];
}

std::vector<std::string> split_csv_line(std::string line)
{
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
  std::vector<std::string> fields;
  std::stringstream ss(line);
  std::string field;
  while (std::getline(ss, field, ',')) {
    fields.push_back(field);
  }
  if (!line.empty() && line.back() == ',') {
    fields.emplace_back();
  }
  return fields;
}

bool parse_finite_double(const std::string & text, double & value)
{
  try {
    size_t used = 0;
    value = std::stod(text, &used);
    return used == text.size() && std::isfinite(value);
  } catch (...) {
    return false;
  }
}

bool parse_uint64(const std::string & text, uint64_t & value)
{
  if (text.empty() || text.front() == '-') {
    return false;
  }
  try {
    size_t used = 0;
    value = std::stoull(text, &used);
    return used == text.size();
  } catch (...) {
    return false;
  }
}

bool parse_joint_name(const std::string & name, size_t & joint_index)
{
  if (name.size() != 2 || name[0] != 'J' || name[1] < '1' || name[1] > '6') {
    return false;
  }
  joint_index = static_cast<size_t>(name[1] - '1');
  return true;
}

bool load_csv(
  const std::string & path, LoadedTrajectory & trajectory,
  std::string & error)
{
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    error = "cannot open file";
    return false;
  }

  std::string line;
  if (!std::getline(ifs, line)) {
    error = "file is empty";
    return false;
  }
  const auto header = split_csv_line(line);
  const auto header_format = classify_playback_csv_header(header, kArmJoints);
  std::vector<size_t> columns;
  if (header_format == CsvHeaderFormat::kLegacy) {
    trajectory.is_v2 = false;
    for (size_t i = 0; i < kArmJoints; ++i) {
      const std::string expected = std::string("J") + std::to_string(i + 1);
      if (header[i + 1] != expected) {
        error = "unsupported legacy CSV header";
        return false;
      }
      trajectory.joint_mask[i] = true;
      columns.push_back(i);
    }
  } else if (header_format == CsvHeaderFormat::kV2) {
    trajectory.is_v2 = true;
    for (size_t column = 2; column < header.size(); ++column) {
      size_t joint_index = 0;
      if (!parse_joint_name(header[column], joint_index) ||
        trajectory.joint_mask[joint_index])
      {
        error = "invalid or duplicate V2 joint column: " + header[column];
        return false;
      }
      trajectory.joint_mask[joint_index] = true;
      columns.push_back(joint_index);
    }
  } else {
    error = "unsupported CSV header";
    return false;
  }

  const size_t expected_fields =
    trajectory.is_v2 ? columns.size() + 2 : columns.size() + 1;
  double previous_time = 0.0;
  uint64_t previous_sample_index = 0;
  size_t line_number = 1;
  while (std::getline(ifs, line)) {
    ++line_number;
    if (line.empty() || line == "\r") {
      continue;
    }
    const auto fields = split_csv_line(line);
    if (fields.size() != expected_fields) {
      error = "wrong column count at line " + std::to_string(line_number);
      return false;
    }

    TrajectoryPoint point;
    if (!parse_finite_double(fields[0], point.t) || point.t < 0.0) {
      error = "invalid timestamp at line " + std::to_string(line_number);
      return false;
    }
    if (!trajectory.points.empty() && point.t <= previous_time) {
      error = "timestamps must be strictly increasing at line " +
        std::to_string(line_number);
      return false;
    }

    const size_t data_offset = trajectory.is_v2 ? 2 : 1;
    if (trajectory.is_v2) {
      if (!parse_uint64(fields[1], point.sample_index)) {
        error = "invalid sample_index at line " + std::to_string(line_number);
        return false;
      }
      if (!trajectory.points.empty() && point.sample_index <= previous_sample_index) {
        error = "sample_index must be strictly increasing at line " +
          std::to_string(line_number);
        return false;
      }
    } else {
      point.sample_index = trajectory.points.size();
    }

    for (size_t column = 0; column < columns.size(); ++column) {
      double position = 0.0;
      if (!parse_finite_double(fields[data_offset + column], position)) {
        error = "invalid position at line " + std::to_string(line_number);
        return false;
      }
      point.pos[columns[column]] = position;
    }
    previous_time = point.t;
    previous_sample_index = point.sample_index;
    trajectory.points.push_back(point);
  }

  if (trajectory.points.empty()) {
    error = "CSV contains no trajectory points";
    return false;
  }
  return true;
}

double minimum_jerk(double u)
{
  return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
}
}  // namespace

class PlaybackNode : public rclcpp::Node
{
public:
  PlaybackNode() : Node("playback")
  {
    can_interface_ = declare_parameter<std::string>("can_interface", "can1");
    baudrate_ = declare_parameter<int>("baudrate", 0);
    const std::string node_ids_str =
      declare_parameter<std::string>("node_ids", "1,2,3,4,5,6");
    const std::string joint_indices_str =
      declare_parameter<std::string>("joint_indices", "");
    const std::string position_min_str =
      declare_parameter<std::string>("position_min", "0.0,0.0,0.0,0.0,0.0,0.0");
    const std::string position_max_str =
      declare_parameter<std::string>("position_max", "0.0,0.0,0.0,0.0,0.0,0.0");
    joint_states_topic_ =
      declare_parameter<std::string>("joint_states_topic", "/arm_joint_states");
    input_file_ = declare_parameter<std::string>("input_file", "");
    speed_scale_ = declare_parameter<double>("speed_scale", 1.0);
    loop_ = declare_parameter<bool>("loop", false);
    approach_velocity_rad_s_ =
      declare_parameter<double>("approach_velocity_rad_s", 0.1);
    approach_rate_hz_ = declare_parameter<double>("approach_rate_hz", 50.0);
    approach_min_duration_s_ =
      declare_parameter<double>("approach_min_duration_s", 0.5);
    start_tolerance_rad_ = declare_parameter<double>("start_tolerance_rad", 0.005);
    feedback_timeout_ms_ = declare_parameter<int>("feedback_timeout_ms", 15);
    max_command_step_rad_ = declare_parameter<double>("max_command_step_rad", 0.005);
    max_following_error_rad_ =
      declare_parameter<double>("max_following_error_rad", 0.05);
    following_error_cycles_ = declare_parameter<int>("following_error_cycles", 3);

    if (can_interface_.empty() || baudrate_ < 0) {
      RCLCPP_FATAL(get_logger(), "can_interface must be non-empty and baudrate non-negative");
      throw std::runtime_error("bad CAN parameters");
    }

    const auto node_ids = miraculous_driver::parse_int_parameter_list(
      node_ids_str, {1, 2, 3, 4, 5, 6}, "node_ids");
    const auto joint_indices =
      miraculous_driver::parse_int_parameter_list(
      joint_indices_str, default_joint_indices(node_ids.size()), "joint_indices");
    const auto position_min = miraculous_driver::parse_double_parameter_list(
      position_min_str, std::vector<double>(kArmJoints, 0.0), "position_min");
    const auto position_max = miraculous_driver::parse_double_parameter_list(
      position_max_str, std::vector<double>(kArmJoints, 0.0), "position_max");

    if (node_ids.empty() || node_ids.size() > miraculous_driver::kArmJoints ||
      !validate_node_ids(node_ids))
    {
      RCLCPP_FATAL(get_logger(),
        "node_ids must list 1..%zu unique values in range 1..127",
        miraculous_driver::kArmJoints);
      throw std::runtime_error("bad node_ids");
    }
    if (joint_indices.size() != node_ids.size() || !validate_joint_indices(joint_indices)) {
      RCLCPP_FATAL(get_logger(),
        "joint_indices must list %zu unique values in range 0..%zu",
        node_ids.size(), miraculous_driver::kArmJoints - 1);
      throw std::runtime_error("bad joint_indices");
    }
    if (!valid_limit_size(position_min, node_ids.size()) ||
      !valid_limit_size(position_max, node_ids.size()))
    {
      RCLCPP_FATAL(get_logger(),
        "position_min and position_max must each contain 1, %zu, or %zu values.",
        node_ids.size(), miraculous_driver::kArmJoints);
      throw std::runtime_error("bad position limits");
    }
    for (size_t i = 0; i < node_ids.size(); ++i) {
      const size_t joint_index = static_cast<size_t>(joint_indices[i]);
      const double lower = limit_value_for_joint(position_min, i, joint_index);
      const double upper = limit_value_for_joint(position_max, i, joint_index);
      if (!std::isfinite(lower) || !std::isfinite(upper) || lower >= upper) {
        RCLCPP_FATAL(get_logger(),
          "active playback requires finite ordered limits for J%zu; got [%.9f, %.9f]",
          joint_index + 1, lower, upper);
        throw std::runtime_error("missing or invalid active playback limits");
      }
    }
    if (!std::isfinite(speed_scale_) || speed_scale_ <= 0.0 || speed_scale_ > 1.0 ||
      !std::isfinite(approach_velocity_rad_s_) || approach_velocity_rad_s_ <= 0.0 ||
      !std::isfinite(approach_rate_hz_) || approach_rate_hz_ <= 0.0 ||
      approach_rate_hz_ > 200.0 ||
      !std::isfinite(approach_min_duration_s_) || approach_min_duration_s_ < 0.0 ||
      !std::isfinite(start_tolerance_rad_) || start_tolerance_rad_ < 0.0 ||
      feedback_timeout_ms_ <= 0 ||
      !std::isfinite(max_command_step_rad_) || max_command_step_rad_ <= 0.0 ||
      !std::isfinite(max_following_error_rad_) || max_following_error_rad_ <= 0.0 ||
      following_error_cycles_ <= 0)
    {
      RCLCPP_FATAL(get_logger(),
        "invalid playback timing/safety parameters; speed_scale must be in (0, 1]");
      throw std::runtime_error("bad playback parameters");
    }

    ArmConfig config;
    config.can_interface = can_interface_;
    config.baudrate = static_cast<CiaBaudrate_t>(baudrate_);
    config.sync_period_us = 0;
    config.read_rate_hz = 50.0;
    config.manual_feedback_timeout_ms =
      static_cast<uint32_t>(feedback_timeout_ms_);
    config.max_command_step_rad = max_command_step_rad_;
    config.max_following_error_rad = max_following_error_rad_;
    config.following_error_cycles = static_cast<uint32_t>(following_error_cycles_);
    for (size_t i = 0; i < node_ids.size(); ++i) {
      const size_t joint_index = static_cast<size_t>(joint_indices[i]);
      JointConfig joint;
      joint.name = std::string("J") + std::to_string(joint_index + 1);
      joint.joint_index = joint_index;
      joint.node_id = static_cast<uint8_t>(node_ids[i]);
      joint.position_min = limit_value_for_joint(position_min, i, joint_index);
      joint.position_max = limit_value_for_joint(position_max, i, joint_index);
      config.joints.push_back(joint);
      joints_.push_back(joint);
      configured_joint_mask_[joint_index] = true;
    }

    arm_ = std::make_unique<MiraculousArm>();
    if (!arm_->init(config)) {
      RCLCPP_FATAL(get_logger(), "Failed to open motors.");
      throw std::runtime_error("init failed");
    }

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::SystemDefaultsQoS());
    play_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/play", std::bind(&PlaybackNode::on_play, this,
        std::placeholders::_1, std::placeholders::_2));
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/stop", std::bind(&PlaybackNode::on_stop, this,
        std::placeholders::_1, std::placeholders::_2));
    state_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&PlaybackNode::publish_state, this));

    RCLCPP_INFO(get_logger(),
      "Playback ready. can=%s input_file=%s speed_scale=%.2f "
      "approach_velocity=%.3f rad/s",
      can_interface_.c_str(), input_file_.c_str(), speed_scale_,
      approach_velocity_rad_s_);
  }

  ~PlaybackNode() override
  {
    stop_requested_ = true;
    if (play_thread_.joinable()) {
      play_thread_.join();
    }
    if (arm_) {
      disable_arm("playback shutdown");
      arm_->shutdown();
    }
  }

private:
  bool disable_arm(const char * context)
  {
    if (!arm_ || arm_->disable()) {
      return true;
    }
    RCLCPP_ERROR(
      get_logger(),
      "%s: failed to verify every drive left Operation Enabled; SDK I/O remains quarantined",
      context);
    return false;
  }

  void on_play(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (playing_) {
      response->success = false;
      response->message = "already playing";
      return;
    }
    if (input_file_.empty()) {
      response->success = false;
      response->message = "input_file not set";
      return;
    }

    LoadedTrajectory trajectory;
    std::string error;
    if (!load_csv(input_file_, trajectory, error) ||
      !validate_trajectory(trajectory, error))
    {
      response->success = false;
      response->message =
        "failed to load trajectory from " + input_file_ + ": " + error;
      return;
    }
    if (loop_ && trajectory.points.size() < 2) {
      response->success = false;
      response->message = "loop playback requires at least two trajectory points";
      return;
    }

    if (play_thread_.joinable()) {
      play_thread_.join();
    }
    stop_requested_ = false;
    playing_ = true;
    response->success = true;
    response->message =
      "starting safe playback (" + std::to_string(trajectory.points.size()) +
      " points, approach to first point enabled)";
    play_thread_ =
      std::thread(&PlaybackNode::playback_loop, this, std::move(trajectory));
  }

  void on_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    stop_requested_ = true;
    response->success = true;
    response->message = "stop requested";
  }

  bool validate_trajectory(LoadedTrajectory & trajectory, std::string & error) const
  {
    bool has_replayed_joint = false;
    for (size_t i = 0; i < kArmJoints; ++i) {
      if (!trajectory.joint_mask[i]) {
        continue;
      }
      if (!configured_joint_mask_[i]) {
        if (trajectory.is_v2) {
          error = "V2 CSV contains unconfigured joint J" + std::to_string(i + 1);
          return false;
        }
        trajectory.joint_mask[i] = false;
        continue;
      }
      has_replayed_joint = true;
    }
    if (!has_replayed_joint) {
      error = "CSV has no joints configured in this playback node";
      return false;
    }

    for (size_t point_index = 0; point_index < trajectory.points.size(); ++point_index) {
      for (const auto & joint : joints_) {
        const size_t i = joint.joint_index;
        if (!trajectory.joint_mask[i] || joint.position_max <= joint.position_min) {
          continue;
        }
        const double position = trajectory.points[point_index].pos[i];
        if (position < joint.position_min || position > joint.position_max) {
          error = "point " + std::to_string(point_index) + " joint " + joint.name +
            " is outside software limits";
          return false;
        }
      }
    }
    return true;
  }

  bool current_position_within_limits(
    const std::array<double, kArmJoints> & position, std::string & error) const
  {
    for (const auto & joint : joints_) {
      const double value = position[joint.joint_index];
      if (!std::isfinite(value)) {
        error = "current position of " + joint.name + " is not finite";
        return false;
      }
      if (joint.position_max <= joint.position_min) {
        continue;
      }
      if (value < joint.position_min || value > joint.position_max) {
        error = "current position of " + joint.name + " is outside software limits";
        return false;
      }
    }
    return true;
  }

  bool sleep_until_interruptible(
    const std::chrono::steady_clock::time_point & wake) const
  {
    while (!stop_requested_ && !arm_->has_fault()) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= wake) {
        return true;
      }
      const auto remaining = wake - now;
      std::this_thread::sleep_for(
        std::min(remaining, std::chrono::steady_clock::duration(
          std::chrono::milliseconds(5))));
    }
    return false;
  }

  bool approach_first_point(
    const LoadedTrajectory & trajectory,
    std::array<double, kArmJoints> & hold_target,
    std::string & error)
  {
    std::array<double, kArmJoints> start{};
    if (!arm_->get_positions_rad(start)) {
      error = "cannot read current position before first-point approach";
      return false;
    }
    if (!current_position_within_limits(start, error)) {
      return false;
    }

    std::array<double, kArmJoints> target = start;
    double max_delta = 0.0;
    for (size_t i = 0; i < kArmJoints; ++i) {
      if (trajectory.joint_mask[i]) {
        target[i] = trajectory.points.front().pos[i];
        max_delta = std::max(max_delta, std::abs(target[i] - start[i]));
      }
    }
    hold_target = target;

    if (max_delta <= start_tolerance_rad_) {
      if (!arm_->set_targets_rad(target)) {
        error = "failed to hold the CSV first point";
        return false;
      }
      return true;
    }

    // The maximum derivative of the minimum-jerk blend is 1.875/T.
    const double duration_s = std::max(
      approach_min_duration_s_,
      1.875 * max_delta / approach_velocity_rad_s_);
    const size_t steps = std::max<size_t>(
      1, static_cast<size_t>(std::ceil(duration_s * approach_rate_hz_)));
    const auto start_time = std::chrono::steady_clock::now();

    RCLCPP_INFO(get_logger(),
      "Approaching CSV first point: max_delta=%.6f rad duration=%.3f s steps=%zu",
      max_delta, duration_s, steps);
    for (size_t step = 1; step <= steps; ++step) {
      const double u = static_cast<double>(step) / static_cast<double>(steps);
      const double blend = minimum_jerk(u);
      std::array<double, kArmJoints> command = start;
      for (size_t i = 0; i < kArmJoints; ++i) {
        if (trajectory.joint_mask[i]) {
          command[i] = start[i] + blend * (target[i] - start[i]);
        }
      }
      const auto wake =
        start_time + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(duration_s * u));
      if (!sleep_until_interruptible(wake)) {
        error = stop_requested_ ? "stopped during first-point approach" :
          "fault during first-point approach";
        return false;
      }
      if (!arm_->set_targets_rad(command) || arm_->has_fault()) {
        error = "target/feedback failure during first-point approach";
        return false;
      }
    }
    return true;
  }

  bool play_once(
    const LoadedTrajectory & trajectory,
    const std::array<double, kArmJoints> & hold_target,
    std::string & error)
  {
    const auto playback_start = std::chrono::steady_clock::now();
    const double first_timestamp = trajectory.points.front().t;
    for (const auto & point : trajectory.points) {
      const double target_time = (point.t - first_timestamp) / speed_scale_;
      const auto wake =
        playback_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(target_time));
      if (!sleep_until_interruptible(wake)) {
        error = stop_requested_ ? "stop requested" : "drive fault during playback";
        return false;
      }

      std::array<double, kArmJoints> targets = hold_target;
      for (size_t i = 0; i < kArmJoints; ++i) {
        if (trajectory.joint_mask[i]) {
          targets[i] = point.pos[i];
        }
      }
      if (!arm_->set_targets_rad(targets) || arm_->has_fault()) {
        error = "target/feedback failure during trajectory playback";
        return false;
      }
    }
    return true;
  }

  void playback_loop(LoadedTrajectory trajectory)
  {
    std::string error;
    std::array<double, kArmJoints> current{};
    if (!arm_->get_positions_rad(current)) {
      RCLCPP_ERROR(get_logger(),
        "No current feedback is available; wait for feedback and retry playback.");
      playing_ = false;
      return;
    }
    if (!current_position_within_limits(current, error)) {
      RCLCPP_ERROR(get_logger(), "Playback rejected: %s", error.c_str());
      playing_ = false;
      return;
    }

    RCLCPP_INFO(get_logger(), "Enabling CSP and seeding current position...");
    if (!arm_->enable_csp()) {
      RCLCPP_ERROR(get_logger(), "enable_csp failed, aborting playback.");
      disable_arm("CSP enable failure");
      playing_ = false;
      return;
    }

    bool completed = true;
    do {
      std::array<double, kArmJoints> hold_target{};
      if (!approach_first_point(trajectory, hold_target, error) ||
        !play_once(trajectory, hold_target, error))
      {
        completed = false;
        break;
      }
    } while (loop_ && !stop_requested_);

    const bool disabled = disable_arm("playback completion");
    playing_ = false;
    if (!disabled) {
      RCLCPP_ERROR(
        get_logger(), "Playback ended, but the drive safe state could not be verified.");
    } else if (completed) {
      RCLCPP_INFO(get_logger(), "Playback finished.");
    } else if (stop_requested_) {
      RCLCPP_INFO(get_logger(), "Playback stopped: %s", error.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Playback aborted: %s", error.c_str());
    }
  }

  void publish_state()
  {
    std::array<double, miraculous_driver::kArmJoints> position{};
    if (!arm_->get_positions_rad(position)) {
      return;
    }
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = now();
    joint_state.name.reserve(joints_.size());
    joint_state.position.reserve(joints_.size());
    for (const auto & joint : joints_) {
      joint_state.name.push_back(joint.name);
      joint_state.position.push_back(position[joint.joint_index]);
    }
    joint_pub_->publish(joint_state);
  }

  std::string can_interface_;
  int baudrate_{0};
  std::string joint_states_topic_;
  std::string input_file_;
  double speed_scale_{1.0};
  bool loop_{false};
  double approach_velocity_rad_s_{0.1};
  double approach_rate_hz_{50.0};
  double approach_min_duration_s_{0.5};
  double start_tolerance_rad_{0.005};
  int feedback_timeout_ms_{15};
  double max_command_step_rad_{0.005};
  double max_following_error_rad_{0.05};
  int following_error_cycles_{3};
  std::vector<JointConfig> joints_;
  std::array<bool, kArmJoints> configured_joint_mask_{};

  std::unique_ptr<MiraculousArm> arm_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr play_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  std::thread play_thread_;
  std::atomic<bool> playing_{false};
  std::atomic<bool> stop_requested_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlaybackNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
