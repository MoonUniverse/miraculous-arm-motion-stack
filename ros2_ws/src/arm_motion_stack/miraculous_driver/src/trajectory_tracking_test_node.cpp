/**
 * @file trajectory_tracking_test_node.cpp
 * @brief Multi-joint sinusoidal trajectory tracking test for CSP mode.
 *
 * Sends the same-phase sine/cosine position command to selected joints at a
 * fixed rate, while recording each commanded target and encoder reading.
 * At the end, saves a CSV file and prints tracking accuracy metrics (RMSE,
 * MAE, max error, correlation).
 *
 * Usage:
 *   ros2 run miraculous_driver trajectory_tracking_test_node
 *   ros2 service call /trajectory_test/start std_srvs/srv/Trigger
 *   ros2 service call /trajectory_test/stop  std_srvs/srv/Trigger
 */

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "miraculous_driver/miraculous_arm.hpp"
#include "strict_parameter_lists.hpp"

using miraculous_driver::ArmConfig;
using miraculous_driver::JointConfig;
using miraculous_driver::MiraculousArm;
using miraculous_driver::kArmJoints;

namespace
{
std::vector<int> default_joint_indices(size_t size)
{
  std::vector<int> out;
  out.reserve(size);
  for (size_t i = 0; i < size; ++i) {
    out.push_back(static_cast<int>(i));
  }
  return out;
}

std::string format_joint_names(const std::vector<size_t> & indices)
{
  std::ostringstream out;
  for (size_t i = 0; i < indices.size(); ++i) {
    if (i > 0) {
      out << ",";
    }
    out << "J" << indices[i] + 1;
  }
  return out.str();
}

bool validate_joint_indices(const std::vector<int> & indices)
{
  std::array<bool, kArmJoints> seen{};
  for (const int index : indices) {
    if (index < 0 || static_cast<size_t>(index) >= kArmJoints ||
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
  return values.size() == 1 || values.size() == active_count || values.size() == kArmJoints;
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

/// Generate a timestamped default filename.
std::string default_filename()
{
  std::time_t t = std::time(nullptr);
  std::tm tm{};
  localtime_r(&t, &tm);
  char buf[64];
  std::strftime(buf, sizeof(buf), "tracking_test_%Y%m%d_%H%M%S.csv", &tm);
  return std::string(buf);
}

/// One synchronized sample for all selected joints.
struct Sample
{
  double timestamp{};                                      ///< seconds since test start
  std::array<double, kArmJoints> command_rad{};             ///< commanded targets [rad]
  std::array<double, kArmJoints> actual_rad{};              ///< encoder positions [rad]
  std::array<double, kArmJoints> error_rad{};               ///< command - actual [rad]
};

}  // namespace

class TrajectoryTrackingTestNode : public rclcpp::Node
{
public:
  TrajectoryTrackingTestNode() : Node("trajectory_test")
  {
    // ---- motor / CAN parameters ----
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
    const int sync_period_us = declare_parameter<int>("sync_period_us", 0);

    // ---- trajectory parameters ----
    amplitude_ = declare_parameter<double>("amplitude", 0.03);        // [rad]
    period_ = declare_parameter<double>("period", 6.0);               // [s]
    frequency_ = declare_parameter<double>("frequency", 50.0);         // [Hz]
    waveform_ = declare_parameter<std::string>("waveform", "sin");    // "sin"|"cos"
    const int legacy_test_joint = declare_parameter<int>("test_joint", 0);
    const std::string test_joints_str =
      declare_parameter<std::string>("test_joints", "");
    duration_ = declare_parameter<double>("duration", 3.0);            // 0 = manual
    output_file_ = declare_parameter<std::string>("output_file", "");
    joint_states_topic_ =
      declare_parameter<std::string>("joint_states_topic", "/arm_joint_states");
    settle_time_ = declare_parameter<double>("settle_time", 0.5);      // [s]
    feedback_timeout_ms_ = declare_parameter<int>("feedback_timeout_ms", 15);
    max_command_step_rad_ = declare_parameter<double>("max_command_step_rad", 0.005);
    max_following_error_rad_ =
      declare_parameter<double>("max_following_error_rad", 0.05);
    following_error_cycles_ = declare_parameter<int>("following_error_cycles", 3);

    // ---- validate ----
    if (can_interface_.empty() || baudrate_ < 0 || sync_period_us != 0 ||
      !std::isfinite(amplitude_) || amplitude_ <= 0.0 ||
      !std::isfinite(period_) || period_ <= 0.0 ||
      !std::isfinite(frequency_) || frequency_ <= 0.0 || frequency_ > 200.0 ||
      !std::isfinite(duration_) || duration_ < 0.0 ||
      !std::isfinite(settle_time_) || settle_time_ < 0.0 ||
      feedback_timeout_ms_ <= 0 ||
      !std::isfinite(max_command_step_rad_) || max_command_step_rad_ <= 0.0 ||
      !std::isfinite(max_following_error_rad_) || max_following_error_rad_ <= 0.0 ||
      following_error_cycles_ <= 0 ||
      (waveform_ != "sin" && waveform_ != "cos"))
    {
      RCLCPP_FATAL(get_logger(),
        "invalid active test parameters: Manual SYNC (sync_period_us=0), finite "
        "timing, feedback timeout, and waveform sin/cos are required");
      throw std::runtime_error("bad active test parameters");
    }
    sync_period_us_ = static_cast<uint32_t>(sync_period_us);

    const std::vector<int> requested_test_joints =
      miraculous_driver::parse_int_parameter_list(
      test_joints_str, {legacy_test_joint}, "test_joints");
    if (requested_test_joints.empty() || !validate_joint_indices(requested_test_joints)) {
      RCLCPP_FATAL(get_logger(),
        "test_joint/test_joints must list unique values in range 0..%zu", kArmJoints - 1);
      throw std::runtime_error("bad test_joints");
    }
    for (const int index : requested_test_joints) {
      test_joints_.push_back(static_cast<size_t>(index));
    }

    const auto node_ids = miraculous_driver::parse_int_parameter_list(
      node_ids_str, {1, 2, 3, 4, 5, 6}, "node_ids");
    const auto joint_indices = miraculous_driver::parse_int_parameter_list(
      joint_indices_str, default_joint_indices(node_ids.size()), "joint_indices");
    const auto position_min = miraculous_driver::parse_double_parameter_list(
      position_min_str, std::vector<double>(kArmJoints, 0.0), "position_min");
    const auto position_max = miraculous_driver::parse_double_parameter_list(
      position_max_str, std::vector<double>(kArmJoints, 0.0), "position_max");

    if (node_ids.empty() || node_ids.size() > kArmJoints ||
      !validate_node_ids(node_ids))
    {
      RCLCPP_FATAL(get_logger(),
        "node_ids must list 1..%zu unique values in range 1..127", kArmJoints);
      throw std::runtime_error("bad params");
    }
    if (joint_indices.size() != node_ids.size() || !validate_joint_indices(joint_indices)) {
      RCLCPP_FATAL(get_logger(),
        "joint_indices must list %zu unique values in range 0..%zu",
        node_ids.size(), kArmJoints - 1);
      throw std::runtime_error("bad joint_indices");
    }
    for (const size_t test_joint : test_joints_) {
      if (std::find(
          joint_indices.begin(), joint_indices.end(), static_cast<int>(test_joint)) ==
        joint_indices.end())
      {
        RCLCPP_FATAL(get_logger(),
          "test joint J%zu is not listed in joint_indices.", test_joint + 1);
        throw std::runtime_error("inactive test_joint");
      }
    }
    if (!valid_limit_size(position_min, node_ids.size()) ||
      !valid_limit_size(position_max, node_ids.size()))
    {
      RCLCPP_FATAL(get_logger(),
        "position_min and position_max must each contain 1, %zu, or %zu values.",
        node_ids.size(), kArmJoints);
      throw std::runtime_error("bad position limits");
    }
    for (size_t i = 0; i < node_ids.size(); ++i) {
      const size_t joint_index = static_cast<size_t>(joint_indices[i]);
      const double lower = limit_value_for_joint(position_min, i, joint_index);
      const double upper = limit_value_for_joint(position_max, i, joint_index);
      if (!std::isfinite(lower) || !std::isfinite(upper) || lower >= upper) {
        RCLCPP_FATAL(get_logger(),
          "active trajectory test requires finite ordered limits for J%zu; "
          "got [%.9f, %.9f]",
          joint_index + 1, lower, upper);
        throw std::runtime_error("missing or invalid active trajectory limits");
      }
      position_min_[joint_index] = lower;
      position_max_[joint_index] = upper;
    }

    // ---- open motors and configure CSP once (power stage remains disabled) ----
    ArmConfig config;
    config.can_interface = can_interface_;
    config.baudrate = static_cast<CiaBaudrate_t>(baudrate_);
    config.sync_period_us = sync_period_us_;
    config.read_rate_hz = std::max(frequency_, 10.0);
    config.manual_feedback_timeout_ms = static_cast<uint32_t>(feedback_timeout_ms_);
    config.max_command_step_rad = max_command_step_rad_;
    config.max_following_error_rad = max_following_error_rad_;
    config.following_error_cycles = static_cast<uint32_t>(following_error_cycles_);
    config.enable_emcy_monitor = declare_parameter<bool>("enable_emcy_monitor", true);
    for (size_t i = 0; i < node_ids.size(); ++i) {
      const size_t joint_index = static_cast<size_t>(joint_indices[i]);
      JointConfig jc;
      jc.name = std::string("J") + std::to_string(joint_index + 1);
      jc.joint_index = joint_index;
      jc.node_id = static_cast<uint8_t>(node_ids[i]);
      jc.position_min = limit_value_for_joint(position_min, i, joint_index);
      jc.position_max = limit_value_for_joint(position_max, i, joint_index);
      config.joints.push_back(jc);
    }

    arm_ = std::make_unique<MiraculousArm>();
    if (!arm_->init(config)) {
      RCLCPP_FATAL(get_logger(), "Failed to initialize motors or configure CSP.");
      throw std::runtime_error("init failed");
    }

    arm_->set_emcy_callback(
      [this](uint8_t node_id, uint16_t error_code, uint8_t error_reg) {
        RCLCPP_ERROR(get_logger(),
          "EMCY from node %u: code=0x%04X reg=0x%02X",
          node_id, error_code, error_reg);
        fault_flag_ = true;
      });

    // ---- ROS interfaces ----
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::SystemDefaultsQoS());

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/start", std::bind(&TrajectoryTrackingTestNode::on_start, this,
        std::placeholders::_1, std::placeholders::_2));
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/stop", std::bind(&TrajectoryTrackingTestNode::on_stop, this,
        std::placeholders::_1, std::placeholders::_2));

    // ---- lightweight state publisher (50 Hz) for RViz monitoring ----
    state_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&TrajectoryTrackingTestNode::publish_state, this));

    RCLCPP_INFO(get_logger(),
      "Trajectory tracking test ready. joints=%s waveform=%s amp=%.3f period=%.1f "
      "freq=%.0f sync_period_us=%u",
      format_joint_names(test_joints_).c_str(), waveform_.c_str(),
      amplitude_, period_, frequency_, sync_period_us_);
    RCLCPP_INFO(get_logger(),
      "Call: ros2 service call /trajectory_test/start std_srvs/srv/Trigger");
  }

  ~TrajectoryTrackingTestNode() override
  {
    if (running_) {
      stop_test();
    }
    if (arm_) {
      arm_->shutdown();
    }
  }

private:
  // ================================================================ services
  void on_start(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (running_) {
      res->success = false;
      res->message = "test already running";
      return;
    }
    res->success = start_test();
    res->message = res->success
      ? ("test started, output -> " + current_file_)
      : "failed to start (see logs)";
  }

  void on_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (!running_) {
      res->success = false;
      res->message = "not running";
      return;
    }
    res->success = stop_test();
    res->message = res->success
      ? ("test stopped, saved " + current_file_)
      : "test stopped, but the drive safe state could not be verified (see logs)";
  }

  // ================================================================ test flow
  bool start_test()
  {
    // CSP was configured once during arm initialization; only enable the power
    // stage and seed the current position for this run.
    RCLCPP_INFO(get_logger(), "Enabling CSP power stage ...");
    if (!arm_->enable_csp()) {
      RCLCPP_ERROR(get_logger(), "enable_csp failed.");
      return false;
    }

    // Let the controller settle on the seeded position.
    RCLCPP_INFO(get_logger(), "Settling %.1f s ...", settle_time_);
    std::this_thread::sleep_for(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(settle_time_)));

    // Read the settled position → use as DC offset so the sine is centred
    // on the current joint angle (avoids a jump to zero).
    std::array<double, kArmJoints> cur{};
    if (!arm_->get_positions_rad(cur)) {
      RCLCPP_ERROR(get_logger(), "No valid motor feedback after enable_csp; refusing to start.");
      disable_arm("feedback failure after CSP enable");
      return false;
    }
    for (const size_t joint : test_joints_) {
      dc_offsets_[joint] = cur[joint];
      if (dc_offsets_[joint] - amplitude_ < position_min_[joint] ||
        dc_offsets_[joint] + amplitude_ > position_max_[joint])
      {
        RCLCPP_ERROR(get_logger(),
          "J%zu test range [%.6f, %.6f] exceeds software limits [%.6f, %.6f]",
          joint + 1, dc_offsets_[joint] - amplitude_, dc_offsets_[joint] + amplitude_,
          position_min_[joint], position_max_[joint]);
        disable_arm("test range rejection");
        return false;
      }
      RCLCPP_INFO(get_logger(), "DC offset (current J%zu pos) = %.6f rad",
        joint + 1, dc_offsets_[joint]);
    }

    // Open output file.
    current_file_ = output_file_.empty() ? default_filename() : output_file_;
    ofs_.open(current_file_);
    if (!ofs_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Cannot open %s", current_file_.c_str());
      disable_arm("output file failure");
      return false;
    }
    if (test_joints_.size() == 1) {
      ofs_ << "timestamp,command_rad,actual_rad,error_rad\n";
    } else {
      ofs_ << "timestamp";
      for (const size_t joint : test_joints_) {
        const std::string name = "J" + std::to_string(joint + 1);
        ofs_ << "," << name << "_command_rad"
             << "," << name << "_actual_rad"
             << "," << name << "_error_rad";
      }
      ofs_ << "\n";
    }
    ofs_.precision(9);
    ofs_ << std::fixed;

    // Start the high-rate control + recording timer.
    samples_.clear();
    fault_flag_ = false;
    running_ = true;
    start_time_ = std::chrono::steady_clock::now();

    const auto period_us = static_cast<int64_t>(1.0e6 / frequency_);
    timer_ = create_wall_timer(
      std::chrono::microseconds(period_us),
      std::bind(&TrajectoryTrackingTestNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "Test started → %s", current_file_.c_str());
    return true;
  }

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

  bool stop_test()
  {
    running_ = false;
    if (timer_) {
      timer_->cancel();
      timer_.reset();
    }
    if (ofs_.is_open()) {
      ofs_.flush();
      ofs_.close();
    }
    const bool disabled = disable_arm("trajectory test stop");

    // Compute and print metrics.
    if (samples_.empty()) {
      RCLCPP_WARN(get_logger(), "No samples recorded.");
      return disabled;
    }
    print_metrics();
    return disabled;
  }

  // ================================================================ timer
  void on_timer()
  {
    if (!running_) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    const double t = std::chrono::duration<double>(now - start_time_).count();

    // Auto-stop if duration is set.
    if (duration_ > 0.0 && t >= duration_) {
      RCLCPP_INFO(get_logger(), "Duration reached (%.1f s), stopping.", duration_);
      stop_test();
      return;
    }

    // Compute commanded target.
    const double omega = 2.0 * M_PI / period_;
    double waveform_val;
    if (waveform_ == "cos") {
      waveform_val = std::cos(omega * t);
    } else {
      waveform_val = std::sin(omega * t);
    }
    // Build all selected targets from the same timestamp. set_targets_rad()
    // writes every configured RPDO before emitting one shared manual SYNC.
    std::array<double, kArmJoints> targets{};
    if (!arm_->get_positions_rad(targets)) {
      RCLCPP_ERROR(get_logger(), "No valid feedback at t=%.3f, stopping test.", t);
      stop_test();
      return;
    }
    std::array<double, kArmJoints> commands{};
    for (const size_t joint : test_joints_) {
      commands[joint] = dc_offsets_[joint] + amplitude_ * waveform_val;
      targets[joint] = commands[joint];
    }
    if (!arm_->set_targets_rad(targets)) {
      RCLCPP_ERROR(get_logger(), "Failed to send CSP target at t=%.3f, stopping test.", t);
      stop_test();
      return;
    }

    // Read actual position (use the cached value from the background thread).
    std::array<double, kArmJoints> actual{};
    if (!arm_->get_positions_rad(actual)) {
      RCLCPP_ERROR(get_logger(), "No valid feedback after target at t=%.3f, stopping test.", t);
      stop_test();
      return;
    }

    // Record sample.
    Sample sample;
    sample.timestamp = t;
    for (const size_t joint : test_joints_) {
      sample.command_rad[joint] = commands[joint];
      sample.actual_rad[joint] = actual[joint];
      sample.error_rad[joint] = commands[joint] - actual[joint];
    }
    samples_.push_back(sample);

    // Write to CSV.
    ofs_ << t;
    for (const size_t joint : test_joints_) {
      ofs_ << "," << sample.command_rad[joint]
           << "," << sample.actual_rad[joint]
           << "," << sample.error_rad[joint];
    }
    ofs_ << "\n";

    // Fault check.
    if (fault_flag_) {
      RCLCPP_ERROR(get_logger(), "Fault detected at t=%.3f, stopping test.", t);
      stop_test();
    }
  }

  // ================================================================ metrics
  void print_metrics() const
  {
    const size_t n = samples_.size();
    RCLCPP_INFO(get_logger(), "========================================");
    RCLCPP_INFO(get_logger(), "  Trajectory Tracking Results");
    RCLCPP_INFO(get_logger(), "========================================");
    RCLCPP_INFO(get_logger(), "  Samples:       %zu", n);
    RCLCPP_INFO(get_logger(), "  Duration:      %.3f s", samples_.back().timestamp);

    for (const size_t joint : test_joints_) {
      print_joint_metrics(joint);
    }

    RCLCPP_INFO(get_logger(), "  CSV file:       %s", current_file_.c_str());
    RCLCPP_INFO(get_logger(), "========================================");
    RCLCPP_INFO(get_logger(), "Plot with:");
    RCLCPP_INFO(get_logger(), "  python3 plot_trajectory.py %s", current_file_.c_str());
  }

  void print_joint_metrics(size_t joint) const
  {
    const size_t n = samples_.size();
    double sum_sq_err = 0.0;
    double sum_abs_err = 0.0;
    double max_abs_err = 0.0;
    double sum_cmd = 0.0;
    double sum_act = 0.0;
    double sum_cmd_act = 0.0;
    double sum_cmd_sq = 0.0;
    double sum_act_sq = 0.0;
    double min_cmd = samples_.front().command_rad[joint];
    double max_cmd = samples_.front().command_rad[joint];
    double min_act = samples_.front().actual_rad[joint];
    double max_act = samples_.front().actual_rad[joint];

    for (const auto & s : samples_) {
      const double command = s.command_rad[joint];
      const double actual = s.actual_rad[joint];
      const double e = s.error_rad[joint];
      const double ae = std::abs(e);
      sum_sq_err += e * e;
      sum_abs_err += ae;
      if (ae > max_abs_err) {
        max_abs_err = ae;
      }
      sum_cmd += command;
      sum_act += actual;
      sum_cmd_act += command * actual;
      sum_cmd_sq += command * command;
      sum_act_sq += actual * actual;
      min_cmd = std::min(min_cmd, command);
      max_cmd = std::max(max_cmd, command);
      min_act = std::min(min_act, actual);
      max_act = std::max(max_act, actual);
    }

    const double rmse = std::sqrt(sum_sq_err / n);
    const double mae = sum_abs_err / n;
    const double lag = estimate_lag(joint);

    // Pearson correlation coefficient.
    const double mean_cmd = sum_cmd / n;
    const double mean_act = sum_act / n;
    const double cov = sum_cmd_act / n - mean_cmd * mean_act;
    const double var_cmd = sum_cmd_sq / n - mean_cmd * mean_cmd;
    const double var_act = sum_act_sq / n - mean_act * mean_act;
    const double corr =
      (var_cmd > 0.0 && var_act > 0.0)
        ? cov / std::sqrt(var_cmd * var_act)
        : 0.0;

    RCLCPP_INFO(get_logger(), "  -- J%zu --", joint + 1);
    RCLCPP_INFO(get_logger(), "  Command range: %.6f .. %.6f rad",
      min_cmd, max_cmd);
    RCLCPP_INFO(get_logger(), "  Actual range:  %.6f .. %.6f rad",
      min_act, max_act);
    RCLCPP_INFO(get_logger(), "  RMSE:          %.6f rad  (%.3f deg)",
      rmse, rmse * 180.0 / M_PI);
    RCLCPP_INFO(get_logger(), "  MAE:           %.6f rad  (%.3f deg)",
      mae, mae * 180.0 / M_PI);
    RCLCPP_INFO(get_logger(), "  Max |error|:   %.6f rad  (%.3f deg)",
      max_abs_err, max_abs_err * 180.0 / M_PI);
    RCLCPP_INFO(get_logger(), "  Correlation:   %.6f", corr);
    if (lag >= 0.0) {
      RCLCPP_INFO(get_logger(), "  Est. lag:      %.1f ms  (%.1f deg phase)",
        lag * 1000.0, lag * 360.0 / period_);
    }
  }

  /// Estimate the phase lag by cross-correlating command and actual.
  double estimate_lag(size_t joint) const
  {
    const size_t n = samples_.size();
    if (n < 10) {
      return -1.0;
    }

    // Use the first full period of data.
    const double dt = samples_[1].timestamp - samples_[0].timestamp;
    if (std::abs(dt) < 1e-9) {
      return -1.0;
    }
    const size_t period_samples =
      static_cast<size_t>(period_ / std::abs(dt));
    const size_t search_len = std::min(period_samples, n / 2);

    double best_corr = -1e9;
    int best_shift = 0;

    for (int shift = 0; shift < static_cast<int>(search_len); ++shift) {
      double sum = 0.0;
      size_t count = 0;
      for (size_t i = 0; i + shift < n; ++i) {
        sum += samples_[i].command_rad[joint] * samples_[i + shift].actual_rad[joint];
        ++count;
      }
      if (count > 0 && sum / count > best_corr) {
        best_corr = sum / count;
        best_shift = shift;
      }
    }
    return best_shift * std::abs(dt);
  }

  // ================================================================ state pub
  void publish_state()
  {
    std::array<double, kArmJoints> pos{};
    if (!arm_->get_positions_rad(pos)) {
      return;
    }
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name.reserve(kArmJoints);
    js.position.reserve(kArmJoints);
    for (size_t i = 0; i < kArmJoints; ++i) {
      js.name.push_back(std::string("J") + std::to_string(i + 1));
      js.position.push_back(pos[i]);
    }
    joint_pub_->publish(js);
  }

  // ---- motor / CAN params ----
  std::string can_interface_;
  int baudrate_{0};
  uint32_t sync_period_us_{0};
  std::string joint_states_topic_;

  // ---- trajectory params ----
  double amplitude_{0.5};
  double period_{5.0};
  double frequency_{50.0};
  std::string waveform_{"sin"};
  std::vector<size_t> test_joints_;
  double duration_{0.0};
  double settle_time_{0.5};
  int feedback_timeout_ms_{15};
  double max_command_step_rad_{0.005};
  double max_following_error_rad_{0.05};
  int following_error_cycles_{3};
  std::string output_file_;
  std::array<double, kArmJoints> position_min_{};
  std::array<double, kArmJoints> position_max_{};

  // ---- state ----
  std::unique_ptr<MiraculousArm> arm_;
  std::array<double, kArmJoints> dc_offsets_{};
  bool running_{false};
  bool fault_flag_{false};

  std::chrono::steady_clock::time_point start_time_;
  std::vector<Sample> samples_;
  std::string current_file_;
  std::ofstream ofs_;

  // ---- ROS interfaces ----
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryTrackingTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
