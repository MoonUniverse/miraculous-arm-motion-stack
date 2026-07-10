/**
 * @file trajectory_tracking_test_node.cpp
 * @brief Single-joint sinusoidal trajectory tracking test for CSP mode.
 *
 * Sends a sine/cosine position command to one joint at a fixed rate, while
 * synchronously recording the commanded target and the actual encoder reading.
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

using miraculous_driver::ArmConfig;
using miraculous_driver::JointConfig;
using miraculous_driver::MiraculousArm;
using miraculous_driver::kArmJoints;

namespace
{
/// Parse a comma-separated integer list.
std::vector<int> parse_int_list(const std::string & s, const std::vector<int> & def)
{
  if (s.empty()) {
    return def;
  }
  std::vector<int> out;
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    try {
      out.push_back(std::stoi(tok));
    } catch (...) {}
  }
  return out.empty() ? def : out;
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

/// Parse a comma-separated double list.  If a single value is given, broadcast
/// it to all kArmJoints entries.
std::vector<double> parse_double_list(const std::string & s, double single_def)
{
  std::vector<double> out;
  if (s.empty()) {
    out.assign(kArmJoints, single_def);
    return out;
  }
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    try {
      out.push_back(std::stod(tok));
    } catch (...) {}
  }
  if (out.size() == 1) {
    out.assign(kArmJoints, out[0]);
  }
  return out;
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

/// Single recorded sample.
struct Sample
{
  double timestamp;     ///< seconds since test start
  double command_rad;   ///< commanded target position [rad]
  double actual_rad;    ///< actual encoder position [rad]
  double error_rad;     ///< command - actual [rad]
};

}  // namespace

class TrajectoryTrackingTestNode : public rclcpp::Node
{
public:
  TrajectoryTrackingTestNode() : Node("trajectory_test")
  {
    // ---- motor / CAN parameters ----
    can_interface_ = declare_parameter<std::string>("can_interface", "can0");
    baudrate_ = declare_parameter<int>("baudrate", 1000);
    const std::string node_ids_str =
      declare_parameter<std::string>("node_ids", "1,2,3,4,5,6");
    const std::string joint_indices_str =
      declare_parameter<std::string>("joint_indices", "");
    const std::string position_min_str =
      declare_parameter<std::string>("position_min", "0.0,0.0,0.0,0.0,0.0,0.0");
    const std::string position_max_str =
      declare_parameter<std::string>("position_max", "0.0,0.0,0.0,0.0,0.0,0.0");
    sync_period_us_ = static_cast<uint32_t>(
      declare_parameter<int>("sync_period_us", 10000));              // 0=manual, >0=SDK timer

    // ---- trajectory parameters ----
    amplitude_ = declare_parameter<double>("amplitude", 0.03);        // [rad]
    period_ = declare_parameter<double>("period", 6.0);               // [s]
    frequency_ = declare_parameter<double>("frequency", 100.0);        // [Hz]
    waveform_ = declare_parameter<std::string>("waveform", "sin");    // "sin"|"cos"
    test_joint_ = static_cast<size_t>(
      declare_parameter<int>("test_joint", 0));                       // 0=J1..5=J6
    duration_ = declare_parameter<double>("duration", 3.0);            // 0 = manual
    output_file_ = declare_parameter<std::string>("output_file", "");
    joint_states_topic_ =
      declare_parameter<std::string>("joint_states_topic", "/arm_joint_states");
    settle_time_ = declare_parameter<double>("settle_time", 0.5);      // [s]

    // ---- validate ----
    if (test_joint_ >= kArmJoints) {
      RCLCPP_FATAL(get_logger(),
        "test_joint must be 0..%zu, got %zu", kArmJoints - 1, test_joint_);
      throw std::runtime_error("bad test_joint");
    }

    auto node_ids = parse_int_list(node_ids_str, {1, 2, 3, 4, 5, 6});
    auto joint_indices = parse_int_list(joint_indices_str, default_joint_indices(node_ids.size()));
    auto position_min = parse_double_list(position_min_str, 0.0);
    auto position_max = parse_double_list(position_max_str, 0.0);

    if (node_ids.empty() || node_ids.size() > kArmJoints) {
      RCLCPP_FATAL(get_logger(),
        "node_ids must list 1..%zu values", kArmJoints);
      throw std::runtime_error("bad params");
    }
    if (joint_indices.size() != node_ids.size() || !validate_joint_indices(joint_indices)) {
      RCLCPP_FATAL(get_logger(),
        "joint_indices must list %zu unique values in range 0..%zu",
        node_ids.size(), kArmJoints - 1);
      throw std::runtime_error("bad joint_indices");
    }
    if (std::find(joint_indices.begin(), joint_indices.end(), static_cast<int>(test_joint_)) ==
      joint_indices.end())
    {
      RCLCPP_FATAL(get_logger(),
        "test_joint=%zu is not listed in joint_indices.", test_joint_);
      throw std::runtime_error("inactive test_joint");
    }
    if (!valid_limit_size(position_min, node_ids.size()) ||
      !valid_limit_size(position_max, node_ids.size()))
    {
      RCLCPP_FATAL(get_logger(),
        "position_min and position_max must each contain 1, %zu, or %zu values.",
        node_ids.size(), kArmJoints);
      throw std::runtime_error("bad position limits");
    }

    // ---- open motors (init, not enabled) ----
    ArmConfig config;
    config.can_interface = can_interface_;
    config.baudrate = static_cast<CiaBaudrate_t>(baudrate_);
    config.sync_period_us = sync_period_us_;
    config.read_rate_hz = std::max(frequency_, 10.0);
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
      RCLCPP_FATAL(get_logger(), "Failed to open motors.");
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
      "Trajectory tracking test ready. joint=J%zu waveform=%s amp=%.3f period=%.1f "
      "freq=%.0f sync_period_us=%u",
      test_joint_ + 1, waveform_.c_str(), amplitude_, period_, frequency_, sync_period_us_);
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
    stop_test();
    res->success = true;
    res->message = "test stopped, saved " + current_file_;
  }

  // ================================================================ test flow
  bool start_test()
  {
    // Enable CSP.
    RCLCPP_INFO(get_logger(), "Enabling CSP ...");
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
      arm_->disable();
      return false;
    }
    dc_offset_ = cur[test_joint_];
    RCLCPP_INFO(get_logger(), "DC offset (current J%zu pos) = %.6f rad",
      test_joint_ + 1, dc_offset_);

    // Open output file.
    current_file_ = output_file_.empty() ? default_filename() : output_file_;
    ofs_.open(current_file_);
    if (!ofs_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Cannot open %s", current_file_.c_str());
      arm_->disable();
      return false;
    }
    ofs_ << "timestamp,command_rad,actual_rad,error_rad\n";
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

  void stop_test()
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
    if (arm_) {
      arm_->disable();
    }

    // Compute and print metrics.
    if (samples_.empty()) {
      RCLCPP_WARN(get_logger(), "No samples recorded.");
      return;
    }
    print_metrics();
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
    const double command = dc_offset_ + amplitude_ * waveform_val;

    // Build target array: test joint follows trajectory, others hold position.
    std::array<double, kArmJoints> targets{};
    {
      std::array<double, kArmJoints> cur{};
      arm_->get_positions_rad(cur);
      for (size_t i = 0; i < kArmJoints; ++i) {
        targets[i] = (i == test_joint_) ? command : cur[i];
      }
    }
    if (!arm_->set_targets_rad(targets)) {
      RCLCPP_ERROR(get_logger(), "Failed to send CSP target at t=%.3f, stopping test.", t);
      stop_test();
      return;
    }

    // Read actual position (use the cached value from the background thread).
    std::array<double, kArmJoints> actual{};
    arm_->get_positions_rad(actual);
    const double actual_j = actual[test_joint_];
    const double error = command - actual_j;

    // Record sample.
    samples_.push_back({t, command, actual_j, error});

    // Write to CSV.
    ofs_ << t << "," << command << "," << actual_j << "," << error << "\n";

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
    double sum_sq_err = 0.0;
    double sum_abs_err = 0.0;
    double max_abs_err = 0.0;
    double sum_cmd = 0.0;
    double sum_act = 0.0;
    double sum_cmd_act = 0.0;
    double sum_cmd_sq = 0.0;
    double sum_act_sq = 0.0;
    double min_cmd = samples_.front().command_rad;
    double max_cmd = samples_.front().command_rad;
    double min_act = samples_.front().actual_rad;
    double max_act = samples_.front().actual_rad;

    for (const auto & s : samples_) {
      const double e = s.error_rad;
      const double ae = std::abs(e);
      sum_sq_err += e * e;
      sum_abs_err += ae;
      if (ae > max_abs_err) {
        max_abs_err = ae;
      }
      sum_cmd += s.command_rad;
      sum_act += s.actual_rad;
      sum_cmd_act += s.command_rad * s.actual_rad;
      sum_cmd_sq += s.command_rad * s.command_rad;
      sum_act_sq += s.actual_rad * s.actual_rad;
      min_cmd = std::min(min_cmd, s.command_rad);
      max_cmd = std::max(max_cmd, s.command_rad);
      min_act = std::min(min_act, s.actual_rad);
      max_act = std::max(max_act, s.actual_rad);
    }

    const double rmse = std::sqrt(sum_sq_err / n);
    const double mae = sum_abs_err / n;
    const double lag = estimate_lag();

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

    RCLCPP_INFO(get_logger(), "========================================");
    RCLCPP_INFO(get_logger(), "  Trajectory Tracking Results");
    RCLCPP_INFO(get_logger(), "========================================");
    RCLCPP_INFO(get_logger(), "  Samples:       %zu", n);
    RCLCPP_INFO(get_logger(), "  Duration:      %.3f s", samples_.back().timestamp);
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
    RCLCPP_INFO(get_logger(), "  CSV file:       %s", current_file_.c_str());
    RCLCPP_INFO(get_logger(), "========================================");
    RCLCPP_INFO(get_logger(), "Plot with:");
    RCLCPP_INFO(get_logger(), "  python3 plot_trajectory.py %s", current_file_.c_str());
  }

  /// Estimate the phase lag by cross-correlating command and actual.
  double estimate_lag() const
  {
    const size_t n = samples_.size();
    if (n < 10) {
      return -1.0;
    }

    // Use the first full period of data.
    const double dt = samples_[1].timestamp - samples_[0].timestamp;
    const size_t period_samples =
      static_cast<size_t>(period_ / std::abs(dt));
    const size_t search_len = std::min(period_samples, n / 2);

    double best_corr = -1e9;
    int best_shift = 0;

    for (int shift = 0; shift < static_cast<int>(search_len); ++shift) {
      double sum = 0.0;
      size_t count = 0;
      for (size_t i = 0; i + shift < n; ++i) {
        sum += samples_[i].command_rad * samples_[i + shift].actual_rad;
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
  int baudrate_{1000};
  uint32_t sync_period_us_{10000};
  std::string joint_states_topic_;

  // ---- trajectory params ----
  double amplitude_{0.5};
  double period_{5.0};
  double frequency_{100.0};
  std::string waveform_{"sin"};
  size_t test_joint_{0};
  double duration_{0.0};
  double settle_time_{0.5};
  std::string output_file_;

  // ---- state ----
  std::unique_ptr<MiraculousArm> arm_;
  double dc_offset_{0.0};
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
