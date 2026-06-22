#include <array>
#include <chrono>
#include <cmath>
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
struct TrajectoryPoint
{
  double t;  // seconds from start
  std::array<double, miraculous_driver::kArmJoints> pos;
};

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

std::vector<double> parse_double_list(const std::string & s, double single_def)
{
  std::vector<double> out;
  if (s.empty()) {
    out.assign(miraculous_driver::kArmJoints, single_def);
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
    out.assign(miraculous_driver::kArmJoints, out[0]);
  }
  return out;
}

/// Load a teach CSV (header: timestamp,J1..J6). Returns false on parse failure.
bool load_csv(const std::string & path, std::vector<TrajectoryPoint> & traj)
{
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    return false;
  }
  std::string line;
  std::getline(ifs, line);  // skip header
  while (std::getline(ifs, line)) {
    if (line.empty()) {
      continue;
    }
    std::stringstream ss(line);
    std::string tok;
    TrajectoryPoint pt{};
    pt.pos.fill(0.0);
    if (!std::getline(ss, tok, ',')) {
      continue;
    }
    try {
      pt.t = std::stod(tok);
    } catch (...) {
      continue;
    }
    for (size_t i = 0; i < miraculous_driver::kArmJoints; ++i) {
      if (!std::getline(ss, tok, ',')) {
        break;
      }
      try {
        pt.pos[i] = std::stod(tok);
      } catch (...) {}
    }
    traj.push_back(pt);
  }
  return !traj.empty();
}
}  // namespace

class PlaybackNode : public rclcpp::Node
{
public:
  PlaybackNode() : Node("playback")
  {
    can_interface_ = declare_parameter<std::string>("can_interface", "can0");
    baudrate_ = declare_parameter<int>("baudrate", 1000);
    const std::string node_ids_str =
      declare_parameter<std::string>("node_ids", "1,2,3,4,5,6");
    const std::string ppr_str =
      declare_parameter<std::string>("pulses_per_radian", "");
    const std::string ppr_single_str =
      declare_parameter<std::string>("pulses_per_radian_single", "0.0");
    joint_states_topic_ = declare_parameter<std::string>("joint_states_topic", "/arm_joint_states");
    input_file_ = declare_parameter<std::string>("input_file", "");
    speed_scale_ = declare_parameter<double>("speed_scale", 1.0);
    loop_ = declare_parameter<bool>("loop", false);

    auto node_ids = parse_int_list(node_ids_str, {1, 2, 3, 4, 5, 6});
    auto ppr = parse_double_list(ppr_str, std::stod(ppr_single_str));

    if (node_ids.size() != miraculous_driver::kArmJoints ||
      ppr.size() != miraculous_driver::kArmJoints)
    {
      RCLCPP_FATAL(get_logger(), "node_ids/pulses_per_radian must list %zu values",
        miraculous_driver::kArmJoints);
      throw std::runtime_error("bad params");
    }

    ArmConfig config;
    config.can_interface = can_interface_;
    config.baudrate = static_cast<CiaBaudrate_t>(baudrate_);
    config.sync_period_us = 0;
    config.read_rate_hz = 100.0;
    for (size_t i = 0; i < miraculous_driver::kArmJoints; ++i) {
      JointConfig jc;
      jc.name = std::string("J") + std::to_string(i + 1);
      jc.node_id = static_cast<uint8_t>(node_ids[i]);
      jc.pulses_per_radian = ppr[i];
      config.joints.push_back(jc);
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

    // Publish joint states continuously for monitoring (light timer).
    state_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&PlaybackNode::publish_state, this));

    RCLCPP_INFO(get_logger(),
      "Playback ready. can=%s input_file=%s speed_scale=%.2f",
      can_interface_.c_str(), input_file_.c_str(), speed_scale_);
  }

  ~PlaybackNode() override
  {
    stop_requested_ = true;
    if (play_thread_.joinable()) {
      play_thread_.join();
    }
    if (arm_) {
      arm_->disable();
      arm_->shutdown();
    }
  }

private:
  void on_play(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (playing_) {
      res->success = false;
      res->message = "already playing";
      return;
    }
    if (input_file_.empty()) {
      res->success = false;
      res->message = "input_file not set";
      return;
    }
    std::vector<TrajectoryPoint> traj;
    if (!load_csv(input_file_, traj)) {
      res->success = false;
      res->message = "failed to load trajectory from " + input_file_;
      return;
    }
    res->success = true;
    res->message = "starting playback (" + std::to_string(traj.size()) + " points)";
    playing_ = true;
    stop_requested_ = false;
    if (play_thread_.joinable()) {
      play_thread_.join();
    }
    play_thread_ = std::thread(&PlaybackNode::playback_loop, this, std::move(traj));
  }

  void on_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    stop_requested_ = true;
    res->success = true;
    res->message = "stop requested";
  }

  void playback_loop(std::vector<TrajectoryPoint> traj)
  {
    RCLCPP_INFO(get_logger(), "Enabling CSP and seeding current position...");
    if (!arm_->enable_csp()) {
      RCLCPP_ERROR(get_logger(), "enable_csp failed, aborting playback.");
      playing_ = false;
      return;
    }
    // Let the controller settle on the seeded position.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    do {
      const auto start = std::chrono::steady_clock::now();
      for (size_t k = 0; k < traj.size() && !stop_requested_; ++k) {
        const TrajectoryPoint & pt = traj[k];
        const double target_t =
          (k == 0) ? 0.0 : (pt.t - traj[0].t) / speed_scale_;
        const auto wake =
          start + std::chrono::microseconds(static_cast<int64_t>(target_t * 1.0e6));
        std::this_thread::sleep_until(wake);

        std::array<double, miraculous_driver::kArmJoints> targets{};
        for (size_t i = 0; i < miraculous_driver::kArmJoints; ++i) {
          targets[i] = pt.pos[i];
        }
        arm_->set_targets_rad(targets);
      }
    } while (loop_ && !stop_requested_);

    arm_->disable();
    playing_ = false;
    RCLCPP_INFO(get_logger(), "Playback finished.");
  }

  void publish_state()
  {
    std::array<double, miraculous_driver::kArmJoints> pos{};
    if (!arm_->get_positions_rad(pos)) {
      return;
    }
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name.reserve(miraculous_driver::kArmJoints);
    js.position.reserve(miraculous_driver::kArmJoints);
    for (size_t i = 0; i < miraculous_driver::kArmJoints; ++i) {
      js.name.push_back(std::string("J") + std::to_string(i + 1));
      js.position.push_back(pos[i]);
    }
    joint_pub_->publish(js);
  }

  std::string can_interface_;
  int baudrate_{1000};
  std::string joint_states_topic_;
  std::string input_file_;
  double speed_scale_{1.0};
  bool loop_{false};

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
