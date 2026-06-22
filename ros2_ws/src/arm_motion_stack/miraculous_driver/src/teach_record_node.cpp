#include <array>
#include <chrono>
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

using namespace std::chrono_literals;

namespace
{
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
}  // namespace

class TeachRecordNode : public rclcpp::Node
{
public:
  TeachRecordNode() : Node("teach_record")
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
    record_rate_ = declare_parameter<double>("record_rate", 50.0);
    output_file_ = declare_parameter<std::string>("output_file", "");
    auto_record_ = declare_parameter<bool>("auto_record", false);

    auto node_ids = parse_int_list(node_ids_str, {1, 2, 3, 4, 5, 6});
    auto ppr = parse_double_list(ppr_str, std::stod(ppr_single_str));

    if (node_ids.size() != miraculous_driver::kArmJoints) {
      RCLCPP_FATAL(get_logger(),
        "node_ids must list %zu ids", miraculous_driver::kArmJoints);
      throw std::runtime_error("bad node_ids");
    }
    if (ppr.size() != miraculous_driver::kArmJoints) {
      RCLCPP_FATAL(get_logger(),
        "pulses_per_radian must list %zu values", miraculous_driver::kArmJoints);
      throw std::runtime_error("bad pulses_per_radian");
    }

    ArmConfig config;
    config.can_interface = can_interface_;
    config.baudrate = static_cast<CiaBaudrate_t>(baudrate_);
    config.sync_period_us = 0;
    config.read_rate_hz = record_rate_;
    for (size_t i = 0; i < miraculous_driver::kArmJoints; ++i) {
      JointConfig jc;
      jc.name = std::string("J") + std::to_string(i + 1);
      jc.node_id = static_cast<uint8_t>(node_ids[i]);
      jc.pulses_per_radian = ppr[i];
      config.joints.push_back(jc);
    }

    arm_ = std::make_unique<MiraculousArm>();
    if (!arm_->init_passive(config)) {
      RCLCPP_FATAL(get_logger(), "Failed to open motors (passive).");
      throw std::runtime_error("init_passive failed");
    }
    RCLCPP_INFO(get_logger(),
      "Teach mode ready (motors NOT enabled, free to drag). can=%s record_rate=%.1f",
      can_interface_.c_str(), record_rate_);

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::SystemDefaultsQoS());

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/start", std::bind(&TeachRecordNode::on_start, this,
        std::placeholders::_1, std::placeholders::_2));
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/stop", std::bind(&TeachRecordNode::on_stop, this,
        std::placeholders::_1, std::placeholders::_2));

    const auto period_us = static_cast<int64_t>(1.0e6 / record_rate_);
    timer_ = create_wall_timer(
      std::chrono::microseconds(period_us),
      std::bind(&TeachRecordNode::on_timer, this));

    if (auto_record_) {
      start_recording();
    }
  }

  ~TeachRecordNode() override
  {
    stop_recording();
    if (arm_) {
      arm_->shutdown();
    }
  }

private:
  void on_start(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    res->success = start_recording();
    res->message = res->success ? ("recording to " + current_file_) : "already recording or file error";
  }

  void on_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    res->success = stop_recording();
    res->message = res->success ? ("saved " + current_file_) : "was not recording";
  }

  bool start_recording()
  {
    if (recording_) {
      return false;
    }
    current_file_ = output_file_.empty() ? default_filename() : output_file_;
    ofs_.open(current_file_);
    if (!ofs_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Cannot open output file %s", current_file_.c_str());
      return false;
    }
    ofs_ << "timestamp,J1,J2,J3,J4,J5,J6\n";
    ofs_.precision(9);
    ofs_ << std::fixed;
    recording_ = true;
    record_start_ = now();
    RCLCPP_INFO(get_logger(), "Started recording to %s", current_file_.c_str());
    return true;
  }

  bool stop_recording()
  {
    if (!recording_) {
      return false;
    }
    recording_ = false;
    if (ofs_.is_open()) {
      ofs_.flush();
      ofs_.close();
    }
    RCLCPP_INFO(get_logger(), "Stopped recording (%s)", current_file_.c_str());
    return true;
  }

  void on_timer()
  {
    std::array<double, miraculous_driver::kArmJoints> pos{};
    if (!arm_->get_positions_rad(pos)) {
      return;
    }

    // Publish joint state for RViz / MoveIt current-state monitoring.
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name.reserve(miraculous_driver::kArmJoints);
    js.position.reserve(miraculous_driver::kArmJoints);
    for (size_t i = 0; i < miraculous_driver::kArmJoints; ++i) {
      js.name.push_back(std::string("J") + std::to_string(i + 1));
      js.position.push_back(pos[i]);
    }
    joint_pub_->publish(js);

    if (recording_ && ofs_.is_open()) {
      const double t = (now() - record_start_).seconds();
      ofs_ << t;
      for (size_t i = 0; i < miraculous_driver::kArmJoints; ++i) {
        ofs_ << "," << pos[i];
      }
      ofs_ << "\n";
    }
  }

  std::string default_filename() const
  {
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[64];
    std::strftime(buf, sizeof(buf), "teach_%Y%m%d_%H%M%S.csv", &tm);
    return std::string(buf);
  }

  std::string can_interface_;
  int baudrate_{1000};
  std::string joint_states_topic_;
  double record_rate_{50.0};
  std::string output_file_;
  bool auto_record_{false};

  std::unique_ptr<MiraculousArm> arm_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool recording_{false};
  std::ofstream ofs_;
  std::string current_file_;
  rclcpp::Time record_start_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeachRecordNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
