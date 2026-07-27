#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <sys/stat.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "miraculous_driver/miraculous_arm.hpp"

using miraculous_driver::ArmConfig;
using miraculous_driver::FeedbackSample;
using miraculous_driver::JointConfig;
using miraculous_driver::MiraculousArm;
using miraculous_driver::kArmJoints;

namespace
{
constexpr int kStartupFeedbackWindowMs = 500;

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

bool file_exists(const std::string & path)
{
  struct stat info {};
  return ::stat(path.c_str(), &info) == 0;
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
    const std::string joint_indices_str =
      declare_parameter<std::string>("joint_indices", "");
    joint_states_topic_ =
      declare_parameter<std::string>("joint_states_topic", "/arm_joint_states");
    record_rate_ = declare_parameter<double>("record_rate", 50.0);
    output_file_ = declare_parameter<std::string>("output_file", "");
    auto_record_ = declare_parameter<bool>("auto_record", false);
    feedback_timeout_ms_ = declare_parameter<int>("feedback_timeout_ms", 2);
    max_consecutive_misses_ = declare_parameter<int>("max_consecutive_misses", 10);
    overwrite_existing_ = declare_parameter<bool>("overwrite_existing", false);

    if (!std::isfinite(record_rate_) || record_rate_ <= 0.0 || record_rate_ > 200.0) {
      RCLCPP_FATAL(get_logger(), "record_rate must be in (0, 200] Hz");
      throw std::runtime_error("bad record_rate");
    }
    if (feedback_timeout_ms_ <= 0) {
      RCLCPP_FATAL(get_logger(), "feedback_timeout_ms must be greater than 0");
      throw std::runtime_error("bad feedback_timeout_ms");
    }
    if (max_consecutive_misses_ <= 0) {
      RCLCPP_FATAL(get_logger(), "max_consecutive_misses must be greater than 0");
      throw std::runtime_error("bad max_consecutive_misses");
    }

    const auto node_ids = parse_int_list(node_ids_str, {1, 2, 3, 4, 5, 6});
    const auto joint_indices =
      parse_int_list(joint_indices_str, default_joint_indices(node_ids.size()));
    if (node_ids.empty() || node_ids.size() > miraculous_driver::kArmJoints ||
      !validate_node_ids(node_ids))
    {
      RCLCPP_FATAL(get_logger(),
        "node_ids must list 1..%zu unique ids in range 1..127",
        miraculous_driver::kArmJoints);
      throw std::runtime_error("bad node_ids");
    }
    if (joint_indices.size() != node_ids.size() || !validate_joint_indices(joint_indices)) {
      RCLCPP_FATAL(get_logger(),
        "joint_indices must list %zu unique values in range 0..%zu",
        node_ids.size(), miraculous_driver::kArmJoints - 1);
      throw std::runtime_error("bad joint_indices");
    }

    ArmConfig config;
    config.can_interface = can_interface_;
    config.baudrate = static_cast<CiaBaudrate_t>(baudrate_);
    config.sync_period_us = 0;
    for (size_t i = 0; i < node_ids.size(); ++i) {
      const size_t joint_index = static_cast<size_t>(joint_indices[i]);
      JointConfig joint;
      joint.name = std::string("J") + std::to_string(joint_index + 1);
      joint.joint_index = joint_index;
      joint.node_id = static_cast<uint8_t>(node_ids[i]);
      config.joints.push_back(joint);
      joints_.push_back(joint);
    }

    arm_ = std::make_unique<MiraculousArm>();
    if (!arm_->init_passive(config)) {
      RCLCPP_FATAL(get_logger(),
        "Passive initialization failed; all configured drives must reach "
        "Ready to Switch On after motor shutdown");
      throw std::runtime_error("init_passive failed");
    }
    arm_->set_emcy_callback(
      [this](uint8_t node_id, uint16_t error_code, uint8_t error_reg) {
        emcy_node_.store(node_id, std::memory_order_relaxed);
        emcy_code_.store(error_code, std::memory_order_relaxed);
        emcy_reg_.store(error_reg, std::memory_order_relaxed);
        emcy_pending_.store(true, std::memory_order_release);
      });

    FeedbackSample startup_sample;
    if (!wait_for_startup_feedback(startup_sample)) {
      arm_->disable_voltage();
      RCLCPP_FATAL(get_logger(),
        "No complete fresh feedback set from all configured joints within %d ms",
        kStartupFeedbackWindowMs);
      throw std::runtime_error("startup feedback unavailable");
    }

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::SystemDefaultsQoS());
    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/start", std::bind(&TeachRecordNode::on_start, this,
        std::placeholders::_1, std::placeholders::_2));
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/stop", std::bind(&TeachRecordNode::on_stop, this,
        std::placeholders::_1, std::placeholders::_2));

    publish_sample(startup_sample);
    RCLCPP_INFO(get_logger(),
      "Teach mode ready: %zu drive(s) verified Ready to Switch On after shutdown, "
      "fresh synchronized feedback available. can=%s record_rate=%.1f Hz",
      joints_.size(), can_interface_.c_str(), record_rate_);

    if (auto_record_) {
      std::string error;
      if (!start_recording(error)) {
        RCLCPP_FATAL(get_logger(), "auto_record failed: %s", error.c_str());
        throw std::runtime_error("auto_record failed");
      }
    }

    const auto period_us = static_cast<int64_t>(1.0e6 / record_rate_);
    timer_ = create_wall_timer(
      std::chrono::microseconds(period_us),
      std::bind(&TeachRecordNode::on_timer, this));
  }

  ~TeachRecordNode() override
  {
    if (recording_) {
      finish_recording("node shutdown", true);
    }
    if (arm_) {
      arm_->set_emcy_callback({});
      arm_->shutdown();
    }
  }

private:
  void on_start(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::string error;
    response->success = start_recording(error);
    response->message =
      response->success ? ("recording V2 CSV to " + current_file_) : error;
  }

  void on_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!recording_) {
      response->success = false;
      response->message = "was not recording";
      return;
    }
    finish_recording("stop requested", false);
    response->success = true;
    response->message = last_stop_summary_;
  }

  bool wait_for_startup_feedback(FeedbackSample & sample)
  {
    const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::milliseconds(kStartupFeedbackWindowMs);
    while (std::chrono::steady_clock::now() < deadline) {
      if (emcy_pending_.load(std::memory_order_acquire) || arm_->has_fault()) {
        return false;
      }
      if (arm_->read_passive_feedback(sample, feedback_timeout_ms_)) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  }

  bool start_recording(std::string & error)
  {
    if (recording_) {
      error = "already recording";
      return false;
    }
    if (fault_latched_ || emcy_pending_.load(std::memory_order_acquire) ||
      arm_->has_fault())
    {
      handle_safety_fault();
      error = "fault is latched; restart teach_record after diagnosing the drive";
      return false;
    }
    if (!arm_->is_passive_ready()) {
      error =
        "cannot start: one or more drives are not Ready to Switch On; "
        "restart teach_record and inspect the drive state";
      return false;
    }

    FeedbackSample first_sample;
    if (!arm_->read_passive_feedback(first_sample, feedback_timeout_ms_)) {
      if (emcy_pending_.load(std::memory_order_acquire) || arm_->has_fault()) {
        handle_safety_fault();
        error = "EMCY/fault received while acquiring the first sample";
      } else {
        error = "cannot start: complete fresh feedback is unavailable";
      }
      return false;
    }

    std::string path;
    if (!resolve_output_path(path, error)) {
      return false;
    }
    ofs_.open(path, std::ios::out | std::ios::trunc);
    if (!ofs_.is_open()) {
      error = "cannot open output file " + path;
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      return false;
    }

    ofs_ << "timestamp,sample_index";
    for (const auto & joint : joints_) {
      ofs_ << "," << joint.name;
    }
    ofs_ << "\n";
    ofs_.precision(9);
    ofs_ << std::fixed;

    current_file_ = path;
    record_start_sync_ = first_sample.sync_time;
    last_record_sync_ = first_sample.sync_time;
    record_attempt_index_ = 0;
    recorded_rows_ = 0;
    missed_attempts_ = 0;
    consecutive_misses_ = 0;
    recording_ = true;
    write_sample(first_sample, 0);
    if (!ofs_) {
      finish_recording("failed to write first sample", true);
      error = last_stop_summary_;
      return false;
    }
    publish_sample(first_sample);
    RCLCPP_INFO(get_logger(), "Started V2 teach recording to %s", current_file_.c_str());
    return true;
  }

  void finish_recording(const std::string & reason, bool aborted)
  {
    if (!recording_) {
      return;
    }
    recording_ = false;
    if (ofs_.is_open()) {
      ofs_.flush();
      ofs_.close();
    }
    const double duration_s =
      std::chrono::duration<double>(last_record_sync_ - record_start_sync_).count();
    std::ostringstream summary;
    summary << (aborted ? "aborted " : "saved ") << current_file_
            << " rows=" << recorded_rows_
            << " misses=" << missed_attempts_
            << " duration=" << duration_s << " s"
            << " reason=" << reason;
    last_stop_summary_ = summary.str();
    if (aborted) {
      RCLCPP_ERROR(get_logger(), "%s", last_stop_summary_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "%s", last_stop_summary_.c_str());
    }
  }

  void on_timer()
  {
    if (emcy_pending_.load(std::memory_order_acquire) || arm_->has_fault()) {
      handle_safety_fault();
      return;
    }

    const bool recording_attempt = recording_;
    uint64_t sample_index = 0;
    if (recording_attempt) {
      sample_index = ++record_attempt_index_;
    }

    FeedbackSample sample;
    if (!arm_->read_passive_feedback(sample, feedback_timeout_ms_)) {
      if (emcy_pending_.load(std::memory_order_acquire) || arm_->has_fault()) {
        handle_safety_fault();
        return;
      }
      if (recording_attempt && recording_) {
        ++missed_attempts_;
        ++consecutive_misses_;
        RCLCPP_WARN(get_logger(),
          "Fresh feedback timeout: miss %d/%d (sample_index=%llu)",
          consecutive_misses_, max_consecutive_misses_,
          static_cast<unsigned long long>(sample_index));
        if (consecutive_misses_ >= max_consecutive_misses_) {
          finish_recording("consecutive fresh-feedback timeout limit reached", true);
        }
      }
      return;
    }

    if (emcy_pending_.load(std::memory_order_acquire) || arm_->has_fault()) {
      handle_safety_fault();
      return;
    }
    consecutive_misses_ = 0;
    publish_sample(sample);

    if (recording_attempt && recording_) {
      write_sample(sample, sample_index);
      if (!ofs_) {
        finish_recording("CSV write failed", true);
      }
    }
  }

  void publish_sample(const FeedbackSample & sample)
  {
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = now();
    joint_state.name.reserve(joints_.size());
    joint_state.position.reserve(joints_.size());
    joint_state.velocity.reserve(joints_.size());
    for (const auto & joint : joints_) {
      joint_state.name.push_back(joint.name);
      joint_state.position.push_back(sample.positions_rad[joint.joint_index]);
      joint_state.velocity.push_back(sample.velocities_rad_s[joint.joint_index]);
    }
    joint_pub_->publish(joint_state);
  }

  void write_sample(const FeedbackSample & sample, uint64_t sample_index)
  {
    const double timestamp =
      std::chrono::duration<double>(sample.sync_time - record_start_sync_).count();
    ofs_ << timestamp << "," << sample_index;
    for (const auto & joint : joints_) {
      ofs_ << "," << sample.positions_rad[joint.joint_index];
    }
    ofs_ << "\n";
    ++recorded_rows_;
    last_record_sync_ = sample.sync_time;
  }

  void handle_safety_fault()
  {
    if (fault_latched_) {
      return;
    }
    fault_latched_ = true;
    const unsigned int node = emcy_node_.load(std::memory_order_relaxed);
    const unsigned int code = emcy_code_.load(std::memory_order_relaxed);
    const unsigned int reg = emcy_reg_.load(std::memory_order_relaxed);
    std::ostringstream reason;
    reason << "EMCY/fault latched";
    if (emcy_pending_.load(std::memory_order_acquire)) {
      reason << " node=" << node << " code=0x" << std::hex << code
             << " reg=0x" << reg;
    }
    if (recording_) {
      finish_recording(reason.str(), true);
    }
    if (!arm_->disable_voltage()) {
      RCLCPP_ERROR(get_logger(),
        "Fault handling could not verify Switch On Disabled on every drive");
    }
    RCLCPP_ERROR(get_logger(), "%s; recording is locked until node restart",
      reason.str().c_str());
  }

  bool resolve_output_path(std::string & path, std::string & error) const
  {
    if (!output_file_.empty()) {
      path = output_file_;
      if (!overwrite_existing_ && file_exists(path)) {
        error = "output file already exists and overwrite_existing=false: " + path;
        return false;
      }
      return true;
    }

    const std::string base = default_filename();
    if (!file_exists(base)) {
      path = base;
      return true;
    }
    const auto dot = base.find_last_of('.');
    const std::string stem = dot == std::string::npos ? base : base.substr(0, dot);
    const std::string extension = dot == std::string::npos ? "" : base.substr(dot);
    for (size_t suffix = 1; suffix < 10000; ++suffix) {
      const std::string candidate =
        stem + "_" + std::to_string(suffix) + extension;
      if (!file_exists(candidate)) {
        path = candidate;
        return true;
      }
    }
    error = "cannot allocate a unique automatic output filename";
    return false;
  }

  std::string default_filename() const
  {
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char buffer[64];
    std::strftime(buffer, sizeof(buffer), "teach_%Y%m%d_%H%M%S.csv", &tm);
    return std::string(buffer);
  }

  std::string can_interface_;
  int baudrate_{1000};
  std::string joint_states_topic_;
  double record_rate_{50.0};
  std::string output_file_;
  bool auto_record_{false};
  int feedback_timeout_ms_{2};
  int max_consecutive_misses_{10};
  bool overwrite_existing_{false};
  std::vector<JointConfig> joints_;

  std::unique_ptr<MiraculousArm> arm_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool recording_{false};
  bool fault_latched_{false};
  std::ofstream ofs_;
  std::string current_file_;
  std::string last_stop_summary_;
  std::chrono::steady_clock::time_point record_start_sync_;
  std::chrono::steady_clock::time_point last_record_sync_;
  uint64_t record_attempt_index_{0};
  uint64_t recorded_rows_{0};
  uint64_t missed_attempts_{0};
  int consecutive_misses_{0};

  std::atomic<bool> emcy_pending_{false};
  std::atomic<uint8_t> emcy_node_{0};
  std::atomic<uint16_t> emcy_code_{0};
  std::atomic<uint8_t> emcy_reg_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeachRecordNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
