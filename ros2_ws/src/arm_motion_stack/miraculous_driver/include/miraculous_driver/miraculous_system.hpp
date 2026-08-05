#ifndef MIRACULOUS_DRIVER__MIRACULOUS_SYSTEM_HPP_
#define MIRACULOUS_DRIVER__MIRACULOUS_SYSTEM_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "std_msgs/msg/string.hpp"

#include "miraculous_driver/miraculous_arm.hpp"

namespace miraculous_driver
{

/// Fail-closed ros2_control SystemInterface for the real six-axis arm.
class MiraculousSystem : public hardware_interface::SystemInterface
{
public:
  using ArmFactory = std::function<std::unique_ptr<MiraculousArm>()>;

  MiraculousSystem();
  explicit MiraculousSystem(ArmFactory arm_factory);
  ~MiraculousSystem() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  double parse_double_param(const std::string & name, double default_value) const;
  std::string parse_string_param(
    const std::string & name, const std::string & default_value) const;
  int parse_int_param(const std::string & name, int default_value) const;
  bool parse_bool_param(const std::string & name, bool default_value) const;
  std::vector<int> parse_int_list_param(
    const std::string & name, const std::vector<int> & default_value) const;
  std::vector<double> parse_double_list_param(
    const std::string & name, const std::vector<double> & default_value) const;

  bool snapshot_is_safe(
    const FeedbackSnapshot & snapshot, bool require_fresh,
    std::string & reason) const;
  bool following_error_is_safe(
    const FeedbackSnapshot & snapshot, std::string & reason);
  void apply_snapshot(const FeedbackSnapshot & snapshot);
  void fail_safe_stop(const std::string & reason);
  void start_remote_watchdog();
  void stop_remote_watchdog() noexcept;
  bool remote_heartbeat_is_fresh() const;
  bool remote_command_advanced() const;
  void shutdown_arm();

  ArmFactory arm_factory_;
  std::vector<std::string> joint_names_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  std::vector<double> position_commands_;
  std::array<bool, kArmJoints> configured_joints_{};
  std::array<double, kArmJoints> position_min_{};
  std::array<double, kArmJoints> position_max_{};
  std::array<double, kArmJoints> last_sent_commands_{};

  std::unique_ptr<MiraculousArm> arm_;
  std::chrono::milliseconds feedback_stale_timeout_{30};
  double max_command_step_rad_{0.0};
  double max_following_error_rad_{0.0};
  size_t following_error_cycles_{0};
  size_t following_error_streak_{0};
  uint64_t last_following_sequence_{0};
  bool last_command_valid_{false};
  bool active_{false};
  bool fault_latched_{false};
  bool stop_issued_{false};
  std::atomic<bool> emcy_latched_{false};

  std::string remote_heartbeat_topic_;
  std::string remote_profile_fingerprint_;
  std::chrono::milliseconds remote_watchdog_timeout_{0};
  double remote_stop_velocity_threshold_rad_s_{0.02};
  std::atomic<int64_t> last_remote_heartbeat_ns_{0};
  std::atomic<bool> remote_heartbeat_seen_{false};
  std::atomic<bool> remote_stale_reported_{false};
  rclcpp::Node::SharedPtr remote_watchdog_node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr remote_heartbeat_subscription_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> remote_watchdog_executor_;
  std::thread remote_watchdog_thread_;
};

}  // namespace miraculous_driver

#endif  // MIRACULOUS_DRIVER__MIRACULOUS_SYSTEM_HPP_
