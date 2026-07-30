#ifndef MIRACULOUS_DRIVER__MIRACULOUS_SYSTEM_HPP_
#define MIRACULOUS_DRIVER__MIRACULOUS_SYSTEM_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"

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
  void apply_snapshot(const FeedbackSnapshot & snapshot);
  void fail_safe_stop(const std::string & reason);
  void shutdown_arm();

  ArmFactory arm_factory_;
  std::vector<std::string> joint_names_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  std::vector<double> position_commands_;
  std::array<bool, kArmJoints> configured_joints_{};
  std::array<double, kArmJoints> position_min_{};
  std::array<double, kArmJoints> position_max_{};

  std::unique_ptr<MiraculousArm> arm_;
  std::chrono::milliseconds feedback_stale_timeout_{30};
  bool active_{false};
  bool fault_latched_{false};
  bool stop_issued_{false};
  std::atomic<bool> emcy_latched_{false};
};

}  // namespace miraculous_driver

#endif  // MIRACULOUS_DRIVER__MIRACULOUS_SYSTEM_HPP_
