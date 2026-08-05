#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace arm_remote_control
{

namespace
{
bool is_sha256(const std::string & value)
{
  if (value.size() != 64) {
    return false;
  }
  for (const char character : value) {
    const bool digit = character >= '0' && character <= '9';
    const bool lower_hex = character >= 'a' && character <= 'f';
    if (!digit && !lower_hex) {
      return false;
    }
  }
  return true;
}
}  // namespace

class RemoteMotionWatchdog : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using SwitchController = controller_manager_msgs::srv::SwitchController;

  RemoteMotionWatchdog()
  : Node("remote_motion_watchdog")
  {
    heartbeat_topic_ = declare_parameter<std::string>(
      "heartbeat_topic", "/arm_remote_control/heartbeat");
    expected_fingerprint_ = declare_parameter<std::string>(
      "expected_profile_fingerprint", "");
    const auto soft_timeout_ms = declare_parameter<int>("soft_timeout_ms", 250);
    const auto monitor_period_ms = declare_parameter<int>("monitor_period_ms", 20);
    const auto cancel_hold_ms = declare_parameter<int>("cancel_hold_ms", 60);
    const auto retry_period_ms = declare_parameter<int>("retry_period_ms", 100);
    controller_name_ = declare_parameter<std::string>(
      "controller_name", "arm_controller");
    const auto controller_manager_name = declare_parameter<std::string>(
      "controller_manager_name", "/controller_manager");
    const auto trajectory_action = declare_parameter<std::string>(
      "trajectory_action", "/arm_controller/follow_joint_trajectory");

    if (
      heartbeat_topic_.empty() || controller_name_.empty() ||
      controller_manager_name.empty() || trajectory_action.empty() ||
      !is_sha256(expected_fingerprint_) || soft_timeout_ms <= 0 ||
      monitor_period_ms <= 0 || cancel_hold_ms < monitor_period_ms ||
      retry_period_ms <= 0)
    {
      throw std::invalid_argument(
              "invalid watchdog names, fingerprint, timeout, or timer period");
    }

    soft_timeout_ = std::chrono::milliseconds(soft_timeout_ms);
    cancel_hold_ = std::chrono::milliseconds(cancel_hold_ms);
    retry_period_ = std::chrono::milliseconds(retry_period_ms);

    auto heartbeat_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    heartbeat_qos.best_effort();
    heartbeat_qos.durability_volatile();
    heartbeat_subscription_ = create_subscription<std_msgs::msg::String>(
      heartbeat_topic_, heartbeat_qos,
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        receive_heartbeat(*message);
      });

    trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, trajectory_action);
    switch_client_ = create_client<SwitchController>(
      controller_manager_name + "/switch_controller");

    auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    status_qos.reliable();
    status_qos.transient_local();
    status_publisher_ = create_publisher<std_msgs::msg::String>("~/state", status_qos);

    stop_motion_service_ = create_service<std_srvs::srv::Trigger>(
      "~/stop_motion",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        begin_fault_stop("board-local stop-motion request");
        response->success = true;
        response->message = "stop requested and fault latched";
      });
    reset_fault_service_ = create_service<std_srvs::srv::Trigger>(
      "~/reset_fault",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        reset_fault(*response);
      });

    monitor_timer_ = create_wall_timer(
      std::chrono::milliseconds(monitor_period_ms),
      [this]() {monitor();});
    publish_state();

    RCLCPP_INFO(
      get_logger(),
      "Waiting for matching PC heartbeat on %s; soft stop timeout=%d ms",
      heartbeat_topic_.c_str(), soft_timeout_ms);
  }

private:
  enum class State
  {
    kWaitingForHeartbeat,
    kMonitoring,
    kStopping,
    kFaultLatched,
  };

  using SteadyClock = std::chrono::steady_clock;

  bool heartbeat_is_fresh() const
  {
    return heartbeat_seen_ &&
           SteadyClock::now() - last_heartbeat_ <= soft_timeout_;
  }

  void receive_heartbeat(const std_msgs::msg::String & message)
  {
    if (message.data != expected_fingerprint_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring heartbeat with a mismatched real-arm profile fingerprint");
      return;
    }

    last_heartbeat_ = SteadyClock::now();
    if (!heartbeat_seen_) {
      heartbeat_seen_ = true;
      if (state_ == State::kWaitingForHeartbeat) {
        state_ = State::kMonitoring;
        RCLCPP_INFO(
          get_logger(), "Matching heartbeat received; disconnect supervision armed");
        publish_state();
      }
    }
  }

  void monitor()
  {
    const auto now = SteadyClock::now();
    if (state_ == State::kMonitoring && !heartbeat_is_fresh()) {
      begin_fault_stop("external PC heartbeat timeout");
    }

    if (switch_request_pending_ && now >= switch_request_deadline_) {
      switch_request_pending_ = false;
      ++switch_request_generation_;
      RCLCPP_ERROR(
        get_logger(), "Controller deactivation response timed out; retrying");
    }

    if (
      (state_ == State::kStopping || state_ == State::kFaultLatched) &&
      !controller_stop_confirmed_ && !switch_request_pending_ &&
      now >= deactivate_not_before_ &&
      now - last_switch_request_ >= retry_period_)
    {
      request_controller_deactivation();
    }
  }

  void begin_fault_stop(const std::string & reason)
  {
    if (state_ == State::kStopping || state_ == State::kFaultLatched) {
      return;
    }

    state_ = State::kStopping;
    controller_stop_confirmed_ = false;
    const auto now = SteadyClock::now();
    deactivate_not_before_ = now + cancel_hold_;
    last_switch_request_ = now - retry_period_;

    RCLCPP_ERROR(
      get_logger(),
      "REMOTE MOTION FAULT LATCHED: %s. Cancelling the active trajectory, then "
      "deactivating %s. Communication recovery will not resume motion.",
      reason.c_str(), controller_name_.c_str());

    if (trajectory_client_->action_server_is_ready()) {
      try {
        trajectory_client_->async_cancel_all_goals();
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(), "Trajectory cancel request failed: %s", error.what());
        deactivate_not_before_ = now;
      }
    } else {
      RCLCPP_WARN(
        get_logger(), "Trajectory action server is unavailable; proceeding to "
        "controller deactivation");
      deactivate_not_before_ = now;
    }
    publish_state();
  }

  void request_controller_deactivation()
  {
    last_switch_request_ = SteadyClock::now();
    if (!switch_client_->service_is_ready()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "controller_manager switch service unavailable; the hardware hard "
        "watchdog remains the Quick Stop fallback");
      return;
    }

    auto request = std::make_shared<SwitchController::Request>();
    request->deactivate_controllers = {controller_name_};
    // A successful response is used as the proof that the trajectory
    // controller is no longer executing.  BEST_EFFORT can report success
    // after skipping a failed deactivation, so fail closed with STRICT.
    request->strictness = SwitchController::Request::STRICT;
    request->activate_asap = true;
    request->timeout.sec = 0;
    request->timeout.nanosec = 200000000;
    switch_request_pending_ = true;
    switch_request_deadline_ = SteadyClock::now() + std::chrono::milliseconds(500);
    const uint64_t request_generation = ++switch_request_generation_;
    switch_client_->async_send_request(
      request,
      [this, request_generation](rclcpp::Client<SwitchController>::SharedFuture future) {
        if (request_generation != switch_request_generation_) {
          return;
        }
        switch_request_pending_ = false;
        SwitchController::Response::SharedPtr response;
        try {
          response = future.get();
        } catch (const std::exception & error) {
          RCLCPP_ERROR(
            get_logger(), "Controller deactivation service failed: %s", error.what());
          return;
        }
        if (!response || !response->ok) {
          RCLCPP_ERROR(
            get_logger(), "Controller deactivation was not confirmed; retrying");
          return;
        }
        controller_stop_confirmed_ = true;
        state_ = State::kFaultLatched;
        RCLCPP_ERROR(
          get_logger(),
          "%s is inactive. The lost-PC fault remains latched; reconnecting the "
          "PC cannot resume the old trajectory.",
          controller_name_.c_str());
        publish_state();
      });
  }

  void reset_fault(std_srvs::srv::Trigger::Response & response)
  {
    if (state_ != State::kFaultLatched) {
      response.success = false;
      response.message = "no resettable fault is latched";
      return;
    }
    if (!controller_stop_confirmed_) {
      response.success = false;
      response.message = "controller deactivation has not been confirmed";
      return;
    }
    if (!heartbeat_is_fresh()) {
      response.success = false;
      response.message = "matching PC heartbeat is not stable";
      return;
    }

    state_ = State::kMonitoring;
    controller_stop_confirmed_ = false;
    response.success = true;
    response.message =
      "fault cleared; arm_controller remains inactive and requires board-local activation";
    RCLCPP_WARN(
      get_logger(),
      "Remote fault cleared locally. Motion remains disabled until arm_controller "
      "is explicitly reactivated on the board.");
    publish_state();
  }

  const char * state_name() const
  {
    switch (state_) {
      case State::kWaitingForHeartbeat:
        return "WAITING_FOR_HEARTBEAT";
      case State::kMonitoring:
        return "MONITORING";
      case State::kStopping:
        return "STOPPING";
      case State::kFaultLatched:
        return "FAULT_LATCHED";
    }
    return "UNKNOWN";
  }

  void publish_state()
  {
    if (!status_publisher_) {
      return;
    }
    std_msgs::msg::String message;
    message.data = state_name();
    status_publisher_->publish(message);
  }

  std::string heartbeat_topic_;
  std::string expected_fingerprint_;
  std::string controller_name_;
  std::chrono::milliseconds soft_timeout_{250};
  std::chrono::milliseconds cancel_hold_{60};
  std::chrono::milliseconds retry_period_{100};
  SteadyClock::time_point last_heartbeat_{};
  SteadyClock::time_point deactivate_not_before_{};
  SteadyClock::time_point last_switch_request_{};
  SteadyClock::time_point switch_request_deadline_{};
  bool heartbeat_seen_{false};
  bool switch_request_pending_{false};
  bool controller_stop_confirmed_{false};
  uint64_t switch_request_generation_{0};
  State state_{State::kWaitingForHeartbeat};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr heartbeat_subscription_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;
  rclcpp::Client<SwitchController>::SharedPtr switch_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_motion_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_fault_service_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
};

}  // namespace arm_remote_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<arm_remote_control::RemoteMotionWatchdog>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("remote_motion_watchdog"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
