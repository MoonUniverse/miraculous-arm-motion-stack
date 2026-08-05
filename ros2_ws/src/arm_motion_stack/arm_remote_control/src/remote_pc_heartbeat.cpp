#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

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

class RemotePcHeartbeat : public rclcpp::Node
{
public:
  RemotePcHeartbeat()
  : Node("remote_pc_heartbeat")
  {
    const auto topic = declare_parameter<std::string>(
      "heartbeat_topic", "/arm_remote_control/heartbeat");
    profile_fingerprint_ = declare_parameter<std::string>(
      "profile_fingerprint", "");
    const auto period_ms = declare_parameter<int>("heartbeat_period_ms", 50);

    if (topic.empty() || !is_sha256(profile_fingerprint_) || period_ms <= 0) {
      throw std::invalid_argument(
              "heartbeat_topic must be non-empty, profile_fingerprint must be "
              "a lowercase SHA-256 value, and heartbeat_period_ms must be positive");
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.best_effort();
    qos.durability_volatile();
    publisher_ = create_publisher<std_msgs::msg::String>(topic, qos);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      [this]() {
        std_msgs::msg::String message;
        message.data = profile_fingerprint_;
        publisher_->publish(message);
      });

    RCLCPP_INFO(
      get_logger(), "Publishing remote-control heartbeat every %lld ms on %s",
      static_cast<long long>(period_ms), topic.c_str());
  }

private:
  std::string profile_fingerprint_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace arm_remote_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<arm_remote_control::RemotePcHeartbeat>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("remote_pc_heartbeat"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
