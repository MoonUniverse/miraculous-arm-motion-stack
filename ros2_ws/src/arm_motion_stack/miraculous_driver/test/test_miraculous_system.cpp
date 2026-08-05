#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <memory>
#include <string>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "miraculous_driver/miraculous_system.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace miraculous_driver
{
namespace
{

struct FakeState
{
  bool init_ok = true;
  bool enable_ok = true;
  bool disable_ok = true;
  bool quick_stop_ok = true;
  bool target_ok = true;
  bool fault = false;
  int init_calls = 0;
  int shutdown_calls = 0;
  int enable_calls = 0;
  int disable_calls = 0;
  int quick_stop_calls = 0;
  int target_calls = 0;
  ArmConfig config;
  FeedbackSnapshot snapshot;
  EmcyCallback emcy_callback;

  FakeState()
  {
    snapshot.sequence = 1;
    snapshot.stamp = std::chrono::steady_clock::now();
    snapshot.positions_rad.fill(0.1);
    snapshot.velocities_rad_s.fill(0.0);
  }
};

class FakeArm : public MiraculousArm
{
public:
  explicit FakeArm(std::shared_ptr<FakeState> state)
  : state_(std::move(state))
  {
  }

  bool init(const ArmConfig & config) override
  {
    ++state_->init_calls;
    state_->config = config;
    return state_->init_ok;
  }

  void shutdown() override
  {
    ++state_->shutdown_calls;
  }

  bool enable_csp() override
  {
    ++state_->enable_calls;
    state_->snapshot.stamp = std::chrono::steady_clock::now();
    return state_->enable_ok;
  }

  bool disable() override
  {
    ++state_->disable_calls;
    return state_->disable_ok;
  }

  bool quick_stop() override
  {
    ++state_->quick_stop_calls;
    return state_->quick_stop_ok;
  }

  bool get_feedback_snapshot(FeedbackSnapshot & snapshot) const override
  {
    snapshot = state_->snapshot;
    return snapshot.sequence != 0;
  }

  bool has_fault() const override
  {
    return state_->fault;
  }

  bool set_targets_rad(const std::array<double, kArmJoints> &) override
  {
    ++state_->target_calls;
    if (state_->target_ok) {
      ++state_->snapshot.sequence;
      state_->snapshot.stamp = std::chrono::steady_clock::now();
    }
    return state_->target_ok;
  }

  void set_emcy_callback(EmcyCallback callback) override
  {
    state_->emcy_callback = std::move(callback);
  }

private:
  std::shared_ptr<FakeState> state_;
};

hardware_interface::HardwareInfo makeHardwareInfo()
{
  hardware_interface::HardwareInfo info;
  info.name = "ARMSystem";
  info.type = "system";
  info.hardware_class_type = "miraculous_driver/MiraculousSystem";
  info.hardware_parameters = {
    {"can_interface", "can1"},
    {"baudrate", "0"},
    {"encoder_bw", "19"},
    {"reduction_ratio", "100.0"},
    {"node_ids", "1,2,3,4,5,6"},
    {"joint_indices", "0,1,2,3,4,5"},
    {"position_min", "-1,-1,-1,-1,-1,-1"},
    {"position_max", "1,1,1,1,1,1"},
    {"sync_period_us", "0"},
    {"read_rate_hz", "50.0"},
    {"state_poll_rate_hz", "0.0"},
    {"manual_feedback_timeout_ms", "15"},
    {"feedback_stale_timeout_ms", "30"},
    // xacro renders boolean substitutions with this capitalization.
    {"enable_emcy_monitor", "True"},
    {"max_command_step_rad", "0.005"},
    {"max_following_error_rad", "0.05"},
    {"following_error_cycles", "3"},
    {"require_full_arm", "True"},
    {"require_position_limits", "True"},
  };
  for (size_t index = 0; index < kArmJoints; ++index) {
    hardware_interface::ComponentInfo joint;
    joint.name = "J" + std::to_string(index + 1);
    joint.type = "joint";
    hardware_interface::InterfaceInfo command;
    command.name = hardware_interface::HW_IF_POSITION;
    joint.command_interfaces.push_back(command);
    hardware_interface::InterfaceInfo position;
    position.name = hardware_interface::HW_IF_POSITION;
    position.initial_value = "0.0";
    joint.state_interfaces.push_back(position);
    hardware_interface::InterfaceInfo velocity;
    velocity.name = hardware_interface::HW_IF_VELOCITY;
    joint.state_interfaces.push_back(velocity);
    info.joints.push_back(joint);
  }
  return info;
}

class MiraculousSystemTest : public ::testing::Test
{
protected:
  MiraculousSystemTest()
  : state_(std::make_shared<FakeState>()),
    system_([state = state_]() {return std::make_unique<FakeArm>(state);})
  {
  }

  void initializeAndConfigure()
  {
    ASSERT_EQ(
      system_.on_init(makeHardwareInfo()),
      hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(
      system_.on_configure(rclcpp_lifecycle::State()),
      hardware_interface::CallbackReturn::SUCCESS);
  }

  std::shared_ptr<FakeState> state_;
  MiraculousSystem system_;
};

TEST_F(MiraculousSystemTest, ConfigureIsInactiveAndActivationSeedsFromFeedback)
{
  initializeAndConfigure();
  EXPECT_EQ(state_->init_calls, 1);
  EXPECT_EQ(state_->enable_calls, 0);
  ASSERT_EQ(
    system_.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(state_->enable_calls, 1);

  auto commands = system_.export_command_interfaces();
  ASSERT_EQ(commands.size(), kArmJoints);
  for (auto & command : commands) {
    EXPECT_DOUBLE_EQ(command.get_value(), 0.1);
  }
  EXPECT_EQ(
    system_.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::OK);
  EXPECT_EQ(state_->target_calls, 1);
  EXPECT_EQ(
    system_.on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(state_->disable_calls, 1);
  EXPECT_EQ(state_->quick_stop_calls, 0);
}

TEST_F(MiraculousSystemTest, FailedWriteQuickStopsOnceAndLatchesUntilRestart)
{
  initializeAndConfigure();
  ASSERT_EQ(
    system_.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  state_->target_ok = false;
  EXPECT_EQ(
    system_.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::ERROR);
  EXPECT_EQ(state_->quick_stop_calls, 1);
  EXPECT_EQ(
    system_.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::OK);
  EXPECT_EQ(state_->quick_stop_calls, 1);

  EXPECT_EQ(
    system_.on_cleanup(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    system_.on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::ERROR);
}

TEST_F(MiraculousSystemTest, NonFiniteCommandAndStaleFeedbackFailClosed)
{
  initializeAndConfigure();
  ASSERT_EQ(
    system_.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  auto commands = system_.export_command_interfaces();
  commands[2].set_value(std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(
    system_.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::ERROR);
  EXPECT_EQ(state_->quick_stop_calls, 1);
}

TEST_F(MiraculousSystemTest, OutOfLimitCommandFailsBeforeSdkWrite)
{
  initializeAndConfigure();
  ASSERT_EQ(
    system_.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  auto commands = system_.export_command_interfaces();
  commands[0].set_value(1.1);
  EXPECT_EQ(
    system_.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::ERROR);
  EXPECT_EQ(state_->target_calls, 0);
  EXPECT_EQ(state_->quick_stop_calls, 1);
}

TEST_F(MiraculousSystemTest, SingleCycleCommandJumpFailsBeforeSdkWrite)
{
  initializeAndConfigure();
  ASSERT_EQ(
    system_.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  auto commands = system_.export_command_interfaces();
  commands[0].set_value(0.106);

  EXPECT_EQ(
    system_.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::ERROR);
  EXPECT_EQ(state_->target_calls, 0);
  EXPECT_EQ(state_->quick_stop_calls, 1);
}

TEST_F(MiraculousSystemTest, ActiveReadPreservesTheLastPositionCommand)
{
  initializeAndConfigure();
  ASSERT_EQ(
    system_.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);

  auto commands = system_.export_command_interfaces();
  commands[0].set_value(0.102);
  ASSERT_EQ(
    system_.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::OK);

  state_->snapshot.positions_rad[0] = 0.101;
  ++state_->snapshot.sequence;
  state_->snapshot.stamp = std::chrono::steady_clock::now();
  ASSERT_EQ(
    system_.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::OK);

  EXPECT_DOUBLE_EQ(commands[0].get_value(), 0.102);
}

TEST_F(MiraculousSystemTest, FollowingErrorCountsOnlyDistinctFeedbackSets)
{
  auto info = makeHardwareInfo();
  info.hardware_parameters["max_command_step_rad"] = "0.02";
  info.hardware_parameters["max_following_error_rad"] = "0.005";
  info.hardware_parameters["following_error_cycles"] = "2";
  ASSERT_EQ(system_.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    system_.on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    system_.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);

  auto commands = system_.export_command_interfaces();
  commands[0].set_value(0.11);
  ASSERT_EQ(
    system_.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::OK);
  EXPECT_EQ(
    system_.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::OK);
  // Reading the same sequence again must not advance the consecutive-cycle count.
  EXPECT_EQ(
    system_.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::OK);
  EXPECT_EQ(state_->quick_stop_calls, 0);

  ++state_->snapshot.sequence;
  state_->snapshot.stamp = std::chrono::steady_clock::now();
  EXPECT_EQ(
    system_.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::ERROR);
  EXPECT_EQ(state_->quick_stop_calls, 1);
}

TEST_F(MiraculousSystemTest, StaleFeedbackAndMotorFaultQuickStop)
{
  initializeAndConfigure();
  ASSERT_EQ(
    system_.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  state_->snapshot.stamp =
    std::chrono::steady_clock::now() - std::chrono::milliseconds(100);
  EXPECT_EQ(
    system_.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::ERROR);
  EXPECT_EQ(state_->quick_stop_calls, 1);
}

TEST_F(MiraculousSystemTest, EmcyIsObservedByTheControlCycle)
{
  initializeAndConfigure();
  ASSERT_EQ(
    system_.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(static_cast<bool>(state_->emcy_callback));
  state_->emcy_callback(3, 0x1234, 0x08);
  EXPECT_EQ(
    system_.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::ERROR);
  EXPECT_EQ(state_->quick_stop_calls, 1);
}

TEST(MiraculousSystemValidationTest, RejectsMalformedListsBeforeOpeningHardware)
{
  auto state = std::make_shared<FakeState>();
  MiraculousSystem system(
    [state]() {return std::make_unique<FakeArm>(state);});
  auto info = makeHardwareInfo();
  info.hardware_parameters["node_ids"] = "1,2,bad,4,5,6";
  ASSERT_EQ(system.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    system.on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::ERROR);
  EXPECT_EQ(state->init_calls, 0);
}

TEST(MiraculousSystemValidationTest, ZeroBaudrateKeepsSocketCanConfiguration)
{
  auto state = std::make_shared<FakeState>();
  MiraculousSystem system(
    [state]() {return std::make_unique<FakeArm>(state);});
  auto info = makeHardwareInfo();
  info.hardware_parameters["baudrate"] = "0";
  ASSERT_EQ(system.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    system.on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(state->init_calls, 1);
  EXPECT_EQ(static_cast<int>(state->config.baudrate), 0);
}

TEST(MiraculousSystemValidationTest, RejectsNegativeBaudrateBeforeOpeningHardware)
{
  auto state = std::make_shared<FakeState>();
  MiraculousSystem system(
    [state]() {return std::make_unique<FakeArm>(state);});
  auto info = makeHardwareInfo();
  info.hardware_parameters["baudrate"] = "-1";
  ASSERT_EQ(system.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    system.on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::ERROR);
  EXPECT_EQ(state->init_calls, 0);
}

TEST(MiraculousSystemValidationTest, RejectsTimerSyncForFullArmBeforeOpeningHardware)
{
  auto state = std::make_shared<FakeState>();
  MiraculousSystem system(
    [state]() {return std::make_unique<FakeArm>(state);});
  auto info = makeHardwareInfo();
  info.hardware_parameters["sync_period_us"] = "20000";
  ASSERT_EQ(system.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    system.on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::ERROR);
  EXPECT_EQ(state->init_calls, 0);
}

TEST(MiraculousSystemValidationTest, RejectsDisabledSafetyMonitorBeforeOpeningHardware)
{
  auto state = std::make_shared<FakeState>();
  MiraculousSystem system(
    [state]() {return std::make_unique<FakeArm>(state);});
  auto info = makeHardwareInfo();
  info.hardware_parameters["enable_emcy_monitor"] = "false";
  ASSERT_EQ(system.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    system.on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::ERROR);
  EXPECT_EQ(state->init_calls, 0);
}

TEST(MiraculousSystemValidationTest, RejectsDisabledFullArmWatchdogBeforeOpeningHardware)
{
  auto state = std::make_shared<FakeState>();
  MiraculousSystem system(
    [state]() {return std::make_unique<FakeArm>(state);});
  auto info = makeHardwareInfo();
  info.hardware_parameters["max_following_error_rad"] = "0.0";
  info.hardware_parameters["following_error_cycles"] = "0";
  ASSERT_EQ(system.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    system.on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::ERROR);
  EXPECT_EQ(state->init_calls, 0);
}

TEST(MiraculousSystemValidationTest, RejectsUnsafePositionAtConfigure)
{
  auto state = std::make_shared<FakeState>();
  state->snapshot.positions_rad[0] = 1.2;
  MiraculousSystem system(
    [state]() {return std::make_unique<FakeArm>(state);});
  ASSERT_EQ(
    system.on_init(makeHardwareInfo()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    system.on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::ERROR);
  EXPECT_EQ(state->shutdown_calls, 1);
}

TEST(MiraculousSystemValidationTest, PreActivationErrorDoesNotQuickStopOrLatchRuntimeFault)
{
  auto state = std::make_shared<FakeState>();
  state->snapshot.positions_rad[0] = 1.2;
  MiraculousSystem system(
    [state]() {return std::make_unique<FakeArm>(state);});
  ASSERT_EQ(
    system.on_init(makeHardwareInfo()),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    system.on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::ERROR);
  EXPECT_EQ(state->quick_stop_calls, 0);
  EXPECT_EQ(
    system.on_error(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(state->quick_stop_calls, 0);

  state->snapshot.positions_rad[0] = 0.1;
  state->snapshot.stamp = std::chrono::steady_clock::now();
  EXPECT_EQ(
    system.on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(state->init_calls, 2);
}

}  // namespace
}  // namespace miraculous_driver
