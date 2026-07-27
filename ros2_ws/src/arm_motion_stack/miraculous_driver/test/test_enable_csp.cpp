#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "miraculous_driver/miraculous_arm.hpp"

namespace miraculous_driver
{

struct FakeEvent
{
  std::string name;
  uint8_t node_id = 0;
  double value = 0.0;
};

struct FakeSdk
{
  std::vector<uint8_t> nodes;
  std::vector<FakeEvent> events;
  std::array<Cia402State_t, 128> states{};
  std::array<Cia402Mode_t, 128> modes{};
  std::array<float, 128> positions{};
  std::array<size_t, 128> target_writes{};
  std::array<size_t, 128> state_reads{};
  uint8_t missing_feedback_node = 0;
  uint8_t fail_target_node = 0;
  uint8_t fail_enable_node = 0;
  uint8_t fail_disable_voltage_node = 0;
  uint8_t fail_quick_stop_node = 0;
  uint8_t fail_set_mode_node = 0;
  uint8_t wrong_final_state_node = 0;
  size_t fail_sync_call = 0;
  size_t sync_calls = 0;
  bool fail_sync_start = false;
  bool feedback_pending = false;
  bool feedback_delivered = false;
  bool csp_active_seen_inside_sdk = false;
};

class MiraculousArmTestPeer
{
public:
  static void configure(
    MiraculousArm & arm, const std::vector<uint8_t> & nodes,
    uint32_t sync_period_us)
  {
    arm.config_ = ArmConfig{};
    arm.config_.sync_period_us = sync_period_us;
    arm.config_.joints.clear();
    arm.motors_.fill(nullptr);
    for (size_t i = 0; i < nodes.size(); ++i) {
      JointConfig joint;
      joint.name = "J" + std::to_string(i + 1);
      joint.joint_index = i;
      joint.node_id = nodes[i];
      joint.position_min = -10.0;
      joint.position_max = 10.0;
      arm.config_.joints.push_back(joint);
      arm.motors_[i] = reinterpret_cast<MiraMotor *>(
        static_cast<uintptr_t>(nodes[i]));
      arm.tpdo2_generation_[i].store(0, std::memory_order_relaxed);
    }
    arm.initialized_ = true;
    arm.passive_ = false;
    arm.csp_active_.store(false, std::memory_order_relaxed);
    arm.exclusive_sdk_io_.store(false, std::memory_order_relaxed);
    arm.sync_timer_running_ = sync_period_us != 0;
    arm.cache_valid_ = false;
  }

  static void deliver_tpdo(MiraculousArm & arm, uint8_t node_id)
  {
    const uint8_t data[8] = {};
    MiraculousArm::tpdo_trampoline(node_id, 2, data, sizeof(data), &arm);
  }

  static bool exclusive_io(const MiraculousArm & arm)
  {
    return arm.exclusive_sdk_io_.load(std::memory_order_acquire);
  }

  static bool timer_running(const MiraculousArm & arm)
  {
    return arm.sync_timer_running_;
  }

  static void detach(MiraculousArm & arm)
  {
    arm.initialized_ = false;
    arm.passive_ = false;
    arm.csp_active_.store(false, std::memory_order_relaxed);
    arm.exclusive_sdk_io_.store(false, std::memory_order_relaxed);
    arm.sync_timer_running_ = false;
    arm.motors_.fill(nullptr);
  }
};

namespace
{

FakeSdk g_fake;
MiraculousArm * g_arm = nullptr;

uint8_t node_id(MiraMotor * motor)
{
  return static_cast<uint8_t>(reinterpret_cast<uintptr_t>(motor));
}

void record(const char * name, uint8_t node = 0, double value = 0.0)
{
  g_fake.events.push_back(FakeEvent{name, node, value});
  if (g_arm && g_arm->is_csp_enabled()) {
    g_fake.csp_active_seen_inside_sdk = true;
  }
}

std::vector<size_t> event_indices(const std::string & name)
{
  std::vector<size_t> indices;
  for (size_t i = 0; i < g_fake.events.size(); ++i) {
    if (g_fake.events[i].name == name) {
      indices.push_back(i);
    }
  }
  return indices;
}

size_t first_event(const std::string & name, uint8_t node = 0)
{
  for (size_t i = 0; i < g_fake.events.size(); ++i) {
    if (g_fake.events[i].name == name &&
      (node == 0 || g_fake.events[i].node_id == node))
    {
      return i;
    }
  }
  return g_fake.events.size();
}

size_t last_event(const std::string & name)
{
  for (size_t i = g_fake.events.size(); i > 0; --i) {
    if (g_fake.events[i - 1].name == name) {
      return i - 1;
    }
  }
  return g_fake.events.size();
}

size_t count_event(const std::string & name, uint8_t node = 0)
{
  return static_cast<size_t>(std::count_if(
           g_fake.events.begin(), g_fake.events.end(),
           [&](const FakeEvent & event) {
             return event.name == name && (node == 0 || event.node_id == node);
           }));
}

class EnableCspTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    g_fake = FakeSdk{};
    g_fake.nodes = {1, 2};
    for (uint8_t node : g_fake.nodes) {
      g_fake.states[node] = CIA_STATE_SWITCH_ON_DISABLED;
      g_fake.modes[node] = CIA_MODE_NONE;
      g_fake.positions[node] = 0.1f * static_cast<float>(node);
    }
    MiraculousArmTestPeer::configure(arm_, g_fake.nodes, 10000);
    g_arm = &arm_;
  }

  void TearDown() override
  {
    g_arm = nullptr;
    MiraculousArmTestPeer::detach(arm_);
  }

  void use_nodes(const std::vector<uint8_t> & nodes, uint32_t sync_period_us = 10000)
  {
    g_fake.nodes = nodes;
    for (uint8_t node : nodes) {
      g_fake.states[node] = CIA_STATE_SWITCH_ON_DISABLED;
      g_fake.modes[node] = CIA_MODE_NONE;
      g_fake.positions[node] = 0.1f * static_cast<float>(node);
    }
    MiraculousArmTestPeer::configure(arm_, nodes, sync_period_us);
  }

  MiraculousArm arm_;
};

TEST_F(EnableCspTest, SuccessSeedsEveryAxisBeforeEnableAndRestartsTimerLast)
{
  ASSERT_TRUE(arm_.enable_csp());
  ASSERT_TRUE(arm_.is_csp_enabled());
  EXPECT_TRUE(MiraculousArmTestPeer::timer_running(arm_));
  EXPECT_FALSE(MiraculousArmTestPeer::exclusive_io(arm_));
  EXPECT_FALSE(g_fake.csp_active_seen_inside_sdk);

  const auto syncs = event_indices("sync_send");
  ASSERT_EQ(syncs.size(), 3u);
  const size_t first_enable = first_event("enable");
  const size_t last_position = last_event("get_position");
  EXPECT_LT(last_position, first_enable);

  std::vector<size_t> first_seed_writes;
  for (uint8_t node : g_fake.nodes) {
    std::vector<const FakeEvent *> writes;
    for (const auto & event : g_fake.events) {
      if (event.name == "target" && event.node_id == node) {
        writes.push_back(&event);
      }
    }
    ASSERT_EQ(writes.size(), 2u);
    EXPECT_DOUBLE_EQ(writes[0]->value, g_fake.positions[node]);
    EXPECT_DOUBLE_EQ(writes[1]->value, g_fake.positions[node]);
    first_seed_writes.push_back(first_event("target", node));
  }
  EXPECT_LT(*std::max_element(first_seed_writes.begin(), first_seed_writes.end()), syncs[1]);
  EXPECT_LT(syncs[1], first_enable);
  EXPECT_LT(last_event("get_state"), first_event("sync_start"));
  EXPECT_LT(syncs.back(), first_event("sync_start"));
  EXPECT_EQ(first_event("sync_stop"), 0u);
}

TEST_F(EnableCspTest, MissingFreshFeedbackNeverEnablesOrRestartsTimer)
{
  g_fake.missing_feedback_node = 2;

  EXPECT_FALSE(arm_.enable_csp());
  EXPECT_FALSE(arm_.is_csp_enabled());
  EXPECT_FALSE(MiraculousArmTestPeer::timer_running(arm_));
  EXPECT_EQ(count_event("enable"), 0u);
  EXPECT_EQ(count_event("sync_start"), 0u);
  EXPECT_EQ(count_event("disable_voltage", 1), 1u);
  EXPECT_EQ(count_event("disable_voltage", 2), 1u);
}

TEST_F(EnableCspTest, SeedTargetFailureRollsBackEveryPreparedAxis)
{
  g_fake.fail_target_node = 2;

  EXPECT_FALSE(arm_.enable_csp());
  EXPECT_FALSE(arm_.is_csp_enabled());
  EXPECT_EQ(count_event("enable"), 0u);
  EXPECT_EQ(count_event("disable_voltage", 1), 1u);
  EXPECT_EQ(count_event("disable_voltage", 2), 1u);
  EXPECT_EQ(count_event("sync_start"), 0u);
}

TEST_F(EnableCspTest, SecondMotorEnableFailureRollsBackAndSkipsLaterMotor)
{
  use_nodes({1, 2, 3});
  g_fake.fail_enable_node = 2;

  EXPECT_FALSE(arm_.enable_csp());
  EXPECT_FALSE(arm_.is_csp_enabled());
  EXPECT_EQ(count_event("enable", 1), 1u);
  EXPECT_EQ(count_event("enable", 2), 1u);
  EXPECT_EQ(count_event("enable", 3), 0u);
  EXPECT_EQ(count_event("quick_stop", 1), 1u);
  EXPECT_EQ(count_event("quick_stop", 2), 1u);
  EXPECT_EQ(count_event("disable_voltage", 1), 1u);
  EXPECT_EQ(count_event("disable_voltage", 2), 1u);
  EXPECT_EQ(count_event("disable_voltage", 3), 1u);

  const size_t target_count = count_event("target");
  std::array<double, kArmJoints> targets{};
  EXPECT_FALSE(arm_.set_targets_rad(targets));
  EXPECT_EQ(count_event("target"), target_count);
}

TEST_F(EnableCspTest, RollbackFailureIsLoggedAndDoesNotStopOtherAxes)
{
  g_fake.fail_enable_node = 2;
  g_fake.fail_disable_voltage_node = 1;
  testing::internal::CaptureStderr();

  EXPECT_FALSE(arm_.enable_csp());

  const std::string stderr_text = testing::internal::GetCapturedStderr();
  EXPECT_NE(stderr_text.find("original failure"), std::string::npos);
  EXPECT_NE(stderr_text.find("rollback failure"), std::string::npos);
  EXPECT_NE(stderr_text.find("node 1"), std::string::npos);
  EXPECT_EQ(count_event("disable_voltage", 2), 1u);
  EXPECT_FALSE(arm_.is_csp_enabled());
  EXPECT_TRUE(MiraculousArmTestPeer::exclusive_io(arm_));
  EXPECT_FALSE(MiraculousArmTestPeer::timer_running(arm_));

  // Playback and other callers may defensively call disable() after a failed
  // enable. An incomplete rollback must remain quarantined if the retry fails.
  arm_.disable();
  EXPECT_EQ(count_event("disable_voltage", 2), 2u);
  EXPECT_TRUE(MiraculousArmTestPeer::exclusive_io(arm_));
}

TEST_F(EnableCspTest, PreEnableSeedSyncFailureRollsBackWithoutEnabling)
{
  g_fake.fail_sync_call = 2;

  EXPECT_FALSE(arm_.enable_csp());
  EXPECT_FALSE(arm_.is_csp_enabled());
  EXPECT_EQ(count_event("enable"), 0u);
  EXPECT_EQ(count_event("disable_voltage", 1), 1u);
  EXPECT_EQ(count_event("disable_voltage", 2), 1u);
  EXPECT_EQ(count_event("sync_start"), 0u);
}

TEST_F(EnableCspTest, CspModeFailureRollsBackWithoutFaultReset)
{
  use_nodes({1, 2, 3});
  g_fake.fail_set_mode_node = 2;

  EXPECT_FALSE(arm_.enable_csp());
  EXPECT_FALSE(arm_.is_csp_enabled());
  EXPECT_EQ(count_event("enable"), 0u);
  EXPECT_EQ(count_event("fault_reset"), 0u);
  EXPECT_EQ(count_event("disable_voltage", 1), 1u);
  EXPECT_EQ(count_event("disable_voltage", 2), 1u);
  EXPECT_EQ(count_event("disable_voltage", 3), 1u);
}

TEST_F(EnableCspTest, FinalStateVerificationFailureRollsBackEnabledAxes)
{
  g_fake.wrong_final_state_node = 2;

  EXPECT_FALSE(arm_.enable_csp());
  EXPECT_FALSE(arm_.is_csp_enabled());
  EXPECT_EQ(count_event("quick_stop", 1), 1u);
  EXPECT_EQ(count_event("quick_stop", 2), 1u);
  EXPECT_EQ(count_event("disable_voltage", 1), 1u);
  EXPECT_EQ(count_event("disable_voltage", 2), 1u);
  EXPECT_EQ(count_event("sync_start"), 0u);
}

TEST_F(EnableCspTest, TimerRestartFailureRollsBackAndLeavesTimerStopped)
{
  g_fake.fail_sync_start = true;

  EXPECT_FALSE(arm_.enable_csp());
  EXPECT_FALSE(arm_.is_csp_enabled());
  EXPECT_FALSE(MiraculousArmTestPeer::timer_running(arm_));
  EXPECT_EQ(count_event("sync_start"), 1u);
  EXPECT_EQ(count_event("quick_stop", 1), 1u);
  EXPECT_EQ(count_event("quick_stop", 2), 1u);
  EXPECT_EQ(count_event("disable_voltage", 1), 1u);
  EXPECT_EQ(count_event("disable_voltage", 2), 1u);
}

}  // namespace
}  // namespace miraculous_driver

extern "C"
{

int __wrap_miraculous_motor_sync_stop(MiraMotor * motor)
{
  miraculous_driver::record("sync_stop", miraculous_driver::node_id(motor));
  return MRC_SUCCESS;
}

int __wrap_miraculous_motor_sync_start(MiraMotor * motor, uint32_t period_us)
{
  miraculous_driver::record(
    "sync_start", miraculous_driver::node_id(motor), period_us);
  return miraculous_driver::g_fake.fail_sync_start ?
         MRC_ERROR_CAN_SOCKET : MRC_SUCCESS;
}

int __wrap_miraculous_motor_sync_send(MiraMotor * motor)
{
  auto & fake = miraculous_driver::g_fake;
  ++fake.sync_calls;
  miraculous_driver::record(
    "sync_send", miraculous_driver::node_id(motor), fake.sync_calls);
  if (fake.fail_sync_call == fake.sync_calls) {
    return MRC_ERROR_CAN_SEND;
  }
  if (fake.sync_calls == 1) {
    fake.feedback_pending = true;
  }
  return MRC_SUCCESS;
}

int __wrap_miraculous_motor_poll(MiraMotor * motor, int timeout_ms)
{
  (void)timeout_ms;
  auto & fake = miraculous_driver::g_fake;
  miraculous_driver::record("poll", miraculous_driver::node_id(motor));
  if (fake.feedback_pending && !fake.feedback_delivered &&
    miraculous_driver::g_arm)
  {
    for (uint8_t node : fake.nodes) {
      if (node != fake.missing_feedback_node) {
        miraculous_driver::MiraculousArmTestPeer::deliver_tpdo(
          *miraculous_driver::g_arm, node);
      }
    }
    fake.feedback_delivered = true;
  }
  return MRC_SUCCESS;
}

int __wrap_miraculous_motor_get_state(
  MiraMotor * motor, Cia402State_t * state)
{
  const uint8_t node = miraculous_driver::node_id(motor);
  miraculous_driver::record("get_state", node);
  ++miraculous_driver::g_fake.state_reads[node];
  if (miraculous_driver::g_fake.wrong_final_state_node == node &&
    miraculous_driver::g_fake.state_reads[node] > 1 &&
    miraculous_driver::g_fake.states[node] == CIA_STATE_OPERATION_ENABLED)
  {
    *state = CIA_STATE_SWITCHED_ON;
  } else {
    *state = miraculous_driver::g_fake.states[node];
  }
  return MRC_SUCCESS;
}

int __wrap_miraculous_motor_shutdown(MiraMotor * motor)
{
  const uint8_t node = miraculous_driver::node_id(motor);
  miraculous_driver::record("shutdown", node);
  miraculous_driver::g_fake.states[node] = CIA_STATE_READY_TO_SWITCH_ON;
  return MRC_SUCCESS;
}

int __wrap_miraculous_motor_wait_state(
  MiraMotor * motor, Cia402State_t expected, int timeout_ms)
{
  (void)timeout_ms;
  const uint8_t node = miraculous_driver::node_id(motor);
  miraculous_driver::record("wait_state", node, expected);
  return miraculous_driver::g_fake.states[node] == expected ?
         MRC_SUCCESS : MRC_ERROR_TIMEOUT;
}

int __wrap_miraculous_motor_get_mode(
  MiraMotor * motor, Cia402Mode_t * mode)
{
  const uint8_t node = miraculous_driver::node_id(motor);
  miraculous_driver::record("get_mode", node);
  *mode = miraculous_driver::g_fake.modes[node];
  return MRC_SUCCESS;
}

int __wrap_miraculous_motor_set_mode(MiraMotor * motor, Cia402Mode_t mode)
{
  const uint8_t node = miraculous_driver::node_id(motor);
  miraculous_driver::record("set_mode", node, mode);
  if (miraculous_driver::g_fake.fail_set_mode_node == node) {
    return MRC_ERROR_MOTION_MODE_REJECTED;
  }
  miraculous_driver::g_fake.modes[node] = mode;
  return MRC_SUCCESS;
}

int __wrap_miraculous_motor_switch_on(MiraMotor * motor)
{
  const uint8_t node = miraculous_driver::node_id(motor);
  miraculous_driver::record("switch_on", node);
  miraculous_driver::g_fake.states[node] = CIA_STATE_SWITCHED_ON;
  return MRC_SUCCESS;
}

int __wrap_miraculous_motor_get_position_ex(
  MiraMotor * motor, float * position, PosUnit_t unit)
{
  (void)unit;
  const uint8_t node = miraculous_driver::node_id(motor);
  miraculous_driver::record("get_position", node);
  *position = miraculous_driver::g_fake.positions[node];
  return MRC_SUCCESS;
}

int __wrap_miraculous_motor_csp_set_target_ex(
  MiraMotor * motor, float target, PosUnit_t unit)
{
  (void)unit;
  const uint8_t node = miraculous_driver::node_id(motor);
  miraculous_driver::record("target", node, target);
  ++miraculous_driver::g_fake.target_writes[node];
  if (miraculous_driver::g_fake.fail_target_node == node &&
    miraculous_driver::g_fake.target_writes[node] == 1)
  {
    return MRC_ERROR_CAN_SEND;
  }
  return MRC_SUCCESS;
}

int __wrap_miraculous_motor_enable(MiraMotor * motor)
{
  const uint8_t node = miraculous_driver::node_id(motor);
  miraculous_driver::record("enable", node);
  if (miraculous_driver::g_fake.fail_enable_node == node) {
    return MRC_ERROR_MOTION_STATE_TRANSITION;
  }
  miraculous_driver::g_fake.states[node] = CIA_STATE_OPERATION_ENABLED;
  return MRC_SUCCESS;
}

int __wrap_miraculous_motor_quick_stop(MiraMotor * motor)
{
  const uint8_t node = miraculous_driver::node_id(motor);
  miraculous_driver::record("quick_stop", node);
  if (miraculous_driver::g_fake.fail_quick_stop_node == node) {
    return MRC_ERROR_CAN_SEND;
  }
  miraculous_driver::g_fake.states[node] = CIA_STATE_QUICK_STOP_ACTIVE;
  return MRC_SUCCESS;
}

int __wrap_miraculous_motor_disable_voltage(MiraMotor * motor)
{
  const uint8_t node = miraculous_driver::node_id(motor);
  miraculous_driver::record("disable_voltage", node);
  if (miraculous_driver::g_fake.fail_disable_voltage_node == node) {
    return MRC_ERROR_CAN_SEND;
  }
  miraculous_driver::g_fake.states[node] = CIA_STATE_SWITCH_ON_DISABLED;
  return MRC_SUCCESS;
}

int __wrap_miraculous_motor_fault_reset(MiraMotor * motor)
{
  miraculous_driver::record("fault_reset", miraculous_driver::node_id(motor));
  return MRC_SUCCESS;
}

}  // extern "C"
