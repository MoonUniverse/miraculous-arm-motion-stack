#include "miraculous_driver/miraculous_arm.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace miraculous_driver
{
MiraculousArm::MiraculousArm()
{
  for (size_t i = 0; i < kArmJoints; ++i) {
    motors_[i] = nullptr;
    cached_states_[i] = CIA_STATE_NOT_READY_TO_SWITCH_ON;
    tpdo2_generation_[i].store(0, std::memory_order_relaxed);
  }
}

MiraculousArm::~MiraculousArm()
{
  shutdown();
}

bool MiraculousArm::validate_config(const ArmConfig & config, const char * caller) const
{
  if (config.joints.empty() || config.joints.size() > kArmJoints) {
    std::fprintf(stderr,
      "[miraculous_arm] %s: expected 1..%zu configured joints, got %zu\n",
      caller, kArmJoints, config.joints.size());
    return false;
  }
  std::array<bool, kArmJoints> seen{};
  std::array<bool, 128> seen_node_ids{};
  for (const auto & joint : config.joints) {
    if (joint.joint_index >= kArmJoints) {
      std::fprintf(stderr,
        "[miraculous_arm] %s: joint %s has invalid joint_index=%zu\n",
        caller, joint.name.c_str(), joint.joint_index);
      return false;
    }
    if (joint.node_id == 0 || joint.node_id > 127) {
      std::fprintf(stderr,
        "[miraculous_arm] %s: joint %s has invalid node_id=%u\n",
        caller, joint.name.c_str(), joint.node_id);
      return false;
    }
    if (seen_node_ids[joint.node_id]) {
      std::fprintf(stderr,
        "[miraculous_arm] %s: duplicate node_id=%u\n",
        caller, joint.node_id);
      return false;
    }
    if (seen[joint.joint_index]) {
      std::fprintf(stderr,
        "[miraculous_arm] %s: duplicate joint_index=%zu\n",
        caller, joint.joint_index);
      return false;
    }
    seen[joint.joint_index] = true;
    seen_node_ids[joint.node_id] = true;
  }
  return true;
}

// ============================ lifecycle ====================================

bool MiraculousArm::init(const ArmConfig & config)
{
  if (initialized_) {
    return true;
  }
  config_ = config;
  if (!validate_config(config_, "init")) {
    return false;
  }
  if (!open_motors()) {
    return false;
  }
  // Bring NMT to Operational so encoder objects (0x6064/0x606C) are readable
  // even before the power stage is enabled. Best-effort: a failed bootstrap on
  // one joint does not abort the others.
  for (const auto & joint : config_.joints) {
    if (miraculous_motor_bootstrap(motors_[joint.joint_index], 3000) < 0) {
      std::fprintf(stderr,
        "[miraculous_arm] init: bootstrap failed for joint %s (node %u)\n",
        joint.name.c_str(), joint.node_id);
    }
  }

  // CSP PDO/SYNC setup is process-lifetime configuration. Some drives do not
  // tolerate running csp_init again after a disable/enable cycle. Timer mode is
  // configured here but stopped before the read thread starts; enable_csp()
  // starts it only after every axis is safely seeded and Operation Enabled.
  const bool timer_sync = (config_.sync_period_us != 0);
  for (const auto & joint : config_.joints) {
    auto * motor = motors_[joint.joint_index];
    if (miraculous_motor_csp_init(motor, config_.sync_period_us, !timer_sync) < 0) {
      std::fprintf(stderr,
        "[miraculous_arm] init: csp_init failed joint %s\n",
        joint.name.c_str());
      close_motors(false);
      return false;
    }
  }
  if (timer_sync) {
    auto * sync_motor = first_motor_locked();
    if (!sync_motor || miraculous_motor_sync_stop(sync_motor) < 0) {
      std::fprintf(stderr,
        "[miraculous_arm] init: failed to stop timer SYNC before CSP enable\n");
      close_motors(false);
      return false;
    }
  }
  sync_timer_running_ = false;
  passive_ = false;
  initialized_ = true;
  start_read_thread();
  return true;
}

bool MiraculousArm::init_passive(const ArmConfig & config)
{
  if (initialized_) {
    return passive_;
  }
  config_ = config;
  if (!validate_config(config_, "init_passive")) {
    return false;
  }
  if (config_.sync_period_us != 0) {
    std::fprintf(stderr,
      "[miraculous_arm] init_passive: sync_period_us must be 0 for explicit samples\n");
    return false;
  }
  if (!open_motors()) {
    return false;
  }

  // Passive teach mode has a deliberately separate lifecycle from CSP. NMT is
  // brought Operational for PDO traffic, but no operation mode or CSP setup is
  // written to the drive.
  for (const auto & joint : config_.joints) {
    if (miraculous_motor_bootstrap(motors_[joint.joint_index], 3000) < 0) {
      std::fprintf(stderr,
        "[miraculous_arm] init_passive: bootstrap failed for joint %s (node %u)\n",
        joint.name.c_str(), joint.node_id);
      close_motors(true);
      return false;
    }
  }
  if (!enter_passive_ready_locked(1000)) {
    std::fprintf(stderr,
      "[miraculous_arm] init_passive: failed to verify Ready to Switch On\n");
    close_motors(true);
    return false;
  }

  passive_ = true;
  initialized_ = true;
  passive_sample_sequence_ = 0;
  return true;
}

void MiraculousArm::shutdown()
{
  stop_read_thread();
  if (initialized_) {
    csp_active_ = false;
    exclusive_sdk_io_ = false;
    {
      std::lock_guard<std::mutex> sdk_lock(sdk_mutex_);
      if (passive_) {
        // Do not send Shutdown (0x0006) here: it can leave Switch On Disabled.
        // Passive mode must remain at controlword 0x0000 until handles close.
        disable_voltage_locked(1000);
      } else {
        for (auto * motor : motors_) {
          if (motor) {
            miraculous_motor_shutdown(motor);
          }
        }
      }
      close_motors(false);
    }
    initialized_ = false;
    passive_ = false;
  }
}

bool MiraculousArm::open_motors()
{
  for (const auto & joint : config_.joints) {
    motors_[joint.joint_index] = miraculous_motor_open(
      config_.can_interface.c_str(), config_.baudrate,
      joint.node_id);
    if (!motors_[joint.joint_index]) {
      std::fprintf(stderr,
        "[miraculous_arm] open: failed to open joint %s (node %u) on %s\n",
        joint.name.c_str(), joint.node_id, config_.can_interface.c_str());
      close_motors(false);
      return false;
    }
    // Radian/velocity conversion parameters used by the _ex APIs. Defaults
    // match the SDK (19-bit encoder, 100:1 gear).
    miraculous_motor_set_encoder_bw(motors_[joint.joint_index], config_.encoder_bw);
    miraculous_motor_set_reduction_ratio(
      motors_[joint.joint_index], static_cast<float>(config_.reduction_ratio));
    miraculous_motor_set_tpdo_callback(
      motors_[joint.joint_index], &MiraculousArm::tpdo_trampoline, this);
  }
  // EMCY monitoring via the SDK's dedicated per-bus dispatcher. Registering on
  // one motor covers every node on the interface (the callback receives the
  // node id). Never touch miraculous_can_set_recv_callback: the CANopen master
  // owns that slot for its RX dispatch, and overriding it kills the TPDO
  // position cache, heartbeat and EMCY handling bus-wide.
  if (config_.enable_emcy_monitor) {
    for (auto * motor : motors_) {
      if (motor) {
        miraculous_motor_set_emcy_callback(motor, &MiraculousArm::emcy_trampoline, this);
        break;
      }
    }
  }
  return true;
}

void MiraculousArm::close_motors(bool force_disable_voltage)
{
  if (force_disable_voltage) {
    disable_voltage_locked(1000);
  }

  for (auto * motor : motors_) {
    if (motor) {
      // EMCY is registered bus-wide, so clearing it once covers this interface.
      miraculous_motor_set_emcy_callback(motor, nullptr, nullptr);
      if (config_.sync_period_us != 0) {
        miraculous_motor_sync_stop(motor);
      }
      break;
    }
  }
  for (auto * motor : motors_) {
    if (motor) {
      miraculous_motor_set_tpdo_callback(motor, nullptr, nullptr);
    }
  }
  for (auto *& motor : motors_) {
    if (motor) {
      miraculous_motor_close(motor);
      motor = nullptr;
    }
  }
  sync_timer_running_ = false;
}

// ============================ PDS state machine ============================

MiraMotor * MiraculousArm::first_motor_locked() const
{
  for (auto * motor : motors_) {
    if (motor) {
      return motor;
    }
  }
  return nullptr;
}

bool MiraculousArm::acquire_fresh_seed_positions_locked(
  std::array<double, kArmJoints> & positions, int timeout_ms)
{
  auto * sync_motor = first_motor_locked();
  if (!sync_motor || timeout_ms <= 0) {
    return false;
  }

  // Drain TPDOs queued before this transaction, then require every configured
  // joint's generation to advance after the controlled SYNC below.
  const int drain_ret = miraculous_motor_poll(sync_motor, 0);
  if (drain_ret < 0) {
    std::fprintf(stderr,
      "[miraculous_arm] enable_csp: seed feedback drain failed: %s (%d)\n",
      mrc_strerror(drain_ret), drain_ret);
    return false;
  }

  std::array<uint64_t, kArmJoints> baseline{};
  for (const auto & joint : config_.joints) {
    baseline[joint.joint_index] =
      tpdo2_generation_[joint.joint_index].load(std::memory_order_acquire);
  }

  if (miraculous_motor_sync_send(sync_motor) < 0) {
    std::fprintf(stderr,
      "[miraculous_arm] enable_csp: seed feedback SYNC failed\n");
    return false;
  }

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  const auto all_fresh = [&]() {
      for (const auto & joint : config_.joints) {
        if (tpdo2_generation_[joint.joint_index].load(std::memory_order_acquire) <=
          baseline[joint.joint_index])
        {
          return false;
        }
      }
      return true;
    };

  while (!all_fresh()) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      for (const auto & joint : config_.joints) {
        if (tpdo2_generation_[joint.joint_index].load(std::memory_order_acquire) <=
          baseline[joint.joint_index])
        {
          std::fprintf(stderr,
            "[miraculous_arm] enable_csp: fresh seed feedback timed out joint %s "
            "node %u\n", joint.name.c_str(), joint.node_id);
        }
      }
      return false;
    }
    const auto remaining_us =
      std::chrono::duration_cast<std::chrono::microseconds>(deadline - now).count();
    const int remaining_ms =
      std::max(1, static_cast<int>((remaining_us + 999) / 1000));
    const int poll_ret = miraculous_motor_poll(sync_motor, remaining_ms);
    if (poll_ret < 0) {
      std::fprintf(stderr,
        "[miraculous_arm] enable_csp: seed feedback poll failed: %s (%d)\n",
        mrc_strerror(poll_ret), poll_ret);
      return false;
    }
  }

  std::array<double, kArmJoints> fresh_positions{};
  for (const auto & joint : config_.joints) {
    float position_rad = 0.0f;
    const int ret = miraculous_motor_get_position_ex(
      motors_[joint.joint_index], &position_rad, POS_UNIT_RADIAN);
    if (ret < 0) {
      std::fprintf(stderr,
        "[miraculous_arm] enable_csp: fresh position read failed joint %s node %u: "
        "%s (%d)\n", joint.name.c_str(), joint.node_id, mrc_strerror(ret), ret);
      return false;
    }
    fresh_positions[joint.joint_index] = static_cast<double>(position_rad);
  }

  positions = fresh_positions;
  return true;
}

bool MiraculousArm::rollback_csp_enable_locked(
  const std::array<CspEnableStage, kArmJoints> & stages)
{
  bool rollback_ok = true;
  csp_active_.store(false, std::memory_order_release);

  // Timerfd only emits SYNC when poll() services it. Stop it before issuing any
  // state commands, and keep the read loop quarantined if stopping fails.
  if (config_.sync_period_us != 0) {
    auto * sync_motor = first_motor_locked();
    const int ret = sync_motor ? miraculous_motor_sync_stop(sync_motor) :
      MRC_ERROR_NOT_INIT;
    if (ret < 0) {
      std::fprintf(stderr,
        "[miraculous_arm] enable_csp rollback failure: timer SYNC stop: %s (%d)\n",
        mrc_strerror(ret), ret);
      rollback_ok = false;
    } else {
      sync_timer_running_ = false;
    }
  }

  // An Enable Operation write may have reached the drive even when the SDK
  // reports a timeout. Quick-stop every such axis, then continue to the common
  // disable-voltage pass regardless of individual failures.
  for (const auto & joint : config_.joints) {
    const size_t i = joint.joint_index;
    if (stages[i] < CspEnableStage::kEnableAttempted || !motors_[i]) {
      continue;
    }
    const int ret = miraculous_motor_quick_stop(motors_[i]);
    if (ret < 0) {
      std::fprintf(stderr,
        "[miraculous_arm] enable_csp rollback failure: quick stop joint %s node %u: "
        "%s (%d)\n", joint.name.c_str(), joint.node_id, mrc_strerror(ret), ret);
      rollback_ok = false;
    }
  }

  // Disable every configured drive, including axes not yet reached by the
  // preparation loop. Their pre-call state is not trusted, so an arm-wide
  // failure must not leave an unvisited axis active.
  for (const auto & joint : config_.joints) {
    const size_t i = joint.joint_index;
    if (!motors_[i]) {
      continue;
    }
    const int ret = miraculous_motor_disable_voltage(motors_[i]);
    if (ret < 0) {
      std::fprintf(stderr,
        "[miraculous_arm] enable_csp rollback failure: disable voltage joint %s "
        "node %u: %s (%d)\n",
        joint.name.c_str(), joint.node_id, mrc_strerror(ret), ret);
      rollback_ok = false;
    }
  }

  for (const auto & joint : config_.joints) {
    const size_t i = joint.joint_index;
    if (!motors_[i]) {
      continue;
    }
    const int ret = miraculous_motor_wait_state(
      motors_[i], CIA_STATE_SWITCH_ON_DISABLED, 1000);
    if (ret < 0) {
      std::fprintf(stderr,
        "[miraculous_arm] enable_csp rollback failure: state verification joint %s "
        "node %u: %s (%d)\n",
        joint.name.c_str(), joint.node_id, mrc_strerror(ret), ret);
      rollback_ok = false;
      continue;
    }
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    cached_states_[i] = CIA_STATE_SWITCH_ON_DISABLED;
  }

  return rollback_ok;
}

bool MiraculousArm::enable_csp()
{
  if (!initialized_ || passive_) {
    return false;
  }
  if (csp_active_.load(std::memory_order_acquire)) {
    return true;
  }

  const bool timer_sync = (config_.sync_period_us != 0);

  // Publish exclusive ownership before waiting for sdk_mutex_. read_loop()
  // rechecks this flag after taking the same mutex, closing the check/lock race.
  exclusive_sdk_io_.store(true, std::memory_order_release);
  std::unique_lock<std::mutex> sdk_lock(sdk_mutex_);
  if (csp_active_.load(std::memory_order_acquire)) {
    exclusive_sdk_io_.store(!timer_sync, std::memory_order_release);
    return true;
  }

  csp_active_.store(false, std::memory_order_release);
  std::array<CspEnableStage, kArmJoints> stages{};
  stages.fill(CspEnableStage::kUntouched);

  const auto fail = [&](const char * step, const JointConfig * joint, int ret) {
      if (joint) {
        std::fprintf(stderr,
          "[miraculous_arm] enable_csp original failure: %s joint %s node %u: "
          "%s (%d)\n", step, joint->name.c_str(), joint->node_id,
          mrc_strerror(ret), ret);
      } else {
        std::fprintf(stderr,
          "[miraculous_arm] enable_csp original failure: %s: %s (%d)\n",
          step, mrc_strerror(ret), ret);
      }
      const bool rollback_ok = rollback_csp_enable_locked(stages);
      // If rollback is incomplete, retain exclusive ownership so read_loop()
      // cannot service timerfd or send inactive-manual feedback SYNC frames.
      exclusive_sdk_io_.store(!rollback_ok, std::memory_order_release);
      return false;
    };

  auto * sync_motor = first_motor_locked();
  if (!sync_motor) {
    return fail("no configured SYNC motor", nullptr, MRC_ERROR_NOT_INIT);
  }
  if (timer_sync) {
    const int ret = miraculous_motor_sync_stop(sync_motor);
    if (ret < 0) {
      return fail("pause timer SYNC", nullptr, ret);
    }
    sync_timer_running_ = false;
  }

  // Prepare every axis without entering Operation Enabled. Do not call
  // full_enable(): it combines all state transitions and automatically resets
  // faults, neither of which is acceptable inside this arm-wide transaction.
  for (const auto & joint : config_.joints) {
    const size_t i = joint.joint_index;
    auto * motor = motors_[i];
    stages[i] = CspEnableStage::kStateChangeAttempted;

    Cia402State_t initial = CIA_STATE_NOT_READY_TO_SWITCH_ON;
    int ret = miraculous_motor_get_state(motor, &initial);
    if (ret < 0) {
      return fail("read initial state", &joint, ret);
    }
    if (initial == CIA_STATE_FAULT || initial == CIA_STATE_FAULT_REACTION_ACTIVE) {
      return fail("drive fault requires explicit reset", &joint, MRC_ERROR_MOTION_FAULT);
    }

    ret = miraculous_motor_shutdown(motor);
    if (ret < 0) {
      return fail("Shutdown to Ready to Switch On", &joint, ret);
    }
    ret = miraculous_motor_wait_state(motor, CIA_STATE_READY_TO_SWITCH_ON, 1000);
    if (ret < 0) {
      return fail("verify Ready to Switch On", &joint, ret);
    }
    stages[i] = CspEnableStage::kReadyToSwitchOn;

    Cia402Mode_t mode = CIA_MODE_NONE;
    ret = miraculous_motor_get_mode(motor, &mode);
    if (ret < 0) {
      return fail("read CSP mode", &joint, ret);
    }
    if (mode != CIA_MODE_CSP) {
      ret = miraculous_motor_set_mode(motor, CIA_MODE_CSP);
      if (ret < 0) {
        return fail("set CSP mode", &joint, ret);
      }
      ret = miraculous_motor_get_mode(motor, &mode);
      if (ret < 0) {
        return fail("verify CSP mode", &joint, ret);
      }
      if (mode != CIA_MODE_CSP) {
        return fail("CSP mode rejected", &joint, MRC_ERROR_MOTION_MODE_REJECTED);
      }
    }
    stages[i] = CspEnableStage::kModeConfirmed;
  }

  for (const auto & joint : config_.joints) {
    const size_t i = joint.joint_index;
    int ret = miraculous_motor_switch_on(motors_[i]);
    if (ret < 0) {
      return fail("Switch On", &joint, ret);
    }
    ret = miraculous_motor_wait_state(motors_[i], CIA_STATE_SWITCHED_ON, 1000);
    if (ret < 0) {
      return fail("verify Switched On", &joint, ret);
    }
    stages[i] = CspEnableStage::kSwitchedOn;
  }

  // All axes are now explicitly below Operation Enabled. Acquire one complete
  // post-SYNC TPDO generation set and record every position before writing any
  // seed target.
  std::array<double, kArmJoints> seed_positions{};
  if (!acquire_fresh_seed_positions_locked(seed_positions, 50)) {
    return fail("acquire fresh seed feedback", nullptr, MRC_ERROR_TIMEOUT);
  }

  for (const auto & joint : config_.joints) {
    const size_t i = joint.joint_index;
    const int ret = miraculous_motor_csp_set_target_ex(
      motors_[i], static_cast<float>(seed_positions[i]), POS_UNIT_RADIAN);
    if (ret < 0) {
      return fail("write pre-enable seed target", &joint, ret);
    }
    stages[i] = CspEnableStage::kSeedWritten;
  }
  int ret = miraculous_motor_sync_send(sync_motor);
  if (ret < 0) {
    return fail("latch pre-enable seed targets", nullptr, ret);
  }

  for (const auto & joint : config_.joints) {
    const size_t i = joint.joint_index;
    stages[i] = CspEnableStage::kEnableAttempted;
    ret = miraculous_motor_enable(motors_[i]);
    if (ret < 0) {
      return fail("Enable Operation", &joint, ret);
    }
    ret = miraculous_motor_wait_state(
      motors_[i], CIA_STATE_OPERATION_ENABLED, 4000);
    if (ret < 0) {
      return fail("verify Operation Enabled", &joint, ret);
    }
    stages[i] = CspEnableStage::kOperationEnabled;
  }

  // Reassert the identical hold target after all axes are enabled, then apply
  // one controlled arm-wide edge before allowing timer/read-thread activity.
  for (const auto & joint : config_.joints) {
    const size_t i = joint.joint_index;
    ret = miraculous_motor_csp_set_target_ex(
      motors_[i], static_cast<float>(seed_positions[i]), POS_UNIT_RADIAN);
    if (ret < 0) {
      return fail("write post-enable hold target", &joint, ret);
    }
  }
  ret = miraculous_motor_sync_send(sync_motor);
  if (ret < 0) {
    return fail("latch post-enable hold targets", nullptr, ret);
  }

  for (const auto & joint : config_.joints) {
    Cia402State_t state = CIA_STATE_NOT_READY_TO_SWITCH_ON;
    ret = miraculous_motor_get_state(motors_[joint.joint_index], &state);
    if (ret < 0) {
      return fail("final state read", &joint, ret);
    }
    if (state != CIA_STATE_OPERATION_ENABLED) {
      return fail(
        "final state is not Operation Enabled", &joint,
        MRC_ERROR_MOTION_STATE_TRANSITION);
    }
  }

  if (timer_sync) {
    ret = miraculous_motor_sync_start(sync_motor, config_.sync_period_us);
    if (ret < 0) {
      return fail("resume timer SYNC", nullptr, ret);
    }
    sync_timer_running_ = true;
  }

  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    for (const auto & joint : config_.joints) {
      const size_t i = joint.joint_index;
      cached_pos_rad_[i] = seed_positions[i];
      cached_states_[i] = CIA_STATE_OPERATION_ENABLED;
    }
    cache_valid_ = true;
  }
  passive_ = false;
  csp_active_.store(true, std::memory_order_release);
  exclusive_sdk_io_.store(!timer_sync, std::memory_order_release);
  return true;
}

bool MiraculousArm::enable()
{
  if (!initialized_) {
    return false;
  }
  std::lock_guard<std::mutex> sdk_lock(sdk_mutex_);
  for (const auto & joint : config_.joints) {
    if (miraculous_motor_full_enable(motors_[joint.joint_index]) < 0) {
      return false;
    }
  }
  return true;
}

void MiraculousArm::disable()
{
  if (!initialized_) {
    return;
  }
  const bool was_active = csp_active_.exchange(false, std::memory_order_acq_rel);
  bool release_exclusive_io = true;
  {
    std::lock_guard<std::mutex> sdk_lock(sdk_mutex_);
    // A caller may defensively call disable() after enable_csp() already
    // performed rollback. If that rollback was incomplete, exclusive I/O is a
    // quarantine: retry the stronger verified disable-voltage path and retain
    // the quarantine if any axis is still not confirmed safe.
    if (!was_active && exclusive_sdk_io_.load(std::memory_order_acquire)) {
      release_exclusive_io = disable_voltage_locked(1000);
    } else {
      // Keep the process-lifetime CSP/SYNC configuration intact so the next
      // enable_csp() can start without calling csp_init again.
      for (const auto & joint : config_.joints) {
        if (motors_[joint.joint_index]) {
          miraculous_motor_disable(motors_[joint.joint_index]);
        }
      }
    }
  }
  // Keep the read thread out of the SDK until every motor is disabled. Once
  // released it resumes the inactive-mode SYNC/TPDO feedback path.
  exclusive_sdk_io_.store(!release_exclusive_io, std::memory_order_release);
}

bool MiraculousArm::disable_voltage_locked(int timeout_ms)
{
  bool ok = true;
  for (const auto & joint : config_.joints) {
    auto * motor = motors_[joint.joint_index];
    if (motor && miraculous_motor_disable_voltage(motor) < 0) {
      std::fprintf(stderr,
        "[miraculous_arm] disable_voltage: command failed joint %s node %u\n",
        joint.name.c_str(), joint.node_id);
      ok = false;
    }
  }

  for (const auto & joint : config_.joints) {
    auto * motor = motors_[joint.joint_index];
    if (!motor) {
      ok = false;
      continue;
    }
    const int ret = miraculous_motor_wait_state(
      motor, CIA_STATE_SWITCH_ON_DISABLED, timeout_ms);
    if (ret < 0) {
      std::fprintf(stderr,
        "[miraculous_arm] disable_voltage: state verification failed joint %s "
        "node %u: %s (%d)\n",
        joint.name.c_str(), joint.node_id, mrc_strerror(ret), ret);
      ok = false;
      continue;
    }
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    cached_states_[joint.joint_index] = CIA_STATE_SWITCH_ON_DISABLED;
  }
  return ok;
}

bool MiraculousArm::enter_passive_ready_locked(int timeout_ms)
{
  bool ok = true;
  for (const auto & joint : config_.joints) {
    auto * motor = motors_[joint.joint_index];
    if (!motor || miraculous_motor_shutdown(motor) < 0) {
      std::fprintf(stderr,
        "[miraculous_arm] passive shutdown: command failed joint %s node %u\n",
        joint.name.c_str(), joint.node_id);
      ok = false;
    }
  }

  for (const auto & joint : config_.joints) {
    auto * motor = motors_[joint.joint_index];
    if (!motor) {
      ok = false;
      continue;
    }
    const int ret = miraculous_motor_wait_state(
      motor, CIA_STATE_READY_TO_SWITCH_ON, timeout_ms);
    if (ret < 0) {
      std::fprintf(stderr,
        "[miraculous_arm] passive shutdown: state verification failed joint %s "
        "node %u: %s (%d)\n",
        joint.name.c_str(), joint.node_id, mrc_strerror(ret), ret);
      ok = false;
      continue;
    }
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    cached_states_[joint.joint_index] = CIA_STATE_READY_TO_SWITCH_ON;
  }
  return ok;
}

bool MiraculousArm::verify_passive_ready_locked()
{
  bool ready = true;
  for (const auto & joint : config_.joints) {
    auto * motor = motors_[joint.joint_index];
    Cia402State_t state = CIA_STATE_NOT_READY_TO_SWITCH_ON;
    const int ret = motor ? miraculous_motor_get_state(motor, &state) :
      MRC_ERROR_INVALID_PARAM;
    if (ret < 0 || state != CIA_STATE_READY_TO_SWITCH_ON) {
      std::fprintf(stderr,
        "[miraculous_arm] passive state check failed joint %s node %u: "
        "ret=%d state=%d expected=%d\n",
        joint.name.c_str(), joint.node_id, ret, static_cast<int>(state),
        static_cast<int>(CIA_STATE_READY_TO_SWITCH_ON));
      ready = false;
      continue;
    }
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    cached_states_[joint.joint_index] = state;
  }
  return ready;
}

bool MiraculousArm::disable_voltage()
{
  if (!initialized_) {
    return false;
  }
  csp_active_ = false;
  bool ok = false;
  {
    std::lock_guard<std::mutex> sdk_lock(sdk_mutex_);
    ok = disable_voltage_locked(1000);
  }
  exclusive_sdk_io_.store(false, std::memory_order_release);
  return ok;
}

void MiraculousArm::quick_stop()
{
  if (!initialized_) {
    return;
  }
  csp_active_ = false;
  {
    std::lock_guard<std::mutex> sdk_lock(sdk_mutex_);
    for (const auto & joint : config_.joints) {
      if (motors_[joint.joint_index]) {
        miraculous_motor_quick_stop(motors_[joint.joint_index]);
      }
    }
  }
  exclusive_sdk_io_.store(false, std::memory_order_release);
}

bool MiraculousArm::fault_reset()
{
  if (!initialized_) {
    return false;
  }
  bool ok = true;
  {
    std::lock_guard<std::mutex> sdk_lock(sdk_mutex_);
    for (const auto & joint : config_.joints) {
      if (miraculous_motor_fault_reset(motors_[joint.joint_index]) < 0) {
        ok = false;
      }
    }
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    fault_detected_ = false;
  }
  return ok;
}

// ============================ reading ======================================

bool MiraculousArm::get_positions_rad(std::array<double, kArmJoints> & positions) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  positions = cached_pos_rad_;
  return cache_valid_;
}

bool MiraculousArm::get_velocities_rad(std::array<double, kArmJoints> & velocities) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  velocities = cached_vel_rad_;
  return cache_valid_;
}

bool MiraculousArm::get_states(std::array<Cia402State_t, kArmJoints> & states) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  states = cached_states_;
  return cache_valid_;
}

bool MiraculousArm::has_fault() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return fault_detected_;
}

bool MiraculousArm::is_passive_ready()
{
  if (!initialized_ || !passive_) {
    return false;
  }
  std::lock_guard<std::mutex> sdk_lock(sdk_mutex_);
  return verify_passive_ready_locked();
}

bool MiraculousArm::read_passive_feedback(FeedbackSample & sample, int timeout_ms)
{
  if (!initialized_ || !passive_ || timeout_ms <= 0) {
    return false;
  }

  std::lock_guard<std::mutex> sdk_lock(sdk_mutex_);

  MiraMotor * sync_motor = nullptr;
  for (auto * motor : motors_) {
    if (motor) {
      sync_motor = motor;
      break;
    }
  }
  if (!sync_motor) {
    return false;
  }

  // Drain frames queued before this acquisition, then take the generation
  // baseline. A successful result therefore requires a TPDO2 received after
  // the SYNC below for every configured joint.
  const int drain_ret = miraculous_motor_poll(sync_motor, 0);
  if (drain_ret < 0) {
    return false;
  }
  std::array<uint64_t, kArmJoints> baseline{};
  for (const auto & joint : config_.joints) {
    baseline[joint.joint_index] =
      tpdo2_generation_[joint.joint_index].load(std::memory_order_acquire);
  }

  FeedbackSample fresh;
  fresh.sync_time = std::chrono::steady_clock::now();
  if (miraculous_motor_sync_send(sync_motor) < 0) {
    std::fprintf(stderr, "[miraculous_arm] passive feedback: manual SYNC failed\n");
    return false;
  }
  const auto deadline = fresh.sync_time + std::chrono::milliseconds(timeout_ms);

  const auto all_fresh = [&]() {
      for (const auto & joint : config_.joints) {
        if (tpdo2_generation_[joint.joint_index].load(std::memory_order_acquire) <=
          baseline[joint.joint_index])
        {
          return false;
        }
      }
      return true;
    };

  while (!all_fresh()) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return false;
    }
    const auto remaining_us =
      std::chrono::duration_cast<std::chrono::microseconds>(deadline - now).count();
    const int remaining_ms =
      std::max(1, static_cast<int>((remaining_us + 999) / 1000));
    const int poll_ret = miraculous_motor_poll(sync_motor, remaining_ms);
    if (poll_ret < 0) {
      std::fprintf(stderr,
        "[miraculous_arm] passive feedback: poll failed: %s (%d)\n",
        mrc_strerror(poll_ret), poll_ret);
      return false;
    }
  }

  for (const auto & joint : config_.joints) {
    const size_t i = joint.joint_index;
    float position = 0.0f;
    float velocity = 0.0f;
    const int position_ret =
      miraculous_motor_get_position_ex(motors_[i], &position, POS_UNIT_RADIAN);
    const int velocity_ret = miraculous_motor_get_velocity_ex(
      motors_[i], &velocity, VEL_SIDE_LOAD, VEL_UNIT_RAD_S);
    if (position_ret < 0 || velocity_ret < 0) {
      std::fprintf(stderr,
        "[miraculous_arm] passive feedback: cache read failed joint %s node %u "
        "(position=%d velocity=%d)\n",
        joint.name.c_str(), joint.node_id, position_ret, velocity_ret);
      return false;
    }
    fresh.positions_rad[i] = static_cast<double>(position);
    fresh.velocities_rad_s[i] = static_cast<double>(velocity);
  }

  fresh.sequence = ++passive_sample_sequence_;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    for (const auto & joint : config_.joints) {
      const size_t i = joint.joint_index;
      cached_pos_rad_[i] = fresh.positions_rad[i];
      cached_vel_rad_[i] = fresh.velocities_rad_s[i];
    }
    cache_valid_ = true;
  }
  sample = fresh;
  return true;
}

// ============================ CSP writing ==================================

bool MiraculousArm::set_targets_rad(const std::array<double, kArmJoints> & targets)
{
  std::array<double, kArmJoints> clamped = targets;
  check_limits(clamped);
  if (!initialized_ || !csp_active_.load(std::memory_order_acquire)) {
    return false;
  }
  bool ok = true;
  {
    std::lock_guard<std::mutex> sdk_lock(sdk_mutex_);
    if (!csp_active_.load(std::memory_order_acquire)) {
      return false;
    }
    for (const auto & joint : config_.joints) {
      const size_t i = joint.joint_index;
      if (miraculous_motor_csp_set_target_ex(
          motors_[i], static_cast<float>(clamped[i]), POS_UNIT_RADIAN) < 0)
      {
        ok = false;
      }
    }
    // Unified manual SYNC so all axes apply targets on the same edge.
    if (config_.sync_period_us == 0) {
      if (!refresh_feedback_locked(true, 2)) {
        ok = false;
      }
    }
  }
  return ok;
}

void MiraculousArm::send_sync()
{
  if (!initialized_ ||
    (!passive_ && !csp_active_.load(std::memory_order_acquire)))
  {
    return;
  }
  std::lock_guard<std::mutex> sdk_lock(sdk_mutex_);
  for (auto * motor : motors_) {
    if (motor) {
      miraculous_motor_sync_send(motor);
      return;
    }
  }
}

bool MiraculousArm::refresh_feedback_locked(bool send_sync, int poll_timeout_ms)
{
  MiraMotor * sync_motor = nullptr;
  for (auto * motor : motors_) {
    if (motor) {
      sync_motor = motor;
      break;
    }
  }
  if (!sync_motor) {
    return false;
  }
  if (send_sync && miraculous_motor_sync_send(sync_motor) < 0) {
    std::fprintf(stderr, "[miraculous_arm] refresh_feedback: manual SYNC failed\n");
    return false;
  }

  // miraculous_motor_poll(sync_motor, poll_timeout_ms);

  std::array<double, kArmJoints> refreshed_pos{};
  std::array<double, kArmJoints> refreshed_vel{};
  bool ok = true;
  for (const auto & joint : config_.joints) {
    const size_t i = joint.joint_index;
    float p_rad = 0.0f;
    float v_rad_s = 0.0f;
    const int pret = miraculous_motor_get_position_ex(motors_[i], &p_rad, POS_UNIT_RADIAN);
    if (pret < 0) {
      std::fprintf(stderr,
        "[miraculous_arm] refresh_feedback: get_position_ex failed joint %s node %u: %s (%d)\n",
        joint.name.c_str(), joint.node_id, mrc_strerror(pret), pret);
      ok = false;
      continue;
    }
    miraculous_motor_get_velocity_ex(motors_[i], &v_rad_s, VEL_SIDE_LOAD, VEL_UNIT_RAD_S);
    refreshed_pos[i] = static_cast<double>(p_rad);
    refreshed_vel[i] = static_cast<double>(v_rad_s);
  }

  if (ok) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (const auto & joint : config_.joints) {
      const size_t i = joint.joint_index;
      cached_pos_rad_[i] = refreshed_pos[i];
      cached_vel_rad_[i] = refreshed_vel[i];
    }
    cache_valid_ = true;
  }
  return ok;
}

// ============================ safety ======================================

bool MiraculousArm::check_limits(std::array<double, kArmJoints> & targets) const
{
  bool within = true;
  for (const auto & joint : config_.joints) {
    const size_t i = joint.joint_index;
    const double lo = joint.position_min;
    const double hi = joint.position_max;
    if (hi > lo) {  // only clamp when limits are configured
      if (targets[i] < lo) {
        targets[i] = lo;
        within = false;
      } else if (targets[i] > hi) {
        targets[i] = hi;
        within = false;
      }
    }
  }
  return within;
}

void MiraculousArm::set_emcy_callback(EmcyCallback callback)
{
  std::lock_guard<std::mutex> lock(emcy_mutex_);
  emcy_callback_ = std::move(callback);
}

// ============================ background read =============================

void MiraculousArm::start_read_thread()
{
  if (read_thread_running_) {
    return;
  }
  read_thread_running_ = true;
  read_thread_ = std::thread(&MiraculousArm::read_loop, this);
}

void MiraculousArm::stop_read_thread()
{
  read_thread_running_ = false;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
}

void MiraculousArm::read_loop()
{
  using clock = std::chrono::steady_clock;
  const auto period_us = static_cast<int64_t>(
    (config_.read_rate_hz > 0.0) ? (1.0e6 / config_.read_rate_hz) : 10000.0);
  const std::chrono::microseconds period(period_us);
  auto next_wake = clock::now();

  std::array<float, kArmJoints> joint_pos_rad{};
  std::array<float, kArmJoints> joint_vel_rad_s{};
  std::array<Cia402State_t, kArmJoints> states{};
  std::array<bool, kArmJoints> position_ok{};
  size_t read_fail_streak = 0;  // consecutive cycles with any position read failure
  // Grace before the first read-failure warning: ~0.5 s at 100 Hz.
  const size_t kReadFailWarnStreak = 50;
  const bool poll_stateword = config_.state_poll_rate_hz > 0.0;
  const size_t state_poll_interval = poll_stateword ?
    static_cast<size_t>(std::max(1.0, config_.read_rate_hz / config_.state_poll_rate_hz)) : 0;
  size_t read_tick = 0;

  while (read_thread_running_) {
    next_wake += period;

    // In active manual CSP, set_targets_rad() performs the complete bus cycle:
    // RPDO writes, one SYNC, poll, and feedback cache update. Avoid both SDK
    // contention and an asynchronous overwrite of that cycle's feedback.
    if (exclusive_sdk_io_.load(std::memory_order_acquire)) {
      std::this_thread::sleep_until(next_wake);
      continue;
    }

    const bool poll_state = poll_stateword && (read_tick++ % state_poll_interval) == 0;
    position_ok.fill(false);
    bool sdk_cycle_completed = false;

    {
      std::lock_guard<std::mutex> sdk_lock(sdk_mutex_);
      // Recheck after acquiring the mutex. enable_csp() publishes exclusive I/O
      // ownership before taking this same mutex, closing the check/lock race.
      if (!exclusive_sdk_io_.load(std::memory_order_acquire)) {
        // The drive's TPDOs are SYNC-triggered: without a SYNC on the bus the
        // SDK position cache never becomes valid (official examples send a SYNC
        // before the first read). While CSP is active the write cycle (manual
        // mode) or the SDK timer (timer mode) already produces SYNC edges and an
        // extra one here would double-latch targets. Before the first safe timer
        // enable, and after a complete rollback, use an explicit feedback SYNC
        // while the drives are non-enabled. enable_csp() holds exclusive SDK I/O
        // across preparation and seeding, so this path cannot interleave there.
        // Then get_position_ex/get_velocity_ex poll and read the latest cache.
        for (auto * motor : motors_) {
          if (motor) {
            if (!csp_active_ &&
              (config_.sync_period_us == 0 || !sync_timer_running_))
            {
              miraculous_motor_sync_send(motor);
              // const double timestamp_s = std::chrono::duration<double>(
              //   std::chrono::system_clock::now().time_since_epoch()).count();
              // std::fprintf(stderr,
              //   "[%.6f] !csp_active_ && config_.sync_period_us == 0:"
              //   "miraculous_motor_sync_send\n",
              //   timestamp_s);
              //miraculous_motor_poll(motor, 1);  // allow SYNC-triggered TPDOs to arrive
            } else {
              //miraculous_motor_poll(motor, 0);
            }
            break;
          }
        }
        for (const auto & joint : config_.joints) {
          const size_t i = joint.joint_index;
          auto * motor = motors_[i];
          if (!motor) {
            continue;
          }
          float p_rad = 0.0f;
          float v_rad_s = 0.0f;
          const int pret = miraculous_motor_get_position_ex(motor, &p_rad, POS_UNIT_RADIAN);
          std::fprintf(stderr,
            "[miraculous_arm] read_loop: get_position_ex joint %s node %u: %s (%d)\n",
            joint.name.c_str(), joint.node_id, mrc_strerror(pret), pret);
          if (pret < 0) {
            // Rate-limited, with a startup grace: the first SYNC-triggered TPDO
            // can miss the 1 ms poll window right after bootstrap, so stay quiet
            // for the first ~0.5 s and only warn on a persistent failure.
            if (read_fail_streak >= kReadFailWarnStreak &&
              ((read_fail_streak - kReadFailWarnStreak) % 500) == 0)  // ~every 5 s at 100 Hz
            {
              std::fprintf(stderr,
                "[miraculous_arm] read_loop: get_position_ex failing joint %s node %u: %s (%d) "
                "(%zu consecutive cycles)\n",
                joint.name.c_str(), joint.node_id, mrc_strerror(pret), pret, read_fail_streak);
            }
            continue;
          }
          miraculous_motor_get_velocity_ex(motor, &v_rad_s, VEL_SIDE_LOAD, VEL_UNIT_RAD_S);
          if (poll_state) {
            Cia402State_t s = CIA_STATE_NOT_READY_TO_SWITCH_ON;
            if (miraculous_motor_get_state(motor, &s) >= 0) {
              states[i] = s;
            }
          }

          joint_pos_rad[i] = p_rad;
          joint_vel_rad_s[i] = v_rad_s;
          position_ok[i] = true;
        }
        sdk_cycle_completed = true;
      }
    }

    if (!sdk_cycle_completed) {
      std::this_thread::sleep_until(next_wake);
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      bool all_configured_positions_ok = true;
      for (const auto & joint : config_.joints) {
        if (!position_ok[joint.joint_index]) {
          all_configured_positions_ok = false;
          break;
        }
      }
      if (all_configured_positions_ok) {
        if (!cache_valid_ || read_fail_streak > 0) {
          std::fprintf(stderr,
            "[miraculous_arm] read_loop: position feedback OK for all configured joints\n");
        }
        read_fail_streak = 0;
        for (const auto & joint : config_.joints) {
          const size_t i = joint.joint_index;
          cached_pos_rad_[i] = static_cast<double>(joint_pos_rad[i]);
          cached_vel_rad_[i] = static_cast<double>(joint_vel_rad_s[i]);
        }
        cache_valid_ = true;
      } else {
        ++read_fail_streak;
      }
      if (poll_state) {
        for (const auto & joint : config_.joints) {
          const size_t i = joint.joint_index;
          cached_states_[i] = states[i];
          if (states[i] == CIA_STATE_FAULT ||
            states[i] == CIA_STATE_FAULT_REACTION_ACTIVE)
          {
            fault_detected_ = true;
          }
        }
      }
    }

    std::this_thread::sleep_until(next_wake);
  }
}

void MiraculousArm::tpdo_trampoline(
  uint8_t node_id, uint8_t pdo_num, const uint8_t * data,
  uint8_t len, void * user_data)
{
  auto * self = static_cast<MiraculousArm *>(user_data);
  if (!self || !data || pdo_num != 2 || len < 8) {
    return;
  }
  for (const auto & joint : self->config_.joints) {
    if (joint.node_id == node_id) {
      self->tpdo2_generation_[joint.joint_index].fetch_add(
        1, std::memory_order_release);
      return;
    }
  }
}

void MiraculousArm::emcy_trampoline(
  uint8_t node_id, uint16_t error_code, uint8_t error_reg,
  const uint8_t * mfg_data, uint8_t mfg_len, void * user_data)
{
  (void)mfg_data;
  (void)mfg_len;
  auto * self = static_cast<MiraculousArm *>(user_data);
  if (!self) {
    return;
  }
  // The SDK dispatcher is bus-wide; only react to EMCYs from our joints.
  bool configured_node = false;
  for (const auto & joint : self->config_.joints) {
    if (joint.node_id == node_id) {
      configured_node = true;
      break;
    }
  }
  if (!configured_node) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(self->state_mutex_);
    self->fault_detected_ = true;
  }
  EmcyCallback cb;
  {
    std::lock_guard<std::mutex> lock(self->emcy_mutex_);
    cb = self->emcy_callback_;
  }
  if (cb) {
    cb(node_id, error_code, error_reg);
  }
}

}  // namespace miraculous_driver
