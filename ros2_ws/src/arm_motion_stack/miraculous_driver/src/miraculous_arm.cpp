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
  }
}

MiraculousArm::~MiraculousArm()
{
  shutdown();
}

// ============================ lifecycle ====================================

bool MiraculousArm::init(const ArmConfig & config)
{
  if (initialized_) {
    return true;
  }
  config_ = config;
  if (config_.joints.empty() || config_.joints.size() > kArmJoints) {
    std::fprintf(stderr,
      "[miraculous_arm] init: expected 1..%zu configured joints, got %zu\n",
      kArmJoints, config_.joints.size());
    return false;
  }
  std::array<bool, kArmJoints> seen{};
  for (const auto & joint : config_.joints) {
    if (joint.joint_index >= kArmJoints) {
      std::fprintf(stderr,
        "[miraculous_arm] init: joint %s has invalid joint_index=%zu\n",
        joint.name.c_str(), joint.joint_index);
      return false;
    }
    if (seen[joint.joint_index]) {
      std::fprintf(stderr,
        "[miraculous_arm] init: duplicate joint_index=%zu\n",
        joint.joint_index);
      return false;
    }
    seen[joint.joint_index] = true;
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
  passive_ = false;
  initialized_ = true;
  start_read_thread();
  return true;
}

bool MiraculousArm::init_passive(const ArmConfig & config)
{
  // init() already bootstraps (NMT Operational) without enabling the power
  // stage, so motors are free to drag. Just mark the passive flag.
  if (!init(config)) {
    return false;
  }
  passive_ = true;
  return true;
}

void MiraculousArm::shutdown()
{
  stop_read_thread();
  if (initialized_) {
    csp_active_ = false;
    for (auto * motor : motors_) {
      if (motor) {
        // Unregister the EMCY callback first: motor_close frees the shared
        // CANopen master, and a late EMCY frame must not call back into this
        // object. The SYNC timer is shared per bus, one stop suffices.
        miraculous_motor_set_emcy_callback(motor, nullptr, nullptr);
        if (config_.sync_period_us != 0) {
          miraculous_motor_sync_stop(motor);
        }
        break;
      }
    }
    for (auto *& motor : motors_) {
      if (motor) {
        miraculous_motor_shutdown(motor);
        miraculous_motor_close(motor);
        motor = nullptr;
      }
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
      for (auto *& motor : motors_) {
        if (motor) {
          miraculous_motor_close(motor);
          motor = nullptr;
        }
      }
      return false;
    }
    // Radian/velocity conversion parameters used by the _ex APIs. Defaults
    // match the SDK (19-bit encoder, 100:1 gear).
    miraculous_motor_set_encoder_bw(motors_[joint.joint_index], config_.encoder_bw);
    miraculous_motor_set_reduction_ratio(
      motors_[joint.joint_index], static_cast<float>(config_.reduction_ratio));
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

// ============================ PDS state machine ============================

bool MiraculousArm::enable_csp()
{
  if (!initialized_) {
    return false;
  }
  // Bootstrap (NMT Operational) is already done in init(); here we only enable
  // the power stage and switch to CSP.
  const bool timer_sync = (config_.sync_period_us != 0);
  {
    std::lock_guard<std::mutex> sdk_lock(sdk_mutex_);
    for (const auto & joint : config_.joints) {
      auto * motor = motors_[joint.joint_index];
      if (miraculous_motor_set_mode(motor, CIA_MODE_CSP) < 0) {
        std::fprintf(stderr, "[miraculous_arm] enable_csp: set CSP mode failed joint %s\n",
          joint.name.c_str());
        return false;
      }
      if (miraculous_motor_full_enable(motor) < 0) {
        std::fprintf(stderr, "[miraculous_arm] enable_csp: enable failed joint %s\n",
          joint.name.c_str());
        return false;
      }
      // PDO mappings are factory-preconfigured (EDS); csp_init only selects the
      // SYNC strategy. In timer mode it writes the node's 0x1006 comm cycle
      // period and (re)arms the per-bus shared SYNC timer, so calling it per
      // motor is idempotent — one timer, no SYNC storm. In manual mode the
      // period is ignored and the write cycle provides the SYNC edges.
      if (miraculous_motor_csp_init(motor, config_.sync_period_us, !timer_sync) < 0) {
        std::fprintf(stderr, "[miraculous_arm] enable_csp: csp_init failed joint %s\n",
          joint.name.c_str());
        return false;
      }
    }
    if (!refresh_feedback_locked(!timer_sync, 10)) {
      std::fprintf(stderr,
        "[miraculous_arm] enable_csp: feedback refresh before seed failed; "
        "trying cached position\n");
    }
  }
  // Seed the CSP controller with the current position to avoid a jump on
  // enable. Refusing to enable without a valid seed is deliberate: the first
  // write cycle would otherwise command whatever stale target the drive holds.
  std::array<double, kArmJoints> cur{};
  if (!get_positions_rad(cur) || !set_targets_rad(cur)) {
    std::fprintf(stderr,
      "[miraculous_arm] enable_csp: cannot seed targets from current position; "
      "disabling motors again\n");
    disable();
    return false;
  }
  passive_ = false;
  csp_active_ = true;
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
  csp_active_ = false;
  std::lock_guard<std::mutex> sdk_lock(sdk_mutex_);
  // Only one shared SYNC timer is ever started (see enable_csp), stop it once.
  if (config_.sync_period_us != 0) {
    for (auto * motor : motors_) {
      if (motor) {
        miraculous_motor_sync_stop(motor);
        break;
      }
    }
  }
  for (const auto & joint : config_.joints) {
    if (motors_[joint.joint_index]) {
      miraculous_motor_disable(motors_[joint.joint_index]);
    }
  }
}

void MiraculousArm::quick_stop()
{
  if (!initialized_) {
    return;
  }
  csp_active_ = false;
  std::lock_guard<std::mutex> sdk_lock(sdk_mutex_);
  for (const auto & joint : config_.joints) {
    if (motors_[joint.joint_index]) {
      miraculous_motor_quick_stop(motors_[joint.joint_index]);
    }
  }
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

// ============================ CSP writing ==================================

bool MiraculousArm::set_targets_rad(const std::array<double, kArmJoints> & targets)
{
  std::array<double, kArmJoints> clamped = targets;
  check_limits(clamped);
  if (!initialized_) {
    return false;
  }
  bool ok = true;
  {
    std::lock_guard<std::mutex> sdk_lock(sdk_mutex_);
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

  miraculous_motor_poll(sync_motor, poll_timeout_ms);

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
    const bool poll_state = poll_stateword && (read_tick++ % state_poll_interval) == 0;
    position_ok.fill(false);

    {
      std::lock_guard<std::mutex> sdk_lock(sdk_mutex_);
      // The drive's TPDOs are SYNC-triggered: without a SYNC on the bus the
      // SDK position cache never becomes valid (official examples send a SYNC
      // before the first read). While CSP is active the write cycle (manual
      // mode) or the SDK timer (timer mode) already produces SYNC edges and an
      // extra one here would double-latch targets, so only send when inactive.
      // Then poll so get_position_ex/get_velocity_ex read the latest cache.
      for (auto * motor : motors_) {
        if (motor) {
          if (!csp_active_) {
            miraculous_motor_sync_send(motor);
            miraculous_motor_poll(motor, 1);  // give the SYNC-triggered TPDOs time to arrive
          } else {
            miraculous_motor_poll(motor, 0);
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
