#include "miraculous_driver/miraculous_arm.hpp"

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
    for (auto *& motor : motors_) {
      if (motor) {
        miraculous_motor_shutdown(motor);
        miraculous_motor_close(motor);
        motor = nullptr;
      }
    }
    can_ctx_ = nullptr;
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
  }
  can_ctx_ = nullptr;
  for (auto * motor : motors_) {
    if (motor) {
      can_ctx_ = miraculous_motor_get_can_ctx(motor);
      break;
    }
  }
  // Register a shared CAN receive callback for EMCY detection.
  if (can_ctx_) {
    miraculous_can_set_recv_callback(can_ctx_, &MiraculousArm::can_recv_trampoline, this);
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
  for (const auto & joint : config_.joints) {
    auto * motor = motors_[joint.joint_index];
    if (miraculous_motor_full_enable(motor) < 0) {
      std::fprintf(stderr, "[miraculous_arm] enable_csp: enable failed joint %s\n",
        joint.name.c_str());
      return false;
    }
    if (miraculous_motor_set_mode(motor, CIA_MODE_CSP) < 0) {
      std::fprintf(stderr, "[miraculous_arm] enable_csp: set CSP mode failed joint %s\n",
        joint.name.c_str());
      return false;
    }
    const bool manual_sync = (config_.sync_period_us == 0);
    if (miraculous_motor_csp_init(motor, config_.sync_period_us, manual_sync) < 0) {
      std::fprintf(stderr, "[miraculous_arm] enable_csp: csp_init failed joint %s\n",
        joint.name.c_str());
      return false;
    }
  }
  // Seed the CSP controller with the current position to avoid a jump on enable.
  std::array<double, kArmJoints> cur{};
  if (get_positions_rad(cur)) {
    set_targets_rad(cur);
  }
  passive_ = false;
  return true;
}

bool MiraculousArm::enable()
{
  if (!initialized_) {
    return false;
  }
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
  for (const auto & joint : config_.joints) {
    if (motors_[joint.joint_index]) {
      if (config_.sync_period_us != 0) {
        miraculous_motor_sync_stop(motors_[joint.joint_index]);
      }
      miraculous_motor_disable(motors_[joint.joint_index]);
    }
  }
}

void MiraculousArm::quick_stop()
{
  if (!initialized_) {
    return;
  }
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
  for (const auto & joint : config_.joints) {
    if (miraculous_motor_fault_reset(motors_[joint.joint_index]) < 0) {
      ok = false;
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
    send_sync();
  }
  return ok;
}

void MiraculousArm::send_sync()
{
  for (auto * motor : motors_) {
    if (motor) {
      miraculous_motor_sync_send(motor);
      return;
    }
  }
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

  while (read_thread_running_) {
    next_wake += period;

    for (const auto & joint : config_.joints) {
      const size_t i = joint.joint_index;
      auto * motor = motors_[i];
      if (!motor) {
        continue;
      }
      float p_rad = 0.0f;
      float v_rad_s = 0.0f;
      Cia402State_t s = CIA_STATE_NOT_READY_TO_SWITCH_ON;
      miraculous_motor_get_position_ex(motor, &p_rad, POS_UNIT_RADIAN);
      miraculous_motor_get_velocity_ex(motor, &v_rad_s, VEL_SIDE_LOAD, VEL_UNIT_RAD_S);
      miraculous_motor_get_state(motor, &s);
      joint_pos_rad[i] = p_rad;
      joint_vel_rad_s[i] = v_rad_s;
      states[i] = s;
    }
    // Poll the shared CAN context to dispatch receive/EMCY callbacks.
    for (auto * motor : motors_) {
      if (motor) {
        miraculous_motor_poll(motor, 0);
        break;
      }
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      for (size_t i = 0; i < kArmJoints; ++i) {
        cached_pos_rad_[i] = static_cast<double>(joint_pos_rad[i]);
        cached_vel_rad_[i] = static_cast<double>(joint_vel_rad_s[i]);
        cached_states_[i] = states[i];
        if (states[i] == CIA_STATE_FAULT ||
          states[i] == CIA_STATE_FAULT_REACTION_ACTIVE)
        {
          fault_detected_ = true;
        }
      }
      cache_valid_ = true;
    }

    std::this_thread::sleep_until(next_wake);
  }
}

void MiraculousArm::can_recv_trampoline(
  uint32_t can_id, const uint8_t * data, uint8_t len, void * user_data)
{
  auto * self = static_cast<MiraculousArm *>(user_data);
  if (!self) {
    return;
  }
  // EMCY COB-ID = 0x080 + node_id (0x081..0x0FF). SYNC itself is 0x080.
  if (can_id < 0x081 || can_id > 0x0FF || len < 3) {
    return;
  }
  const uint8_t node_id = static_cast<uint8_t>(can_id - 0x080);
  const uint16_t error_code =
    static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
  const uint8_t error_reg = data[2];
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
