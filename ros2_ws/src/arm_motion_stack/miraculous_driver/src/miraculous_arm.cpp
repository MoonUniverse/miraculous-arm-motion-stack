#include "miraculous_driver/miraculous_arm.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>

namespace miraculous_driver
{
namespace
{
constexpr uint32_t kSyncCanId = 0x080;  // CANopen SYNC COB-ID
}

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
  if (config_.joints.size() != kArmJoints) {
    std::fprintf(stderr,
      "[miraculous_arm] init: expected %zu joints, got %zu\n",
      kArmJoints, config_.joints.size());
    return false;
  }
  for (size_t i = 0; i < kArmJoints; ++i) {
    if (config_.joints[i].pulses_per_radian <= 0.0) {
      std::fprintf(stderr,
        "[miraculous_arm] init: joint %zu (%s) has invalid pulses_per_radian=%g\n",
        i, config_.joints[i].name.c_str(),
        config_.joints[i].pulses_per_radian);
      return false;
    }
  }
  if (!open_motors()) {
    return false;
  }
  // Bring NMT to Operational so encoder objects (0x6064/0x606C) are readable
  // even before the power stage is enabled. Best-effort: a failed bootstrap on
  // one joint does not abort the others.
  for (size_t i = 0; i < kArmJoints; ++i) {
    if (miraculous_motor_bootstrap(motors_[i], 3000) < 0) {
      std::fprintf(stderr,
        "[miraculous_arm] init: bootstrap failed for joint %zu (node %u)\n",
        i, config_.joints[i].node_id);
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
    for (size_t i = 0; i < kArmJoints; ++i) {
      if (motors_[i]) {
        miraculous_motor_shutdown(motors_[i]);
        miraculous_motor_close(motors_[i]);
        motors_[i] = nullptr;
      }
    }
    can_ctx_ = nullptr;
    initialized_ = false;
    passive_ = false;
  }
}

bool MiraculousArm::open_motors()
{
  for (size_t i = 0; i < kArmJoints; ++i) {
    motors_[i] = miraculous_motor_open(
      config_.can_interface.c_str(), config_.baudrate,
      config_.joints[i].node_id);
    if (!motors_[i]) {
      std::fprintf(stderr,
        "[miraculous_arm] open: failed to open joint %zu (node %u) on %s\n",
        i, config_.joints[i].node_id, config_.can_interface.c_str());
      for (size_t j = 0; j < i; ++j) {
        miraculous_motor_close(motors_[j]);
        motors_[j] = nullptr;
      }
      return false;
    }
  }
  can_ctx_ = miraculous_motor_get_can_ctx(motors_[0]);
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
  for (size_t i = 0; i < kArmJoints; ++i) {
    if (miraculous_motor_full_enable(motors_[i]) < 0) {
      std::fprintf(stderr, "[miraculous_arm] enable_csp: enable failed joint %zu\n", i);
      return false;
    }
    if (miraculous_motor_set_mode(motors_[i], CIA_MODE_CSP) < 0) {
      std::fprintf(stderr, "[miraculous_arm] enable_csp: set CSP mode failed joint %zu\n", i);
      return false;
    }
    // manual=true: SDK won't auto-send SYNC; wrapper sends one unified SYNC per cycle.
    if (miraculous_motor_csp_init(motors_[i], config_.sync_period_us, true) < 0) {
      std::fprintf(stderr, "[miraculous_arm] enable_csp: csp_init failed joint %zu\n", i);
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
  for (size_t i = 0; i < kArmJoints; ++i) {
    if (miraculous_motor_full_enable(motors_[i]) < 0) {
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
  for (size_t i = 0; i < kArmJoints; ++i) {
    if (motors_[i]) {
      miraculous_motor_disable(motors_[i]);
    }
  }
}

void MiraculousArm::quick_stop()
{
  if (!initialized_) {
    return;
  }
  for (size_t i = 0; i < kArmJoints; ++i) {
    if (motors_[i]) {
      miraculous_motor_quick_stop(motors_[i]);
    }
  }
}

bool MiraculousArm::fault_reset()
{
  if (!initialized_) {
    return false;
  }
  bool ok = true;
  for (size_t i = 0; i < kArmJoints; ++i) {
    if (miraculous_motor_fault_reset(motors_[i]) < 0) {
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

bool MiraculousArm::get_positions_pulse(std::array<int32_t, kArmJoints> & positions) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  positions = cached_pos_pulse_;
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
  for (size_t i = 0; i < kArmJoints; ++i) {
    if (miraculous_motor_csp_set_target_ex(motors_[i],
                                            static_cast<float>(clamped[i]),
                                            POS_UNIT_RADIAN) < 0) {
      ok = false;
    }
  }
  // Unified SYNC so all axes apply their targets on the same edge.
  // NOTE: if csp_set_target is found to auto-send SYNC internally, set
  // sync_period_us != 0 (SDK timer) and drop this manual frame. See plan risk #1.
  if (config_.sync_period_us == 0) {
    send_sync();
  }
  return ok;
}

bool MiraculousArm::set_targets_pulse(const std::array<int32_t, kArmJoints> & targets)
{
  // Deprecated: use set_targets_rad() with SDK _ex API for better precision.
  // This function converts pulses back to radians and calls set_targets_rad().
  std::array<double, kArmJoints> targets_rad{};
  for (size_t i = 0; i < kArmJoints; ++i) {
    targets_rad[i] = pulse_to_rad(targets[i], i);
  }
  return set_targets_rad(targets_rad);
}

void MiraculousArm::send_sync()
{
  if (can_ctx_) {
    miraculous_can_send(can_ctx_, kSyncCanId, nullptr, 0);
  }
}

// ============================ safety ======================================

bool MiraculousArm::check_limits(std::array<double, kArmJoints> & targets) const
{
  bool within = true;
  for (size_t i = 0; i < kArmJoints; ++i) {
    const double lo = config_.joints[i].position_min;
    const double hi = config_.joints[i].position_max;
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

  std::array<float, kArmJoints> pos_rad{};
  std::array<int32_t, kArmJoints> vel_pulse{};
  std::array<Cia402State_t, kArmJoints> states{};

  while (read_thread_running_) {
    next_wake += period;

    for (size_t i = 0; i < kArmJoints; ++i) {
      if (!motors_[i]) {
        continue;
      }
      float p_rad = 0.0f;
      int32_t v = 0;
      Cia402State_t s = CIA_STATE_NOT_READY_TO_SWITCH_ON;
      miraculous_motor_get_position_ex(motors_[i], &p_rad, POS_UNIT_RADIAN);
      miraculous_motor_get_velocity(motors_[i], &v);
      miraculous_motor_get_state(motors_[i], &s);
      pos_rad[i] = p_rad;
      vel_pulse[i] = v;
      states[i] = s;
    }
    // Poll the shared CAN context to dispatch receive/EMCY callbacks.
    if (motors_[0]) {
      miraculous_motor_poll(motors_[0], 0);
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      for (size_t i = 0; i < kArmJoints; ++i) {
        cached_pos_rad_[i] = static_cast<double>(pos_rad[i]);
        // Also cache pulse values for compatibility (using pulses_per_radian)
        cached_pos_pulse_[i] = rad_to_pulse(cached_pos_rad_[i], i);
        cached_vel_rad_[i] =
          (config_.joints[i].pulses_per_radian > 0.0)
            ? static_cast<double>(vel_pulse[i]) / config_.joints[i].pulses_per_radian
            : 0.0;
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

// ============================ unit conversion =============================
// NOTE: These functions are kept for backward compatibility (e.g., set_targets_pulse).
// New code should prefer SDK _ex API (csp_set_target_ex, get_position_ex) which
// handles unit conversion internally with encoder bit-width precision.

int32_t MiraculousArm::rad_to_pulse(double rad, size_t i) const
{
  const double ppr = config_.joints[i].pulses_per_radian;
  return static_cast<int32_t>(std::llround(rad * ppr));
}

double MiraculousArm::pulse_to_rad(int32_t pulse, size_t i) const
{
  const double ppr = config_.joints[i].pulses_per_radian;
  return (ppr > 0.0) ? static_cast<double>(pulse) / ppr : 0.0;
}

}  // namespace miraculous_driver
