#ifndef MIRACULOUS_DRIVER__MIRACULOUS_ARM_HPP_
#define MIRACULOUS_DRIVER__MIRACULOUS_ARM_HPP_

#include <array>
#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "miraculous_sdk.h"

namespace miraculous_driver
{

/// Number of joints on the ARM 6DOF manipulator (J1..J6).
constexpr size_t kArmJoints = 6;

/// Per-joint configuration: CANopen node id and joint-side software limits.
struct JointConfig
{
  std::string name;            ///< joint name, e.g. "J1"
  size_t joint_index = 0;       ///< fixed ROS joint slot: 0=J1 .. 5=J6
  uint8_t node_id = 0;         ///< CANopen node id (1..127)
  double position_min = 0.0;   ///< joint lower limit [rad]
  double position_max = 0.0;   ///< joint upper limit [rad]
};

/// Arm-wide configuration.
struct ArmConfig
{
  std::string can_interface = "can0";        ///< SocketCAN interface name
  CiaBaudrate_t baudrate = CIA_BAUDRATE_1000; ///< CAN baudrate (0 = keep current)
  std::vector<JointConfig> joints;            ///< configured real joints, 1..kArmJoints entries
  /// CSP SYNC period [us]. 0 = manual SYNC (wrapper sends one unified SYNC frame
  /// per write cycle after setting all targets). Non-zero = one shared SDK SYNC
  /// timer started on the first motor handle (never one timer per motor).
  uint32_t sync_period_us = 0;
  /// Encoder bits for the SDK radian conversion (2^bw counts/rev). SDK default 19.
  uint8_t encoder_bw = 19;
  /// Gear ratio for load-side velocity conversion. SDK default 100.
  double reduction_ratio = 100.0;
  double read_rate_hz = 100.0;                ///< background read thread frequency
  double state_poll_rate_hz = 0.0;            ///< statusword SDO polling; 0 disables it
  /// Register the SDK's dedicated EMCY callback (miraculous_motor_set_emcy_callback).
  /// Never use miraculous_can_set_recv_callback here: the SDK's CANopen master
  /// occupies that slot for its own RX dispatch (TPDO cache / heartbeat / EMCY)
  /// and re-registering it silently kills all position feedback.
  bool enable_emcy_monitor = true;
};

/// EMCY event callback signature.
using EmcyCallback =
  std::function<void(uint8_t node_id, uint16_t error_code, uint8_t error_reg)>;

/**
 * @brief C++ wrapper around miraculous_sdk managing the 6 MiraMotor handles of
 *        the ARM manipulator.
 *
 * Responsibilities:
 *  - open / bootstrap / enable / disable the configured motors
 *  - pass joint-side radians directly through the SDK _ex APIs
 *  - background thread polling actual position/velocity into a mutex cache
 *  - CSP write of configured target positions with a unified SYNC broadcast
 *  - joint limit clamping and EMCY detection
 *
 * Threading: read() callers (ros2_control or teach/playback nodes) only copy the
 * cached values and never block on CAN/SDO traffic.
 */
class MiraculousArm
{
public:
  MiraculousArm();
  ~MiraculousArm();

  MiraculousArm(const MiraculousArm &) = delete;
  MiraculousArm & operator=(const MiraculousArm &) = delete;

  // ---- lifecycle -----------------------------------------------------------

  /// Open 6 motors without enabling. Starts the
  /// background read thread. Returns false on any open failure (opened motors
  /// are closed automatically).
  bool init(const ArmConfig & config);

  /// Passive (teach) mode: open + bootstrap motors so NMT is Operational and
  /// encoder values are readable, but motors are NOT enabled (free to drag).
  bool init_passive(const ArmConfig & config);

  /// Stop the read thread, disable and close all motors. Safe to call once.
  void shutdown();

  bool is_initialized() const { return initialized_; }
  bool is_passive() const { return passive_; }

  // ---- PDS state machine ---------------------------------------------------

  /// set_mode(CSP) + full_enable + csp_init for configured motors (bootstrap is
  /// done in init()). Seeds the CSP targets with the current position; fails if
  /// that position cannot be read, so a successful return guarantees the arm
  /// holds its pose. After this the arm follows set_targets_rad() each write
  /// cycle.
  bool enable_csp();

  /// Enable all motors (without switching to CSP). Rarely needed directly.
  bool enable();

  /// Disable all motors (Operation Enabled -> Switched On).
  void disable();

  /// Quick-stop all motors (emergency deceleration).
  void quick_stop();

  /// Fault reset on all motors, then leave them disabled.
  /// @return true if all motors reset successfully.
  bool fault_reset();

  // ---- reading (thread-safe) ----------------------------------------------

  bool get_positions_rad(std::array<double, kArmJoints> & positions) const;
  bool get_velocities_rad(std::array<double, kArmJoints> & velocities) const;
  bool get_states(std::array<Cia402State_t, kArmJoints> & states) const;
  bool has_fault() const;

  // ---- CSP writing ---------------------------------------------------------

  /// Set joint-side target positions [rad] for configured motors, then send one
  /// unified SYNC frame in manual SYNC mode. Targets are clamped to joint-side
  /// limits.
  /// @return true if all writes succeeded.
  bool set_targets_rad(const std::array<double, kArmJoints> & targets);

  /// Send a single SYNC frame (CAN id 0x80, len 0) on the shared CAN context.
  void send_sync();

  // ---- safety -------------------------------------------------------------

  /// Clamp targets to per-joint limits. Returns true if no clamping was needed.
  bool check_limits(std::array<double, kArmJoints> & targets) const;

  /// Register an EMCY callback. EMCY frames (CAN id 0x080 + node_id) are
  /// delivered through the SDK's per-bus EMCY dispatcher.
  void set_emcy_callback(EmcyCallback callback);

private:
  // internal helpers
  bool open_motors();
  bool refresh_feedback_locked(bool send_sync, int poll_timeout_ms);
  void start_read_thread();
  void stop_read_thread();
  void read_loop();
  static void emcy_trampoline(
    uint8_t node_id, uint16_t error_code, uint8_t error_reg,
    const uint8_t * mfg_data, uint8_t mfg_len, void * user_data);

  ArmConfig config_;
  std::array<MiraMotor *, kArmJoints> motors_{};

  bool initialized_{false};
  bool passive_{false};
  /// True while CSP is enabled: the SYNC source is then the write cycle
  /// (manual mode) or the SDK timer (timer mode), so the read thread must not
  /// inject extra SYNC edges. While false, the read thread sends one SYNC per
  /// cycle because the drive's TPDOs are SYNC-triggered and would otherwise
  /// never update the position cache.
  std::atomic<bool> csp_active_{false};
  /// Manual CSP writer ownership. While true, set_targets_rad() owns all SDK
  /// I/O (RPDO, SYNC, poll, feedback cache update) and the read thread sleeps.
  /// Timer CSP leaves this false because its timerfd is serviced by read_loop().
  std::atomic<bool> manual_csp_writer_owns_io_{false};

  // cached state (mutex protected)
  mutable std::mutex state_mutex_;
  mutable std::mutex sdk_mutex_;
  std::array<double, kArmJoints> cached_pos_rad_{};
  std::array<double, kArmJoints> cached_vel_rad_{};
  std::array<Cia402State_t, kArmJoints> cached_states_{};
  bool cache_valid_{false};
  bool fault_detected_{false};

  // background thread
  std::thread read_thread_;
  std::atomic<bool> read_thread_running_{false};

  // EMCY
  EmcyCallback emcy_callback_;
  mutable std::mutex emcy_mutex_;
};

}  // namespace miraculous_driver

#endif  // MIRACULOUS_DRIVER__MIRACULOUS_ARM_HPP_
