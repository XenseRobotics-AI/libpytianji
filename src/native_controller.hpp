#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <cstdint>
#include <mutex>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>

namespace libpytianji {

class PinkIKModel;

constexpr std::size_t kArmCount = 2;
constexpr std::size_t kArmDof = 7;
constexpr std::size_t kJointCount = kArmCount * kArmDof;
constexpr std::size_t kPoseElements = 16;

using Pose = std::array<double, kPoseElements>;
using JointVector = std::array<double, kJointCount>;
using ArmVector = std::array<double, kArmDof>;

enum class ControllerMode : std::uint8_t {
  Stopped = 0,
  Starting = 1,
  Streaming = 2,
  Home = 3,
  Hold = 4,
  Fault = 5,
};

struct TianjiState {
  JointVector q;
  JointVector dq;
  JointVector last_command_q;
  Pose tcp_left;
  Pose tcp_right;
  std::uint64_t timestamp_ns;
  int fault_code;
  bool valid;
};

struct TianjiStatus {
  ControllerMode mode{ControllerMode::Stopped};
  std::uint64_t target_sequence{0};
  std::uint64_t consumed_sequence{0};
  std::uint64_t trajectory_sequence{0};
  std::uint64_t trajectory_done_sequence{0};
  std::uint64_t ik_success_count{0};
  std::uint64_t ik_failure_count{0};
  // Number of transitions into the native Cartesian-input watchdog hold.
  // This is distinct from ``ik_failure_count`` so a caller can tell an
  // absent/stale producer apart from a rejected IK sample.
  std::uint64_t command_timeout_count{0};
  // Count scheduler configuration failures and missed native periods.  The
  // boolean below remains a sticky summary; the counter lets the Python
  // boundary report a new warning without polling the real-time thread.
  std::uint64_t scheduler_warning_count{0};
  // Age of the latest accepted Cartesian target, measured with the same
  // monotonic clock as the planner.  Zero means that no target has arrived.
  std::uint64_t target_age_ns{0};
  int last_ik_status{0};
  int fault_code{0};
  std::uint64_t timestamp_ns{0};
  int planner_hz{300};
  int io_hz{1000};
  int command_period_ms{5};
  bool running{false};
  bool trajectory_active{false};
  bool scheduler_degraded{false};
  bool command_timed_out{false};
  // Startup is not considered complete until a valid feedback frame has been
  // received and the planner has published its first command seeded from
  // that frame.  Exposing both gates makes a failed/partial start observable
  // to the Python lifecycle and, importantly, prevents a zero-filled mailbox
  // from ever being mistaken for a safe robot target.
  bool feedback_ready{false};
  bool command_ready{false};
  // A bounded diagnostic copied out of the worker-owned fault buffer.  This
  // lets the Python log distinguish a failed SDK transaction from a normal
  // stop without allocating or formatting text in either real-time loop.
  std::array<char, 256> fault_message{};
};

// Fixed-size sequence-lock slot.  Writers publish a complete value by
// changing the sequence from even -> odd -> even.  Readers retry if a write
// overlaps their copy.  Values are deliberately trivially copyable and never
// allocate on a real-time thread.
template <typename T> struct SeqSlot {
  static_assert(std::is_trivially_copyable_v<T>,
                "native mailbox values must be trivially copyable");
  static constexpr std::size_t kWordCount =
      (sizeof(T) + sizeof(std::uint64_t) - 1) / sizeof(std::uint64_t);

  std::atomic<std::uint64_t> sequence;
  std::array<std::atomic<std::uint64_t>, kWordCount> words;
  std::atomic_flag writer = ATOMIC_FLAG_INIT;

  SeqSlot() noexcept {
    sequence.store(0, std::memory_order_relaxed);
    for (auto &word : words) {
      word.store(0, std::memory_order_relaxed);
    }
  }

  void write(const T &next) noexcept {
    while (writer.test_and_set(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
    std::array<std::uint64_t, kWordCount> encoded{};
    std::memcpy(encoded.data(), &next, sizeof(T));
    const auto current = sequence.load(std::memory_order_relaxed);
    sequence.store(current + 1, std::memory_order_release);
    for (std::size_t index = 0; index < kWordCount; ++index) {
      words[index].store(encoded[index], std::memory_order_relaxed);
    }
    sequence.store(current + 2, std::memory_order_release);
    writer.clear(std::memory_order_release);
  }

  bool read(T &out) const noexcept {
    for (int attempt = 0; attempt < 8; ++attempt) {
      const auto before = sequence.load(std::memory_order_acquire);
      if (before & 1U) {
        continue;
      }
      std::array<std::uint64_t, kWordCount> encoded{};
      for (std::size_t index = 0; index < kWordCount; ++index) {
        encoded[index] = words[index].load(std::memory_order_relaxed);
      }
      const auto after = sequence.load(std::memory_order_acquire);
      if (before == after) {
        std::memcpy(&out, encoded.data(), sizeof(T));
        return true;
      }
    }
    return false;
  }
};

class NativeTianjiController {
public:
  NativeTianjiController(
      std::string robot_ip, std::string kinematics_config_path,
      int planner_hz = 300, int io_hz = 1000,
      double command_timeout_s = 0.5,
      double max_tcp_linear_velocity_m_s = 0.5,
      double max_tcp_angular_velocity_rad_s = 1.0, int velocity_ratio = 100,
      int acceleration_ratio = 100,
      ArmVector left_joint_min_rad = DefaultJointLowerLimit(),
      ArmVector left_joint_max_rad = DefaultJointUpperLimit(),
      ArmVector right_joint_min_rad = DefaultJointLowerLimit(),
      ArmVector right_joint_max_rad = DefaultJointUpperLimit(),
      bool use_application_joint_limits = false, int realtime_priority = 0,
      int cpu_affinity = -1, bool simulate = false,
      double max_joint_velocity_deg_s = 180.0,
      double ik_safety_velocity_scale = 1.0,
      bool enable_workspace_check = false,
      std::array<double, 3> workspace_min_m = {-1.2, -1.2, 0.0},
      std::array<double, 3> workspace_max_m = {1.2, 1.2, 1.5},
      bool enable_inter_arm_distance_check = false,
      double min_tcp_distance_m = 0.18,
      ArmVector pd_joint_stiffness = DefaultJointStiffness(),
      ArmVector pd_joint_damping = DefaultJointDamping());
  ~NativeTianjiController();

  NativeTianjiController(const NativeTianjiController &) = delete;
  NativeTianjiController &operator=(const NativeTianjiController &) = delete;

  void start();
  std::uint64_t submitCartesianTargets(const Pose &left_pose,
                                       const Pose &right_pose,
                                       const std::array<bool, 2> &active);
  std::uint64_t requestJointTrajectory(const ArmVector &left_q,
                                       const ArmVector &right_q,
                                       double duration_s);
  bool waitForTrajectory(double timeout_s);
  TianjiState getState() const;
  TianjiStatus getStatus() const;
  void cancel();
  void stop();
  void estop();

  // Exposed for focused native tests and diagnostics.
  static Pose identityPose() noexcept;
  static Pose interpolatePose(const Pose &start, const Pose &end,
                              double alpha) noexcept;
  static double poseTranslationDistance(const Pose &a, const Pose &b) noexcept;
  static double poseAngularDistance(const Pose &a, const Pose &b) noexcept;
  static double minimumJerk(double tau) noexcept;
  static const char *modeName(ControllerMode mode) noexcept;

  static ArmVector DefaultJointLowerLimit() noexcept;
  static ArmVector DefaultJointUpperLimit() noexcept;
  static ArmVector DefaultJointStiffness() noexcept;
  static ArmVector DefaultJointDamping() noexcept;

private:
  struct CartesianTarget {
    Pose pose[2];
    std::array<bool, 2> active;
    std::uint64_t sequence;
    std::uint64_t arrival_ns;
    std::uint64_t interval_ns;
  };

  struct JointTrajectory {
    JointVector target;
    std::uint64_t sequence;
    std::uint64_t arrival_ns;
    std::uint64_t duration_ns;
  };

  struct JointCommand {
    JointVector q;
    std::uint64_t source_sequence;
    std::uint64_t timestamp_ns;
    bool hold;
  };

  void startImpl();
  void stopImpl(bool emergency);
  void ioLoop();
  void plannerLoop();
  void cleanupVendorSession(bool emergency) noexcept;
  void configureRealtimeThread(bool planner_thread);
  void setFault(int code, const char *message) noexcept;
  void setFaultMessage(const char *message) noexcept;
  bool readLatestTarget(CartesianTarget &target) const noexcept;
  bool readLatestTrajectory(JointTrajectory &trajectory) const noexcept;
  bool readLatestCommand(JointCommand &command) const noexcept;
  void publishCommand(const JointCommand &command) noexcept;
  void publishState(const TianjiState &state) noexcept;
  void initializePlannerKinematics();
  void updateSimulatedState(const JointCommand &command, std::uint64_t now_ns);
  void updateHardwareState(std::uint64_t now_ns);
  bool solveArmIK(std::size_t arm, const Pose &target, const ArmVector &reference,
                  ArmVector &solution, int &status);
  bool solveArmIKSimulated(std::size_t arm, const Pose &target,
                           const ArmVector &reference, ArmVector &solution);
  Pose fkArm(std::size_t arm, const ArmVector &q_rad) const;
  bool candidateIsSafe(std::size_t arm, const ArmVector &candidate,
                       const ArmVector &reference, double max_step) const;
  bool jointWithinLimits(std::size_t arm, const ArmVector &q) const noexcept;
  bool j67WithinLimits(std::size_t arm, const ArmVector &q) const noexcept;
  bool finitePose(const Pose &pose) const noexcept;
  bool poseInWorkspace(const Pose &pose) const noexcept;
  bool interArmDistanceSafe(const Pose &left, const Pose &right) const noexcept;
  void clearCartesianTargetAfterHome(std::uint64_t home_target_sequence) noexcept;
  void notifyTrajectoryDone(std::uint64_t sequence) noexcept;

  std::string robot_ip_;
  std::string kinematics_config_path_;
  int planner_hz_{300};
  int io_hz_{1000};
  std::uint64_t planner_period_ns_{5'000'000};
  std::uint64_t io_period_ns_{1'000'000};
  std::uint64_t command_period_ns_{5'000'000};
  std::uint64_t command_timeout_ns_{500'000'000};
  double max_tcp_linear_velocity_m_s_{0.5};
  double max_tcp_angular_velocity_rad_s_{1.0};
  double max_joint_velocity_deg_s_{180.0};
  // 1.0 preserves normal per-cycle velocity checks; values above one are a
  // diagnostic relaxation while hard joint/J67/singularity checks remain.
  double ik_safety_velocity_scale_{1.0};
  bool enable_workspace_check_{false};
  std::array<double, 3> workspace_min_m_{-1.2, -1.2, 0.0};
  std::array<double, 3> workspace_max_m_{1.2, 1.2, 1.5};
  bool enable_inter_arm_distance_check_{false};
  double min_tcp_distance_m_{0.18};
  int velocity_ratio_{100};
  int acceleration_ratio_{100};
  ArmVector left_joint_min_rad_{};
  ArmVector left_joint_max_rad_{};
  ArmVector right_joint_min_rad_{};
  ArmVector right_joint_max_rad_{};
  bool use_application_joint_limits_{false};
  int realtime_priority_{0};
  int cpu_affinity_{-1};
  bool simulate_{false};
  ArmVector pd_joint_stiffness_{};
  ArmVector pd_joint_damping_{};

  std::atomic<bool> running_{false};
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> emergency_requested_{false};
  std::atomic<bool> io_ready_{false};
  std::atomic<bool> planner_ready_{false};
  std::atomic<bool> feedback_ready_{false};
  std::atomic<bool> planner_command_ready_{false};
  std::atomic<bool> startup_failed_{false};
  std::atomic<bool> scheduler_degraded_{false};

  SeqSlot<CartesianTarget> cartesian_mailbox_;
  SeqSlot<JointTrajectory> trajectory_mailbox_;
  SeqSlot<JointCommand> command_mailbox_;
  SeqSlot<TianjiState> feedback_mailbox_;
  SeqSlot<TianjiState> state_mailbox_;

  std::atomic<std::uint64_t> next_cartesian_sequence_{0};
  std::atomic<std::uint64_t> next_trajectory_sequence_{0};
  std::atomic<std::uint64_t> last_target_arrival_ns_{0};
  std::atomic<std::uint64_t> last_trajectory_arrival_ns_{0};
  std::atomic<std::uint64_t> last_command_sent_ns_{0};
  std::atomic<std::uint64_t> trajectory_done_sequence_{0};
  std::atomic<std::uint64_t> cancel_cartesian_sequence_{0};
  std::atomic<std::uint64_t> cancel_trajectory_sequence_{0};
  std::atomic<std::uint64_t> ik_success_count_{0};
  std::atomic<std::uint64_t> ik_failure_count_{0};
  std::atomic<std::uint64_t> command_timeout_count_{0};
  std::atomic<std::uint64_t> scheduler_warning_count_{0};
  std::atomic<bool> command_timed_out_{false};
  std::atomic<int> last_ik_status_{0};
  std::atomic<int> fault_code_{0};

  mutable std::mutex status_mutex_;
  std::atomic<ControllerMode> mode_{ControllerMode::Stopped};
  std::atomic<std::uint64_t> consumed_sequence_{0};
  std::atomic<std::uint64_t> active_trajectory_sequence_{0};
  std::atomic<std::uint64_t> home_started_target_sequence_{0};
  std::atomic<bool> trajectory_active_{false};
  // Fault reporting is read by the Python lifecycle thread only after the
  // worker has stopped.  Keep it fixed-size so an SDK failure cannot allocate
  // on a real-time thread.
  std::array<char, 256> fault_message_{};
  std::condition_variable status_cv_;

  std::thread planner_thread_;
  std::thread io_thread_;

  // Planner-owned state.  No other thread accesses these objects directly.
  JointVector planner_q_{};
  Pose planner_pose_[2]{identityPose(), identityPose()};
  Pose segment_start_pose_[2]{identityPose(), identityPose()};
  Pose segment_end_pose_[2]{identityPose(), identityPose()};
  std::uint64_t active_cartesian_sequence_{0};
  std::uint64_t failed_cartesian_sequence_{0};
  std::uint64_t segment_start_ns_{0};
  std::uint64_t segment_duration_ns_{5'000'000};
  std::array<bool, 2> segment_active_{{false, false}};
  // The first active waypoint after an inactive/re-anchor edge is a hand-off
  // sample, not a motion request.  Keep that arm on its measured FK pose for
  // one native segment; the following fresh waypoint is the first one that
  // may move it.  This is enforced in C++ so a stale enable-edge sample from
  // any Python/VR producer cannot move the hardware.
  std::array<bool, 2> segment_activation_hold_{{false, false}};
  bool cartesian_segment_complete_{false};
  std::uint64_t active_trajectory_seq_{0};
  std::uint64_t last_started_trajectory_seq_{0};
  std::uint64_t trajectory_start_ns_{0};
  JointVector trajectory_start_q_{};
  JointVector trajectory_target_q_{};
  bool kine_ready_{false};
  // Native Pink/Pinocchio model.  It is constructed before the worker threads
  // start; the planner only performs fixed-size FK/IK operations afterwards.
  std::unique_ptr<PinkIKModel> pink_ik_;
  bool planner_seeded_from_feedback_{false};

  // Vendor kinematics configuration and limits; only plannerLoop reads the
  // initialized solver contexts.
  std::array<int, 2> kine_type_{};
  std::array<std::array<std::array<double, 4>, 8>, 2> kine_dh_{};
  std::array<std::array<std::array<double, 4>, 7>, 2> kine_pnva_{};
  std::array<std::array<std::array<double, 3>, 4>, 2> kine_bd_{};
  std::array<std::array<double, 7>, 2> vendor_lower_deg_{};
  std::array<std::array<double, 7>, 2> vendor_upper_deg_{};
  std::array<std::array<double, 7>, 2> vendor_velocity_deg_s_{};
  std::array<std::array<double, 7>, 2> vendor_acceleration_deg_s2_{};

  // I/O-thread-owned vendor state.  The vendor SDK itself is a process-wide
  // singleton; keeping every call in ioLoop prevents concurrent transactions.
  // We use OnLinkTo plus an explicit feedback wait instead of the convenience
  // Connect() wrapper, whose fixed five-millisecond first-frame window is too
  // short for startup.  Track the attempted session separately so cleanup
  // always releases it before the next controller instance starts.
  bool vendor_session_created_{false};
  bool vendor_connected_{false};
  std::uint64_t io_tick_{0};
  // Feedback serials are maintained by the SDK I/O worker.  A finite copy of
  // the vendor buffer is not sufficient evidence that the link is alive: the
  // SDK keeps the last packet after a network stall.  The worker therefore
  // records the last advancing serial and trips the hard fault path when it
  // stops changing.
  std::array<int, kArmCount> io_feedback_serial_{{0, 0}};
  std::array<unsigned int, kArmCount> io_feedback_stable_frames_{{0, 0}};
  std::array<std::uint64_t, kArmCount> io_feedback_change_ns_{{0, 0}};
  bool io_has_sent_command_{false};
  JointVector io_feedback_q_{};
  JointVector io_feedback_dq_{};
  JointVector io_last_sent_q_{};
  TianjiState io_state_{};
};

} // namespace libpytianji
