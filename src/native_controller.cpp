#include "native_controller.hpp"
#include "pink_ik.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <thread>

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#endif

// The vendor headers are intentionally included only in this translation unit.
// No vendor type crosses the Python boundary.
#include "FxRobot.h"
#include "MarvinSDK.h"

namespace libpytianji {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kPositionEpsilon = 1.0e-9;
constexpr int kIkOk = 1;
constexpr int kIkFailed = -1;
constexpr int kIkInvalidPose = -2;
constexpr int kIkUnsafeCandidate = -3;
// A physical startup is not considered valid until several *advancing*
// feedback frames have arrived from both arms.  The vendor SDK initializes
// its shared buffer to zero and may leave the last packet in place during a
// network stall, so a single finite/non-zero snapshot is not enough.
constexpr std::uint64_t kFeedbackStartupTimeoutNs = 2'000'000'000ULL;
constexpr unsigned int kRequiredFreshFeedbackFrames = 3U;
// This is a gross discontinuity guard in the I/O thread.  Normal rate
// limiting happens in the planner; this guard exists only to make a stale or
// zero-initialized mailbox incapable of commanding a large physical jump.
constexpr double kMaxCommandJumpRad = 5.0 * kDegToRad;
constexpr std::uint64_t kPdEstimatorSettleNs = 1'000'000'000ULL;

// The vendor kinematics library exposes a process-wide binary log switch,
// not a severity-based logger.  Keep its very chatty normal-path diagnostics
// disabled for real-time operation and allow an explicit opt-in when a field
// investigation needs the raw vendor trace.
bool vendorVerboseLoggingEnabled() noexcept {
  // One switch controls both vendor libraries.  The kinematics SDK calls its
  // flag ``FX_LOG_SWITCH`` while Marvin calls its process-side flag
  // ``OnLocalLogOn/Off``; exposing one operator-facing variable avoids a
  // partially quiet process where one 1 kHz path still floods stdout.
  const char *value = std::getenv("LIBPYTIANJI_VENDOR_LOG");
  if (value == nullptr) {
    // Keep the old, more specific name as a compatibility alias for field
    // scripts that were already prepared while the logging fix was developed.
    value = std::getenv("LIBPYTIANJI_VENDOR_KINE_LOG");
  }
  if (value == nullptr) {
    return false;
  }
  return std::strcmp(value, "1") == 0 || std::strcmp(value, "true") == 0 ||
         std::strcmp(value, "TRUE") == 0 || std::strcmp(value, "on") == 0 ||
         std::strcmp(value, "ON") == 0;
}

std::uint64_t nowNs() noexcept {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

template <typename T> bool finiteValue(T value) noexcept {
  return std::isfinite(static_cast<double>(value));
}

double clamp01(double value) noexcept {
  return std::clamp(value, 0.0, 1.0);
}

struct Quaternion {
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

double quaternionNorm(const Quaternion &q) noexcept {
  return std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

Quaternion normalizeQuaternion(Quaternion q) noexcept {
  const double norm = quaternionNorm(q);
  if (!finiteValue(norm) || norm < kPositionEpsilon) {
    return {};
  }
  q.w /= norm;
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  return q;
}

Quaternion rotationToQuaternion(const Pose &pose) noexcept {
  const double m00 = pose[0];
  const double m01 = pose[1];
  const double m02 = pose[2];
  const double m10 = pose[4];
  const double m11 = pose[5];
  const double m12 = pose[6];
  const double m20 = pose[8];
  const double m21 = pose[9];
  const double m22 = pose[10];
  Quaternion q;
  const double trace = m00 + m11 + m22;
  if (trace > 0.0) {
    const double s = std::sqrt(std::max(0.0, trace + 1.0)) * 2.0;
    q.w = 0.25 * s;
    q.x = (m21 - m12) / s;
    q.y = (m02 - m20) / s;
    q.z = (m10 - m01) / s;
  } else if (m00 > m11 && m00 > m22) {
    const double s = std::sqrt(std::max(0.0, 1.0 + m00 - m11 - m22)) * 2.0;
    q.w = (m21 - m12) / s;
    q.x = 0.25 * s;
    q.y = (m01 + m10) / s;
    q.z = (m02 + m20) / s;
  } else if (m11 > m22) {
    const double s = std::sqrt(std::max(0.0, 1.0 + m11 - m00 - m22)) * 2.0;
    q.w = (m02 - m20) / s;
    q.x = (m01 + m10) / s;
    q.y = 0.25 * s;
    q.z = (m12 + m21) / s;
  } else {
    const double s = std::sqrt(std::max(0.0, 1.0 + m22 - m00 - m11)) * 2.0;
    q.w = (m10 - m01) / s;
    q.x = (m02 + m20) / s;
    q.y = (m12 + m21) / s;
    q.z = 0.25 * s;
  }
  return normalizeQuaternion(q);
}

void quaternionToRotation(const Quaternion &raw, Pose &pose) noexcept {
  const Quaternion q = normalizeQuaternion(raw);
  const double xx = q.x * q.x;
  const double yy = q.y * q.y;
  const double zz = q.z * q.z;
  const double xy = q.x * q.y;
  const double xz = q.x * q.z;
  const double yz = q.y * q.z;
  const double wx = q.w * q.x;
  const double wy = q.w * q.y;
  const double wz = q.w * q.z;
  pose[0] = 1.0 - 2.0 * (yy + zz);
  pose[1] = 2.0 * (xy - wz);
  pose[2] = 2.0 * (xz + wy);
  pose[4] = 2.0 * (xy + wz);
  pose[5] = 1.0 - 2.0 * (xx + zz);
  pose[6] = 2.0 * (yz - wx);
  pose[8] = 2.0 * (xz - wy);
  pose[9] = 2.0 * (yz + wx);
  pose[10] = 1.0 - 2.0 * (xx + yy);
}

Quaternion slerp(Quaternion a, Quaternion b, double alpha) noexcept {
  a = normalizeQuaternion(a);
  b = normalizeQuaternion(b);
  double dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
  if (dot < 0.0) {
    dot = -dot;
    b.w = -b.w;
    b.x = -b.x;
    b.y = -b.y;
    b.z = -b.z;
  }
  dot = std::clamp(dot, -1.0, 1.0);
  if (dot > 0.9995) {
    Quaternion out{a.w + alpha * (b.w - a.w), a.x + alpha * (b.x - a.x),
                   a.y + alpha * (b.y - a.y), a.z + alpha * (b.z - a.z)};
    return normalizeQuaternion(out);
  }
  const double theta = std::acos(dot);
  const double sinTheta = std::sin(theta);
  if (std::abs(sinTheta) < kPositionEpsilon) {
    return a;
  }
  const double wa = std::sin((1.0 - alpha) * theta) / sinTheta;
  const double wb = std::sin(alpha * theta) / sinTheta;
  return normalizeQuaternion(
      Quaternion{wa * a.w + wb * b.w, wa * a.x + wb * b.x,
                 wa * a.y + wb * b.y, wa * a.z + wb * b.z});
}

bool parseIp(const std::string &value, std::array<unsigned char, 4> &ip) {
  unsigned int a = 0;
  unsigned int b = 0;
  unsigned int c = 0;
  unsigned int d = 0;
  char tail = '\0';
  if (std::sscanf(value.c_str(), "%u.%u.%u.%u%c", &a, &b, &c, &d, &tail) !=
      4) {
    return false;
  }
  if (a > 255 || b > 255 || c > 255 || d > 255 || a == 0 || a == 127 ||
      (a == 255 && b == 255 && c == 255 && d == 255)) {
    return false;
  }
  ip = {static_cast<unsigned char>(a), static_cast<unsigned char>(b),
        static_cast<unsigned char>(c), static_cast<unsigned char>(d)};
  return true;
}

template <typename Array> bool allFinite(const Array &values) noexcept {
  for (const auto value : values) {
    if (!finiteValue(value)) {
      return false;
    }
  }
  return true;
}

} // namespace

NativeTianjiController::NativeTianjiController(
    std::string robot_ip, std::string kinematics_config_path, int planner_hz,
    int io_hz, double command_timeout_s, double max_tcp_linear_velocity_m_s,
    double max_tcp_angular_velocity_rad_s, int velocity_ratio,
    int acceleration_ratio, ArmVector left_joint_min_rad,
    ArmVector left_joint_max_rad, ArmVector right_joint_min_rad,
    ArmVector right_joint_max_rad, bool use_application_joint_limits,
    int realtime_priority, int cpu_affinity, bool simulate,
    double max_joint_velocity_deg_s, double ik_safety_velocity_scale,
    bool enable_workspace_check,
    std::array<double, 3> workspace_min_m,
    std::array<double, 3> workspace_max_m,
    bool enable_inter_arm_distance_check, double min_tcp_distance_m,
    ArmVector pd_joint_stiffness, ArmVector pd_joint_damping)
    : robot_ip_(std::move(robot_ip)),
      kinematics_config_path_(std::move(kinematics_config_path)),
      planner_hz_(planner_hz), io_hz_(io_hz),
      max_tcp_linear_velocity_m_s_(max_tcp_linear_velocity_m_s),
      max_tcp_angular_velocity_rad_s_(max_tcp_angular_velocity_rad_s),
      max_joint_velocity_deg_s_(max_joint_velocity_deg_s),
      ik_safety_velocity_scale_(ik_safety_velocity_scale),
      enable_workspace_check_(enable_workspace_check),
      workspace_min_m_(workspace_min_m), workspace_max_m_(workspace_max_m),
      enable_inter_arm_distance_check_(enable_inter_arm_distance_check),
      min_tcp_distance_m_(min_tcp_distance_m),
      velocity_ratio_(velocity_ratio), acceleration_ratio_(acceleration_ratio),
      left_joint_min_rad_(left_joint_min_rad),
      left_joint_max_rad_(left_joint_max_rad),
      right_joint_min_rad_(right_joint_min_rad),
      right_joint_max_rad_(right_joint_max_rad),
      use_application_joint_limits_(use_application_joint_limits),
      realtime_priority_(realtime_priority), cpu_affinity_(cpu_affinity),
      simulate_(simulate),
      pd_joint_stiffness_(pd_joint_stiffness),
      pd_joint_damping_(pd_joint_damping) {
  if (planner_hz_ < 1 || planner_hz_ > 1000 || io_hz_ < 1 || io_hz_ > 2000 ||
      io_hz_ < planner_hz_) {
    throw std::invalid_argument("planner_hz/io_hz are outside supported range");
  }
  if (!std::isfinite(command_timeout_s) || command_timeout_s <= 0.0 ||
      !std::isfinite(max_tcp_linear_velocity_m_s_) ||
      max_tcp_linear_velocity_m_s_ <= 0.0 ||
      !std::isfinite(max_tcp_angular_velocity_rad_s_) ||
      max_tcp_angular_velocity_rad_s_ <= 0.0 ||
      !std::isfinite(max_joint_velocity_deg_s_) ||
      max_joint_velocity_deg_s_ <= 0.0 ||
      !std::isfinite(ik_safety_velocity_scale_) ||
      ik_safety_velocity_scale_ <= 0.0) {
    throw std::invalid_argument("native planner limits must be finite and positive");
  }
  if (velocity_ratio_ < 1 || velocity_ratio_ > 100 || acceleration_ratio_ < 1 ||
      acceleration_ratio_ > 100) {
    throw std::invalid_argument("velocity_ratio and acceleration_ratio must be in [1, 100]");
  }
  if (realtime_priority_ < 0 || realtime_priority_ > 99 || cpu_affinity_ < -1) {
    throw std::invalid_argument("invalid realtime priority or CPU affinity");
  }
  if (!allFinite(workspace_min_m_) || !allFinite(workspace_max_m_) ||
      !allFinite(left_joint_min_rad_) || !allFinite(left_joint_max_rad_) ||
      !allFinite(right_joint_min_rad_) || !allFinite(right_joint_max_rad_) ||
      !allFinite(pd_joint_stiffness_) || !allFinite(pd_joint_damping_)) {
    throw std::invalid_argument("native planner arrays must be finite");
  }
  for (std::size_t i = 0; i < 3; ++i) {
    if (workspace_min_m_[i] >= workspace_max_m_[i]) {
      throw std::invalid_argument("workspace_min_m must be below workspace_max_m");
    }
  }
  if (!std::isfinite(min_tcp_distance_m_) || min_tcp_distance_m_ < 0.0) {
    throw std::invalid_argument("min_tcp_distance_m must be finite and non-negative");
  }
  for (std::size_t arm = 0; arm < kArmCount; ++arm) {
    const auto &lower = arm == 0 ? left_joint_min_rad_ : right_joint_min_rad_;
    const auto &upper = arm == 0 ? left_joint_max_rad_ : right_joint_max_rad_;
    if (use_application_joint_limits_) {
      for (std::size_t joint = 0; joint < kArmDof; ++joint) {
        if (lower[joint] >= upper[joint]) {
          throw std::invalid_argument("application joint limits are not ordered");
        }
      }
    }
  }
  planner_period_ns_ = static_cast<std::uint64_t>(
      std::llround(1.0e9 / static_cast<double>(planner_hz_)));
  io_period_ns_ = static_cast<std::uint64_t>(
      std::llround(1.0e9 / static_cast<double>(io_hz_)));
  command_period_ns_ = std::max<std::uint64_t>(
      planner_period_ns_, static_cast<std::uint64_t>(5'000'000));
  if (io_hz_ < 200) {
    command_period_ns_ = io_period_ns_;
  }
  command_timeout_ns_ = static_cast<std::uint64_t>(
      std::llround(command_timeout_s * 1.0e9));
  planner_q_.fill(0.0);
  trajectory_start_q_.fill(0.0);
  trajectory_target_q_.fill(0.0);
  io_feedback_q_.fill(0.0);
  io_feedback_dq_.fill(0.0);
  io_last_sent_q_.fill(0.0);
  io_state_.tcp_left = identityPose();
  io_state_.tcp_right = identityPose();
  for (auto &pose : planner_pose_) {
    pose = identityPose();
  }
  for (auto &pose : segment_start_pose_) {
    pose = identityPose();
  }
  for (auto &pose : segment_end_pose_) {
    pose = identityPose();
  }
  segment_duration_ns_ = planner_period_ns_;
}

NativeTianjiController::~NativeTianjiController() { stop(); }

ArmVector NativeTianjiController::DefaultJointLowerLimit() noexcept {
  return ArmVector{-kPi, -kPi, -kPi, -kPi, -kPi, -kPi, -kPi};
}

ArmVector NativeTianjiController::DefaultJointUpperLimit() noexcept {
  return ArmVector{kPi, kPi, kPi, kPi, kPi, kPi, kPi};
}

ArmVector NativeTianjiController::DefaultJointStiffness() noexcept {
  return ArmVector{14.0, 14.0, 14.0, 10.5, 5.6, 5.6, 5.6};
}

ArmVector NativeTianjiController::DefaultJointDamping() noexcept {
  return ArmVector{0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
}

Pose NativeTianjiController::identityPose() noexcept {
  return Pose{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
              0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
}

Pose NativeTianjiController::interpolatePose(
    const Pose &start, const Pose &end, double alpha) noexcept {
  const double t = clamp01(alpha);
  Pose result = start;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    result[axis * 4 + 3] =
        start[axis * 4 + 3] + t * (end[axis * 4 + 3] - start[axis * 4 + 3]);
  }
  const auto q = slerp(rotationToQuaternion(start), rotationToQuaternion(end), t);
  quaternionToRotation(q, result);
  result[12] = 0.0;
  result[13] = 0.0;
  result[14] = 0.0;
  result[15] = 1.0;
  return result;
}

double NativeTianjiController::poseTranslationDistance(const Pose &a,
                                                        const Pose &b) noexcept {
  const double dx = a[3] - b[3];
  const double dy = a[7] - b[7];
  const double dz = a[11] - b[11];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double NativeTianjiController::poseAngularDistance(const Pose &a,
                                                   const Pose &b) noexcept {
  const auto qa = rotationToQuaternion(a);
  const auto qb = rotationToQuaternion(b);
  const double dot = std::clamp(
      std::abs(qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z), 0.0,
      1.0);
  return 2.0 * std::acos(dot);
}

double NativeTianjiController::minimumJerk(double tau) noexcept {
  const double t = clamp01(tau);
  return t * t * t * (10.0 + t * (-15.0 + 6.0 * t));
}

const char *NativeTianjiController::modeName(ControllerMode mode) noexcept {
  switch (mode) {
  case ControllerMode::Starting:
    return "starting";
  case ControllerMode::Streaming:
    return "streaming";
  case ControllerMode::Home:
    return "home";
  case ControllerMode::Hold:
    return "hold";
  case ControllerMode::Fault:
    return "fault";
  case ControllerMode::Stopped:
  default:
    return "stopped";
  }
}

void NativeTianjiController::start() { startImpl(); }

void NativeTianjiController::startImpl() {
  bool expected = false;
  if (!running_.compare_exchange_strong(expected, true)) {
    throw std::runtime_error("Tianji native controller is already running");
  }
  stop_requested_.store(false, std::memory_order_release);
  emergency_requested_.store(false, std::memory_order_release);
  io_ready_.store(false, std::memory_order_release);
  planner_ready_.store(false, std::memory_order_release);
  feedback_ready_.store(false, std::memory_order_release);
  planner_command_ready_.store(false, std::memory_order_release);
  startup_failed_.store(false, std::memory_order_release);
  scheduler_degraded_.store(false, std::memory_order_release);
  fault_code_.store(0, std::memory_order_release);
  last_ik_status_.store(0, std::memory_order_release);
  next_cartesian_sequence_.store(0, std::memory_order_release);
  next_trajectory_sequence_.store(0, std::memory_order_release);
  last_target_arrival_ns_.store(0, std::memory_order_release);
  last_trajectory_arrival_ns_.store(0, std::memory_order_release);
  last_command_sent_ns_.store(0, std::memory_order_release);
  trajectory_done_sequence_.store(0, std::memory_order_release);
  cancel_cartesian_sequence_.store(0, std::memory_order_release);
  cancel_trajectory_sequence_.store(0, std::memory_order_release);
  ik_success_count_.store(0, std::memory_order_release);
  ik_failure_count_.store(0, std::memory_order_release);
  command_timeout_count_.store(0, std::memory_order_release);
  scheduler_warning_count_.store(0, std::memory_order_release);
  command_timed_out_.store(false, std::memory_order_release);
  consumed_sequence_.store(0, std::memory_order_release);
  active_trajectory_sequence_.store(0, std::memory_order_release);
  home_started_target_sequence_.store(0, std::memory_order_release);
  trajectory_active_.store(false, std::memory_order_release);
  mode_.store(ControllerMode::Starting, std::memory_order_release);
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    fault_message_.fill('\0');
  }
  planner_q_.fill(0.0);
  trajectory_start_q_.fill(0.0);
  trajectory_target_q_.fill(0.0);
  active_cartesian_sequence_ = 0;
  failed_cartesian_sequence_ = 0;
  segment_start_ns_ = 0;
  segment_duration_ns_ = planner_period_ns_;
  segment_active_ = {false, false};
  segment_activation_hold_ = {false, false};
  cartesian_segment_complete_ = false;
  active_trajectory_seq_ = 0;
  last_started_trajectory_seq_ = 0;
  trajectory_start_ns_ = 0;
  planner_seeded_from_feedback_ = false;
  pink_ik_.reset();
  kine_ready_ = false;
  vendor_session_created_ = false;
  vendor_connected_ = false;
  io_feedback_serial_.fill(0);
  io_feedback_stable_frames_.fill(0);
  io_feedback_change_ns_.fill(0);
  io_has_sent_command_ = false;
  io_feedback_q_.fill(0.0);
  io_feedback_dq_.fill(0.0);
  io_last_sent_q_.fill(0.0);
  // This is only a mailbox placeholder.  The I/O loop is explicitly gated by
  // feedback_ready_/planner_command_ready_ and must never transmit this
  // zero-filled value during startup.
  JointCommand empty_command{};
  empty_command.q.fill(0.0);
  empty_command.hold = true;
  CartesianTarget empty_target{};
  empty_target.pose[0] = identityPose();
  empty_target.pose[1] = identityPose();
  trajectory_mailbox_.write(JointTrajectory{});
  cartesian_mailbox_.write(empty_target);
  command_mailbox_.write(empty_command);
  TianjiState empty_state{};
  empty_state.tcp_left = identityPose();
  empty_state.tcp_right = identityPose();
  feedback_mailbox_.write(empty_state);
  state_mailbox_.write(empty_state);
  try {
    initializePlannerKinematics();
  } catch (...) {
    running_.store(false, std::memory_order_release);
    stop_requested_.store(true, std::memory_order_release);
    mode_.store(ControllerMode::Fault, std::memory_order_release);
    throw;
  }
  try {
    planner_thread_ = std::thread([this] {
      try {
        plannerLoop();
      } catch (const std::exception &error) {
        setFault(14, error.what());
      } catch (...) {
        setFault(14, "unhandled exception in Tianji planner thread");
      }
    });
    io_thread_ = std::thread([this] {
      try {
        ioLoop();
      } catch (const std::exception &error) {
        setFault(15, error.what());
        cleanupVendorSession(true);
        io_ready_.store(false, std::memory_order_release);
        running_.store(false, std::memory_order_release);
        status_cv_.notify_all();
      } catch (...) {
        setFault(15, "unhandled exception in Tianji SDK I/O thread");
        cleanupVendorSession(true);
        io_ready_.store(false, std::memory_order_release);
        running_.store(false, std::memory_order_release);
        status_cv_.notify_all();
      }
    });
  } catch (...) {
    // A std::thread construction failure must not leave the other worker
    // joinable while this object unwinds.  No command is considered ready at
    // this point, so stopping the partial startup is position-safe.
    stop_requested_.store(true, std::memory_order_release);
    emergency_requested_.store(true, std::memory_order_release);
    startup_failed_.store(true, std::memory_order_release);
    status_cv_.notify_all();
    if (planner_thread_.joinable()) {
      planner_thread_.join();
    }
    if (io_thread_.joinable()) {
      io_thread_.join();
    }
    running_.store(false, std::memory_order_release);
    mode_.store(ControllerMode::Fault, std::memory_order_release);
    throw;
  }

  // Do not return before the I/O thread has either connected/configured the
  // controller or reported a startup error.  This keeps Python lifecycle
  // semantics deterministic while all vendor calls still belong to ioLoop.
  // The worker's bounded startup budget is 2 s for fresh feedback, up to
  // 3 s per arm for a transient mode reject, plus the one-second PD settling
  // window.  Leave margin for a busy host/UDP stack so a healthy controller
  // is not torn down by the caller exactly as the second arm completes.
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(15);
  std::unique_lock<std::mutex> lock(status_mutex_);
  status_cv_.wait_until(lock, deadline, [this] {
    return (io_ready_.load(std::memory_order_acquire) &&
            planner_command_ready_.load(std::memory_order_acquire) &&
            running_.load(std::memory_order_acquire) &&
            !startup_failed_.load(std::memory_order_acquire) &&
            fault_code_.load(std::memory_order_acquire) == 0) ||
           startup_failed_.load(std::memory_order_acquire) ||
           stop_requested_.load(std::memory_order_acquire);
  });
  const bool ready = io_ready_.load(std::memory_order_acquire) &&
                     feedback_ready_.load(std::memory_order_acquire) &&
                     planner_command_ready_.load(std::memory_order_acquire) &&
                     running_.load(std::memory_order_acquire) &&
                     !startup_failed_.load(std::memory_order_acquire);
  if (!ready) {
    lock.unlock();
    std::string startup_message;
    {
      std::lock_guard<std::mutex> message_lock(status_mutex_);
      startup_message = fault_message_.data();
    }
    if (startup_message.empty()) {
      // A wait_until timeout can happen without an exception in either
      // worker (for example, when the controller never emits a fresh frame).
      // Make that distinction visible to the Python lifecycle instead of
      // collapsing it into the vague "failed to start" message.  The values
      // are sampled before stopImpl() tears the workers down.
      startup_message =
          "Tianji native startup did not become ready within 15 s "
          "(io_ready=" +
          std::to_string(io_ready_.load(std::memory_order_acquire)) +
          ", feedback_ready=" +
          std::to_string(feedback_ready_.load(std::memory_order_acquire)) +
          ", command_ready=" +
          std::to_string(
              planner_command_ready_.load(std::memory_order_acquire)) +
          ", running=" +
          std::to_string(running_.load(std::memory_order_acquire)) +
          ", startup_failed=" +
          std::to_string(startup_failed_.load(std::memory_order_acquire)) +
          ", fault_code=" +
          std::to_string(fault_code_.load(std::memory_order_acquire)) + ")";
    }
    stopImpl(true);
    throw std::runtime_error(startup_message);
  }
}

std::uint64_t NativeTianjiController::submitCartesianTargets(
    const Pose &left_pose, const Pose &right_pose,
    const std::array<bool, 2> &active) {
  if (!running_.load(std::memory_order_acquire)) {
    throw std::runtime_error("Tianji native controller is not running");
  }
  // A running worker is not necessarily a safe worker yet.  During the
  // startup handshake the SDK may have a valid link but no measured frame (or
  // the planner may not have published its first feedback-seeded hold).  Do
  // not let an API caller enqueue a target in that window: accepting it would
  // make the first planner seed depend on the zero-filled mailbox and could
  // reproduce the dangerous move-to-zero observed on the real arm.
  if (!feedback_ready_.load(std::memory_order_acquire) ||
      !planner_command_ready_.load(std::memory_order_acquire)) {
    throw std::runtime_error(
        "Tianji native controller is not ready: waiting for measured feedback");
  }
  if (!finitePose(left_pose) || !finitePose(right_pose)) {
    throw std::invalid_argument("Cartesian targets must be finite homogeneous poses");
  }
  if (enable_workspace_check_ &&
      ((active[0] && !poseInWorkspace(left_pose)) ||
       (active[1] && !poseInWorkspace(right_pose)))) {
    throw std::out_of_range("Cartesian target is outside the configured workspace");
  }
  if (enable_inter_arm_distance_check_ && active[0] && active[1] &&
      !interArmDistanceSafe(left_pose, right_pose)) {
    throw std::out_of_range("Cartesian target violates the inter-arm distance limit");
  }
  CartesianTarget target;
  target.pose[0] = left_pose;
  target.pose[1] = right_pose;
  target.active = active;
  target.arrival_ns = nowNs();
  target.sequence = next_cartesian_sequence_.fetch_add(1) + 1;
  const auto previous = last_target_arrival_ns_.exchange(target.arrival_ns);
  target.interval_ns = previous != 0 && target.arrival_ns > previous
                           ? target.arrival_ns - previous
                           : planner_period_ns_;
  cartesian_mailbox_.write(target);
  return target.sequence;
}

std::uint64_t NativeTianjiController::requestJointTrajectory(
    const ArmVector &left_q, const ArmVector &right_q, double duration_s) {
  if (!running_.load(std::memory_order_acquire)) {
    throw std::runtime_error("Tianji native controller is not running");
  }
  if (!feedback_ready_.load(std::memory_order_acquire) ||
      !planner_command_ready_.load(std::memory_order_acquire)) {
    throw std::runtime_error(
        "Tianji native controller is not ready: waiting for measured feedback");
  }
  if (!allFinite(left_q) || !allFinite(right_q) || !std::isfinite(duration_s) ||
      duration_s <= 0.0) {
    throw std::invalid_argument("joint trajectory requires finite q and positive duration");
  }
  if (!jointWithinLimits(0, left_q) || !jointWithinLimits(1, right_q)) {
    throw std::out_of_range("joint trajectory exceeds configured joint limits");
  }
  JointTrajectory trajectory;
  for (std::size_t i = 0; i < kArmDof; ++i) {
    trajectory.target[i] = left_q[i];
    trajectory.target[kArmDof + i] = right_q[i];
  }
  trajectory.arrival_ns = nowNs();
  trajectory.sequence = next_trajectory_sequence_.fetch_add(1) + 1;
  trajectory.duration_ns = static_cast<std::uint64_t>(
      std::llround(duration_s * 1.0e9));
  trajectory_mailbox_.write(trajectory);
  last_trajectory_arrival_ns_.store(trajectory.arrival_ns,
                                    std::memory_order_release);
  // Targets arriving before or during Home are intentionally discarded when
  // the trajectory completes; the next teleoperation sample re-anchors the
  // Cartesian segment from the actual post-Home feedback.
  home_started_target_sequence_.store(
      next_cartesian_sequence_.load(std::memory_order_acquire),
      std::memory_order_release);
  mode_.store(ControllerMode::Home, std::memory_order_release);
  trajectory_active_.store(true, std::memory_order_release);
  active_trajectory_sequence_.store(trajectory.sequence,
                                    std::memory_order_release);
  status_cv_.notify_all();
  return trajectory.sequence;
}

bool NativeTianjiController::waitForTrajectory(double timeout_s) {
  if (!std::isfinite(timeout_s) || timeout_s < 0.0) {
    throw std::invalid_argument("timeout_s must be finite and non-negative");
  }
  const auto requested = next_trajectory_sequence_.load(std::memory_order_acquire);
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(timeout_s);
  std::unique_lock<std::mutex> lock(status_mutex_);
  return status_cv_.wait_until(lock, deadline, [this, requested] {
    return trajectory_done_sequence_.load(std::memory_order_acquire) >= requested ||
           fault_code_.load(std::memory_order_acquire) != 0 ||
           !running_.load(std::memory_order_acquire);
  }) && trajectory_done_sequence_.load(std::memory_order_acquire) >= requested;
}

void NativeTianjiController::cancel() {
  if (!running_.load(std::memory_order_acquire)) {
    return;
  }
  cancel_cartesian_sequence_.store(
      next_cartesian_sequence_.load(std::memory_order_acquire),
      std::memory_order_release);
  cancel_trajectory_sequence_.store(
      next_trajectory_sequence_.load(std::memory_order_acquire),
      std::memory_order_release);
  last_target_arrival_ns_.store(0, std::memory_order_release);
  command_timed_out_.store(false, std::memory_order_release);
  // The planner is the sole writer of command_mailbox_.  Cancellation is
  // therefore published as an atomic generation marker and materialized as a
  // hold command on the next planner tick; this avoids a second mailbox
  // writer racing the real-time thread.
  trajectory_active_.store(false, std::memory_order_release);
  active_trajectory_sequence_.store(0, std::memory_order_release);
  if (mode_.load(std::memory_order_acquire) != ControllerMode::Fault) {
    mode_.store(ControllerMode::Hold, std::memory_order_release);
  }
  status_cv_.notify_all();
}

void NativeTianjiController::stop() { stopImpl(false); }

void NativeTianjiController::estop() {
  emergency_requested_.store(true, std::memory_order_release);
  stopImpl(true);
}

void NativeTianjiController::stopImpl(bool emergency) {
  if (emergency) {
    emergency_requested_.store(true, std::memory_order_release);
  }
  if (!running_.load(std::memory_order_acquire) && !planner_thread_.joinable() &&
      !io_thread_.joinable()) {
    return;
  }
  stop_requested_.store(true, std::memory_order_release);
  status_cv_.notify_all();
  if (planner_thread_.joinable() &&
      planner_thread_.get_id() != std::this_thread::get_id()) {
    planner_thread_.join();
  }
  if (io_thread_.joinable() && io_thread_.get_id() != std::this_thread::get_id()) {
    io_thread_.join();
  }
  running_.store(false, std::memory_order_release);
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (fault_code_.load(std::memory_order_acquire) == 0) {
      mode_.store(ControllerMode::Stopped, std::memory_order_release);
    }
    trajectory_active_.store(false, std::memory_order_release);
  }
  status_cv_.notify_all();
}

TianjiState NativeTianjiController::getState() const {
  TianjiState state{};
  state_mailbox_.read(state);
  TianjiState feedback{};
  if (feedback_mailbox_.read(feedback)) {
    state.q = feedback.q;
    state.dq = feedback.dq;
    state.fault_code = feedback.fault_code;
    state.timestamp_ns = feedback.timestamp_ns;
    state.valid = feedback.valid;
  }
  JointCommand command{};
  if (command_mailbox_.read(command)) {
    state.last_command_q = command.q;
  }
  return state;
}

TianjiStatus NativeTianjiController::getStatus() const {
  TianjiStatus status{};
  status.target_sequence = next_cartesian_sequence_.load(std::memory_order_acquire);
  status.consumed_sequence =
      consumed_sequence_.load(std::memory_order_acquire);
  status.trajectory_sequence = next_trajectory_sequence_.load(std::memory_order_acquire);
  status.trajectory_done_sequence =
      trajectory_done_sequence_.load(std::memory_order_acquire);
  status.ik_success_count = ik_success_count_.load(std::memory_order_acquire);
  status.ik_failure_count = ik_failure_count_.load(std::memory_order_acquire);
  status.command_timeout_count =
      command_timeout_count_.load(std::memory_order_acquire);
  status.scheduler_warning_count =
      scheduler_warning_count_.load(std::memory_order_acquire);
  status.last_ik_status = last_ik_status_.load(std::memory_order_acquire);
  status.fault_code = fault_code_.load(std::memory_order_acquire);
  status.timestamp_ns = nowNs();
  const auto last_target =
      last_target_arrival_ns_.load(std::memory_order_acquire);
  status.target_age_ns =
      last_target != 0 && status.timestamp_ns > last_target
          ? status.timestamp_ns - last_target
          : 0;
  status.planner_hz = planner_hz_;
  status.io_hz = io_hz_;
  status.command_period_ms = static_cast<int>(command_period_ns_ / 1'000'000);
  status.running = running_.load(std::memory_order_acquire);
  status.scheduler_degraded = scheduler_degraded_.load(std::memory_order_acquire);
  status.mode = mode_.load(std::memory_order_acquire);
  status.trajectory_active =
      trajectory_active_.load(std::memory_order_acquire);
  status.command_timed_out =
      command_timed_out_.load(std::memory_order_acquire);
  status.feedback_ready = feedback_ready_.load(std::memory_order_acquire);
  status.command_ready =
      planner_command_ready_.load(std::memory_order_acquire);
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    std::memcpy(status.fault_message.data(), fault_message_.data(),
                status.fault_message.size());
    status.fault_message.back() = '\0';
  }
  return status;
}

bool NativeTianjiController::readLatestTarget(CartesianTarget &target) const noexcept {
  return cartesian_mailbox_.read(target);
}

bool NativeTianjiController::readLatestTrajectory(
    JointTrajectory &trajectory) const noexcept {
  return trajectory_mailbox_.read(trajectory);
}

bool NativeTianjiController::readLatestCommand(JointCommand &command) const noexcept {
  return command_mailbox_.read(command);
}

void NativeTianjiController::publishCommand(const JointCommand &command) noexcept {
  command_mailbox_.write(command);
}

void NativeTianjiController::publishState(const TianjiState &state) noexcept {
  state_mailbox_.write(state);
}

void NativeTianjiController::setFaultMessage(const char *message) noexcept {
  std::lock_guard<std::mutex> lock(status_mutex_);
  const char *text =
      message == nullptr ? "unknown Tianji native error" : message;
  std::snprintf(fault_message_.data(), fault_message_.size(), "%s", text);
}

void NativeTianjiController::setFault(int code, const char *message) noexcept {
  int expected = 0;
  if (fault_code_.compare_exchange_strong(expected, code,
                                          std::memory_order_acq_rel)) {
    setFaultMessage(message);
  }
  startup_failed_.store(true, std::memory_order_release);
  emergency_requested_.store(true, std::memory_order_release);
  // A fault is terminal for this controller instance.  Stopping the loops
  // here prevents the I/O thread from sending another command after a bad
  // feedback frame or SDK transaction, while stop() remains responsible for
  // joining both workers and releasing the vendor handle.
  stop_requested_.store(true, std::memory_order_release);
  mode_.store(ControllerMode::Fault, std::memory_order_release);
  trajectory_active_.store(false, std::memory_order_release);
  status_cv_.notify_all();
}

bool NativeTianjiController::finitePose(const Pose &pose) const noexcept {
  if (!allFinite(pose)) {
    return false;
  }
  if (std::abs(pose[12]) > 1.0e-6 || std::abs(pose[13]) > 1.0e-6 ||
      std::abs(pose[14]) > 1.0e-6 || std::abs(pose[15] - 1.0) > 1.0e-6) {
    return false;
  }
  const double c0 = pose[0] * pose[0] + pose[4] * pose[4] + pose[8] * pose[8];
  const double c1 = pose[1] * pose[1] + pose[5] * pose[5] + pose[9] * pose[9];
  const double c2 = pose[2] * pose[2] + pose[6] * pose[6] + pose[10] * pose[10];
  const double d01 = pose[0] * pose[1] + pose[4] * pose[5] + pose[8] * pose[9];
  const double d02 = pose[0] * pose[2] + pose[4] * pose[6] + pose[8] * pose[10];
  const double d12 = pose[1] * pose[2] + pose[5] * pose[6] + pose[9] * pose[10];
  return std::abs(c0 - 1.0) < 1.0e-4 && std::abs(c1 - 1.0) < 1.0e-4 &&
         std::abs(c2 - 1.0) < 1.0e-4 && std::abs(d01) < 1.0e-4 &&
         std::abs(d02) < 1.0e-4 && std::abs(d12) < 1.0e-4;
}

bool NativeTianjiController::poseInWorkspace(const Pose &pose) const noexcept {
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double value = pose[axis * 4 + 3];
    if (value < workspace_min_m_[axis] || value > workspace_max_m_[axis]) {
      return false;
    }
  }
  return true;
}

bool NativeTianjiController::interArmDistanceSafe(
    const Pose &left, const Pose &right) const noexcept {
  return poseTranslationDistance(left, right) >= min_tcp_distance_m_;
}

void NativeTianjiController::configureRealtimeThread(bool planner_thread) {
#ifdef __linux__
  if (cpu_affinity_ >= 0) {
    // CPU_SET() does not validate its index and can write past cpu_set_t for
    // a malformed configuration.  Treat an unrepresentable affinity as a
    // scheduler degradation (the worker remains position-safe) instead of
    // allowing startup to corrupt memory or fail nondeterministically.
    if (cpu_affinity_ >= CPU_SETSIZE) {
      scheduler_degraded_.store(true, std::memory_order_release);
      scheduler_warning_count_.fetch_add(1, std::memory_order_acq_rel);
    } else {
      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);
      CPU_SET(cpu_affinity_, &cpuset);
      if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
        scheduler_degraded_.store(true, std::memory_order_release);
        scheduler_warning_count_.fetch_add(1, std::memory_order_acq_rel);
      }
    }
  }
  if (realtime_priority_ > 0) {
    sched_param parameter{};
    // Keep I/O slightly above planner when both share a CPU.
    parameter.sched_priority =
        std::clamp(realtime_priority_ + (planner_thread ? 0 : 1), 1, 99);
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &parameter) != 0) {
      scheduler_degraded_.store(true, std::memory_order_release);
      scheduler_warning_count_.fetch_add(1, std::memory_order_acq_rel);
    }
  }
#else
  (void)planner_thread;
#endif
}

void NativeTianjiController::initializePlannerKinematics() {
  if (simulate_) {
    kine_ready_ = false;
    pink_ik_.reset();
    return;
  }
  // ``libKine.so`` prints every normal FK/IK call by default.  That output is
  // synchronous stdio in the planner thread and can itself consume a
  // meaningful fraction of a 5 ms cycle.  The vendor API has no warn level;
  // use its binary switch for a quiet default and let operators opt into the
  // raw trace with LIBPYTIANJI_VENDOR_LOG=1.
  FX_LOG_SWITCH(vendorVerboseLoggingEnabled() ? 1 : 0);
  const auto slash = kinematics_config_path_.find_last_of("/\\");
  const std::string config_name =
      slash == std::string::npos
          ? kinematics_config_path_
          : kinematics_config_path_.substr(slash + 1);
  std::string normalized_config_name = config_name;
  std::transform(normalized_config_name.begin(), normalized_config_name.end(),
                 normalized_config_name.begin(), [](unsigned char value) {
                   return static_cast<char>(std::tolower(value));
                 });
  // The native Marvin model below is calibrated against the M6-40 URDF and
  // its SDK frame convention.  Silently pairing it with an M6-31/M3/SRS
  // PNVA file would make FK/IK coordinates look plausible while commanding
  // the wrong physical geometry, so fail before either worker starts.
  if (normalized_config_name != "ccs_m6_40.mvkdcfg") {
    throw std::invalid_argument(
        "native Pink/Pinocchio Tianji model is calibrated for "
        "ccs_m6_40.MvKDCfg; received " + config_name);
  }
  int type[2]{};
  double gravity[2][3]{};
  double dh[2][8][4]{};
  double pnva[2][7][4]{};
  double bd[2][4][3]{};
  double mass[2][7]{};
  double mcp[2][7][3]{};
  double inertia[2][7][6]{};
  std::array<char, 4096> path{};
  if (kinematics_config_path_.size() >= path.size()) {
    throw std::invalid_argument("kinematics configuration path is too long");
  }
  std::memcpy(path.data(), kinematics_config_path_.c_str(),
              kinematics_config_path_.size() + 1);
  if (!LOADMvCfg(path.data(), type, gravity, dh, pnva, bd, mass, mcp,
                 inertia)) {
    throw std::runtime_error("LOADMvCfg failed for Tianji kinematics configuration");
  }
  for (std::size_t arm = 0; arm < kArmCount; ++arm) {
    kine_type_[arm] = type[arm];
    std::memcpy(kine_dh_[arm].data(), dh[arm], sizeof(dh[arm]));
    std::memcpy(kine_pnva_[arm].data(), pnva[arm], sizeof(pnva[arm]));
    std::memcpy(kine_bd_[arm].data(), bd[arm], sizeof(bd[arm]));
    if (!FX_Robot_Init_Type(static_cast<int>(arm), type[arm]) ||
        !FX_Robot_Init_Kine(static_cast<int>(arm), dh[arm]) ||
        !FX_Robot_Init_Lmt(static_cast<int>(arm), pnva[arm], bd[arm])) {
      throw std::runtime_error("FX_Robot kinematics initialization failed");
    }
    for (std::size_t joint = 0; joint < kArmDof; ++joint) {
      vendor_upper_deg_[arm][joint] = pnva[arm][joint][0];
      vendor_lower_deg_[arm][joint] = pnva[arm][joint][1];
      vendor_velocity_deg_s_[arm][joint] =
          std::max(1.0, std::abs(pnva[arm][joint][2]));
      vendor_acceleration_deg_s2_[arm][joint] =
          std::max(1.0, std::abs(pnva[arm][joint][3]));
    }
  }
  // Construct the numerical model only after the vendor limits have been
  // loaded, but before either real-time worker starts.  No Python/Pink object
  // is touched from the planner thread; this is libpytianji's own native
  // Pinocchio task solver.
  try {
    pink_ik_ = std::make_unique<PinkIKModel>();
  } catch (const std::exception &error) {
    throw std::runtime_error(std::string("Pink/Pinocchio Tianji model initialization failed: ") +
                             error.what());
  }
  kine_ready_ = true;
}

Pose NativeTianjiController::fkArm(std::size_t arm,
                                   const ArmVector &q_rad) const {
  Pose pose = identityPose();
  if (arm >= kArmCount) {
    return pose;
  }
  if (simulate_ || !pink_ik_) {
    // Simulation and pre-feedback startup use a deterministic small-angle
    // model solely for state/continuity tests.  Hardware always uses the
    // same Pinocchio model as its IK so the planner's reference pose and
    // Cartesian target share exactly one kinematic convention.
    pose[3] = q_rad[0];
    pose[7] = q_rad[1];
    pose[11] = q_rad[2];
    return pose;
  }
  pose = pink_ik_->forwardKinematics(arm, q_rad);
  if (!finitePose(pose)) {
    throw std::runtime_error("Pink/Pinocchio FK returned an invalid pose");
  }
  return pose;
}

bool NativeTianjiController::jointWithinLimits(std::size_t arm,
                                               const ArmVector &q) const noexcept {
  if (arm >= kArmCount || !allFinite(q)) {
    return false;
  }
  if (use_application_joint_limits_) {
    const auto &lower = arm == 0 ? left_joint_min_rad_ : right_joint_min_rad_;
    const auto &upper = arm == 0 ? left_joint_max_rad_ : right_joint_max_rad_;
    for (std::size_t i = 0; i < kArmDof; ++i) {
      if (q[i] < lower[i] || q[i] > upper[i]) {
        return false;
      }
    }
  }
  if (kine_ready_) {
    for (std::size_t i = 0; i < kArmDof; ++i) {
      const double deg = q[i] * kRadToDeg;
      if (deg < vendor_lower_deg_[arm][i] || deg > vendor_upper_deg_[arm][i]) {
        return false;
      }
    }
  }
  return true;
}

bool NativeTianjiController::j67WithinLimits(std::size_t arm,
                                             const ArmVector &q) const noexcept {
  if (!kine_ready_ || arm >= kArmCount) {
    return true;
  }
  const double j6 = q[5] * kRadToDeg;
  const double j7 = q[6] * kRadToDeg;
  if (std::abs(j6) <= 1.0) {
    return true;
  }
  const auto &bd = kine_bd_[arm];
  bool nonzero = false;
  for (const auto &row : bd) {
    for (const auto value : row) {
      nonzero = nonzero || std::abs(value) > 1.0e-12;
    }
  }
  if (!nonzero) {
    return true;
  }
  const auto boundary = [j6](const auto &row) {
    return row[0] * j6 * j6 + row[1] * j6 + row[2];
  };
  const auto &upper = j6 > 0.0 ? bd[0] : bd[1];
  const auto &lower = j6 > 0.0 ? bd[3] : bd[2];
  return j7 >= boundary(lower) && j7 <= boundary(upper);
}

bool NativeTianjiController::candidateIsSafe(
    std::size_t arm, const ArmVector &candidate, const ArmVector &reference,
    double max_step) const {
  if (!jointWithinLimits(arm, candidate) || !j67WithinLimits(arm, candidate) ||
      !allFinite(candidate) || !allFinite(reference)) {
    return false;
  }
  for (std::size_t joint = 0; joint < kArmDof; ++joint) {
    // Do not use circular distance here.  A +/-pi wrap is a real command
    // discontinuity for a bounded vendor joint, not an innocuous equivalent
    // angle.
    const double delta_deg =
        std::abs((candidate[joint] - reference[joint]) * kRadToDeg);
    double velocity_limit = max_joint_velocity_deg_s_ * ik_safety_velocity_scale_;
    if (kine_ready_) {
      velocity_limit = std::min(velocity_limit,
                                vendor_velocity_deg_s_[arm][joint] *
                                    ik_safety_velocity_scale_);
    }
    const double max_step_deg = std::max(
        0.0, velocity_limit / static_cast<double>(std::max(1, planner_hz_)));
    // ``max_step`` is the application-level step; vendor PNVA is an
    // independent hard limit and can only make the admissible step smaller.
    if (delta_deg > std::min(max_step * kRadToDeg, max_step_deg) + 1.0e-5) {
      return false;
    }
  }
  return true;
}

bool NativeTianjiController::solveArmIKSimulated(
    std::size_t arm, const Pose &target, const ArmVector &reference,
    ArmVector &solution) {
  (void)arm;
  solution = reference;
  // The simulation backend preserves the timing/mailbox contract without
  // pretending to be a robot model.  A bounded Cartesian displacement is
  // represented in the first three joints, which makes unit tests able to
  // observe a changing command while hardware uses the native
  // Pink/Pinocchio model above.
  solution[0] = target[3];
  solution[1] = target[7];
  solution[2] = target[11];
  return allFinite(solution);
}

bool NativeTianjiController::solveArmIK(std::size_t arm, const Pose &target,
                                        const ArmVector &reference,
                                        ArmVector &solution, int &status) {
  status = kIkFailed;
  if (!finitePose(target) || arm >= kArmCount || !allFinite(reference)) {
    status = kIkInvalidPose;
    return false;
  }
  if (simulate_) {
    const bool ok = solveArmIKSimulated(arm, target, reference, solution);
    status = ok ? kIkOk : kIkFailed;
    return ok;
  }
  if (!pink_ik_) {
    status = kIkFailed;
    return false;
  }

  // Build the one-cycle output envelope from the same velocity limits used by
  // the native safety gate.  The independent Pink solver can converge to a
  // complete Cartesian solution several cycles away from the warm-start; as
  // in libtron2rt, clip that candidate toward the reference before publishing
  // it rather than freezing the stream for a single large but valid waypoint.
  ArmVector max_step_rad{};
  const double application_limit =
      max_joint_velocity_deg_s_ * ik_safety_velocity_scale_;
  for (std::size_t joint = 0; joint < kArmDof; ++joint) {
    double velocity_limit = application_limit;
    if (kine_ready_) {
      velocity_limit = std::min(velocity_limit,
                                vendor_velocity_deg_s_[arm][joint] *
                                    ik_safety_velocity_scale_);
    }
    max_step_rad[joint] =
        std::max(1.0e-6, velocity_limit * kDegToRad /
                               static_cast<double>(std::max(1, planner_hz_)));
  }

  PinkIKDiagnostics diagnostics{};
  ArmVector candidate = reference;
  const bool converged =
      pink_ik_->solve(arm, target, reference, max_step_rad, candidate,
                      diagnostics);
  const double max_step =
      application_limit * kDegToRad /
      static_cast<double>(std::max(1, planner_hz_));
  if (converged) {
    ArmVector bounded_candidate = candidate;
    for (std::size_t joint = 0; joint < kArmDof; ++joint) {
      const double delta = bounded_candidate[joint] - reference[joint];
      const double limit = max_step_rad[joint];
      if (std::abs(delta) > limit) {
        bounded_candidate[joint] =
            reference[joint] + std::copysign(limit, delta);
      }
    }
    if (candidateIsSafe(arm, bounded_candidate, reference, max_step)) {
      solution = bounded_candidate;
      // The published command is intentionally the bounded point, so the
      // next planner tick warm-starts from a point on the same continuous
      // branch instead of jumping to the full IK endpoint.
      status = kIkOk;
      return true;
    }
    status = kIkUnsafeCandidate;
    return false;
  }
  // A non-converged native Pink solve is a solver failure, independently of
  // whether its best intermediate candidate happened to be finite.  Keeping
  // this distinction makes -1 useful when diagnosing unreachable/singular
  // targets and reserves -3 for a converged result rejected by hard safety.
  status = kIkFailed;
  return false;
}

void NativeTianjiController::clearCartesianTargetAfterHome(
    std::uint64_t home_target_sequence) noexcept {
  CartesianTarget latest{};
  if (readLatestTarget(latest) && latest.sequence >= home_target_sequence) {
    // Home owns the planner until it completes.  Drop the latest target that
    // was queued before/during Home; Python submits a fresh target after it
    // re-anchors from the post-Home feedback snapshot.
    consumed_sequence_.store(latest.sequence, std::memory_order_release);
  }
  active_cartesian_sequence_ = 0;
  failed_cartesian_sequence_ = 0;
  segment_active_ = {false, false};
  segment_activation_hold_ = {false, false};
  cartesian_segment_complete_ = false;
}

void NativeTianjiController::notifyTrajectoryDone(std::uint64_t sequence) noexcept {
  trajectory_done_sequence_.store(sequence, std::memory_order_release);
  status_cv_.notify_all();
}

void NativeTianjiController::plannerLoop() {
  configureRealtimeThread(true);
  planner_ready_.store(true, std::memory_order_release);
  auto next_tick = std::chrono::steady_clock::now();
  while (!stop_requested_.load(std::memory_order_acquire) &&
         !startup_failed_.load(std::memory_order_acquire)) {
    if (!simulate_ && !io_ready_.load(std::memory_order_acquire)) {
      next_tick = std::chrono::steady_clock::now() +
                  std::chrono::nanoseconds(planner_period_ns_);
      std::this_thread::sleep_until(next_tick);
      continue;
    }
    const auto tick_start = nowNs();
    TianjiState feedback_snapshot{};
    const bool have_feedback = feedback_mailbox_.read(feedback_snapshot);
    if (!planner_seeded_from_feedback_ && have_feedback &&
        feedback_snapshot.valid && allFinite(feedback_snapshot.q)) {
      planner_q_ = feedback_snapshot.q;
      for (std::size_t arm = 0; arm < kArmCount; ++arm) {
        ArmVector seed{};
        for (std::size_t joint = 0; joint < kArmDof; ++joint) {
          seed[joint] = planner_q_[arm * kArmDof + joint];
        }
        planner_pose_[arm] = fkArm(arm, seed);
        segment_start_pose_[arm] = planner_pose_[arm];
        segment_end_pose_[arm] = planner_pose_[arm];
      }
      planner_seeded_from_feedback_ = true;
    }

    // Cancellation is a generation marker.  It is consumed by the planner,
    // so the Python thread never writes the command mailbox concurrently.
    const auto cancel_cartesian =
        cancel_cartesian_sequence_.load(std::memory_order_acquire);
    if (cancel_cartesian != 0) {
      if (active_cartesian_sequence_ <= cancel_cartesian) {
        active_cartesian_sequence_ = 0;
        failed_cartesian_sequence_ = 0;
        cartesian_segment_complete_ = false;
        // A cancellation is also a control hand-off.  The next Cartesian
        // target must be seeded from fresh feedback instead of from the last
        // command that happened to be in flight when cancel() was called.
        segment_active_ = {false, false};
        segment_activation_hold_ = {false, false};
        const auto consumed = consumed_sequence_.load(std::memory_order_acquire);
        consumed_sequence_.store(std::max(consumed, cancel_cartesian),
                                 std::memory_order_release);
      }
      if (next_cartesian_sequence_.load(std::memory_order_acquire) <=
          cancel_cartesian) {
        cancel_cartesian_sequence_.store(0, std::memory_order_release);
      }
    }
    const auto cancel_trajectory =
        cancel_trajectory_sequence_.load(std::memory_order_acquire);
    if (cancel_trajectory != 0) {
      if (active_trajectory_seq_ <= cancel_trajectory) {
        active_trajectory_seq_ = 0;
        trajectory_active_.store(false, std::memory_order_release);
        active_trajectory_sequence_.store(0, std::memory_order_release);
        if (mode_.load(std::memory_order_acquire) != ControllerMode::Fault) {
          mode_.store(ControllerMode::Hold, std::memory_order_release);
        }
      }
      if (next_trajectory_sequence_.load(std::memory_order_acquire) <=
          cancel_trajectory) {
        cancel_trajectory_sequence_.store(0, std::memory_order_release);
      }
    }

    JointTrajectory trajectory{};
    const bool have_trajectory = readLatestTrajectory(trajectory);
    if (have_trajectory && trajectory.sequence > last_started_trajectory_seq_ &&
        trajectory.sequence >
            cancel_trajectory_sequence_.load(std::memory_order_acquire)) {
      active_trajectory_seq_ = trajectory.sequence;
      last_started_trajectory_seq_ = trajectory.sequence;
      // Joint trajectories (including Home) start at the newest measured
      // position, not at a stale planner target.  If tracking was interrupted
      // or a prior Cartesian target timed out, this prevents the first Home
      // sample from jumping toward an old command -- and makes a
      // zero-initialized process value incapable of becoming a hardware start.
      JointVector measured_start = planner_q_;
      if (have_feedback && feedback_snapshot.valid &&
          allFinite(feedback_snapshot.q)) {
        bool measured_within_limits = true;
        for (std::size_t arm = 0; arm < kArmCount; ++arm) {
          ArmVector measured_arm{};
          for (std::size_t joint = 0; joint < kArmDof; ++joint) {
            measured_arm[joint] =
                feedback_snapshot.q[arm * kArmDof + joint];
          }
          measured_within_limits =
              measured_within_limits && jointWithinLimits(arm, measured_arm);
        }
        if (measured_within_limits) {
          measured_start = feedback_snapshot.q;
        }
      }
      trajectory_start_q_ = measured_start;
      planner_q_ = measured_start;
      for (std::size_t arm = 0; arm < kArmCount; ++arm) {
        ArmVector start_arm{};
        for (std::size_t joint = 0; joint < kArmDof; ++joint) {
          start_arm[joint] = measured_start[arm * kArmDof + joint];
        }
        planner_pose_[arm] = fkArm(arm, start_arm);
        segment_start_pose_[arm] = planner_pose_[arm];
        segment_end_pose_[arm] = planner_pose_[arm];
        segment_active_[arm] = false;
        segment_activation_hold_[arm] = false;
      }
      trajectory_target_q_ = trajectory.target;
      const double requested =
          static_cast<double>(trajectory.duration_ns) / 1.0e9;
      double required = requested;
      for (std::size_t i = 0; i < kJointCount; ++i) {
        const std::size_t arm = i / kArmDof;
        const std::size_t joint = i % kArmDof;
        const double delta_deg =
            std::abs(trajectory_target_q_[i] - trajectory_start_q_[i]) * kRadToDeg;
        double velocity_limit = max_joint_velocity_deg_s_;
        double acceleration_limit = std::numeric_limits<double>::infinity();
        if (kine_ready_) {
          velocity_limit =
              std::min(velocity_limit, vendor_velocity_deg_s_[arm][joint]);
          acceleration_limit = vendor_acceleration_deg_s2_[arm][joint];
        }
        // Quintic minimum jerk peaks at 1.875 * delta / duration.
        required = std::max(
            required, 1.875 * delta_deg /
                          std::max(1.0, velocity_limit));
        // Its peak acceleration is 10 / sqrt(3) * delta / duration^2.
        if (std::isfinite(acceleration_limit)) {
          required = std::max(
              required,
              std::sqrt((10.0 / std::sqrt(3.0)) * delta_deg /
                        std::max(1.0, acceleration_limit)));
        }
      }
      trajectory_start_ns_ = tick_start;
      const auto requested_ns = static_cast<std::uint64_t>(
          std::llround(std::max(
                           static_cast<double>(planner_period_ns_) / 1.0e9,
                           required) *
                       1.0e9));
      segment_duration_ns_ = std::max(requested_ns, planner_period_ns_);
      trajectory_active_.store(true, std::memory_order_release);
      active_trajectory_sequence_.store(trajectory.sequence,
                                        std::memory_order_release);
      mode_.store(ControllerMode::Home, std::memory_order_release);
      // Do not consume Cartesian targets while Home is active.
    }

    const bool in_home = trajectory_active_.load(std::memory_order_acquire) &&
                         active_trajectory_seq_ != 0;

    JointCommand command{};
    command.q = planner_q_;
    command.timestamp_ns = tick_start;
    command.source_sequence = 0;
    command.hold = false;

    if (in_home) {
      const double alpha = clamp01(
          static_cast<double>(tick_start - trajectory_start_ns_) /
          static_cast<double>(std::max<std::uint64_t>(1, segment_duration_ns_)));
      const double progress = minimumJerk(alpha);
      for (std::size_t i = 0; i < kJointCount; ++i) {
        command.q[i] = trajectory_start_q_[i] +
                       progress * (trajectory_target_q_[i] - trajectory_start_q_[i]);
      }
      planner_q_ = command.q;
      if (alpha >= 1.0 - 1.0e-9) {
        command.q = trajectory_target_q_;
        planner_q_ = command.q;
        notifyTrajectoryDone(active_trajectory_seq_);
        clearCartesianTargetAfterHome(
            home_started_target_sequence_.load(std::memory_order_acquire));
        trajectory_active_.store(false, std::memory_order_release);
        active_trajectory_sequence_.store(0, std::memory_order_release);
        mode_.store(ControllerMode::Streaming, std::memory_order_release);
        active_trajectory_seq_ = 0;
      }
    } else {
      CartesianTarget target{};
      const bool have_target = readLatestTarget(target);
      if (have_target && target.sequence > consumed_sequence_.load(
                                          std::memory_order_acquire) &&
          target.sequence > cancel_cartesian_sequence_.load(
                                 std::memory_order_acquire)) {
        if (target.sequence > active_cartesian_sequence_) {
          // A fresh producer sample clears the previous watchdog episode.  A
          // later timeout will increment the counter exactly once until a new
          // sample is observed.
          command_timed_out_.store(false, std::memory_order_release);
          // Measure fresh producer arrival intervals, not VR timestamps.  The
          // Python BiPico4 loop does not submit unchanged timestamped packets,
          // so this is the actual VR waypoint interval: an early waypoint gets
          // a shorter segment and a late waypoint gets a longer one.  Keep a
          // native-period floor for the impossible/too-fast case.
          double measured = std::max(
              static_cast<double>(planner_period_ns_) / 1.0e9,
              static_cast<double>(target.interval_ns) / 1.0e9);
          // Snapshot the previous active state before updating this segment.
          // An inactive->active transition is a control hand-off: the first
          // active sample is consumed as a measured-position anchor and must
          // not be interpreted as a request to move to a stale hand pose.
          const auto was_active = segment_active_;
          std::array<bool, 2> activation_hold{{false, false}};
          // An inactive -> active edge means that control has just been
          // re-enabled (or that a communication gap was recovered).  Do not
          // let the old planner command become the first command after that
          // hand-off: re-anchor the affected arm to the newest measured q and
          // its FK pose before constructing the interpolation segment.  This
          // is the native counterpart of the Pico4 adapter's re-anchor and
          // closes the stale-target path that could move an arm as the enable
          // pedal was pressed.
          bool reanchor_failed = false;
          for (std::size_t arm = 0; arm < kArmCount; ++arm) {
            if (target.active[arm] && !was_active[arm]) {
              activation_hold[arm] = true;
              bool measured_ok = have_feedback && feedback_snapshot.valid &&
                                 allFinite(feedback_snapshot.q);
              ArmVector measured_arm{};
              if (measured_ok) {
                for (std::size_t joint = 0; joint < kArmDof; ++joint) {
                  measured_arm[joint] =
                      feedback_snapshot.q[arm * kArmDof + joint];
                }
                measured_ok = jointWithinLimits(arm, measured_arm) &&
                              j67WithinLimits(arm, measured_arm);
              }
              if (measured_ok) {
                for (std::size_t joint = 0; joint < kArmDof; ++joint) {
                  planner_q_[arm * kArmDof + joint] = measured_arm[joint];
                }
                planner_pose_[arm] = fkArm(arm, measured_arm);
              } else {
                reanchor_failed = true;
              }
            }
          }
          for (std::size_t arm = 0; arm < kArmCount; ++arm) {
            segment_start_pose_[arm] = planner_pose_[arm];
            // Keep an activation-edge arm exactly at its freshly measured
            // pose for one segment.  The next producer waypoint is then the
            // first sample allowed to generate Cartesian motion.
            segment_end_pose_[arm] =
                activation_hold[arm] ? planner_pose_[arm] : target.pose[arm];
            if (!target.active[arm]) {
              continue;
            }
            if (activation_hold[arm]) {
              continue;
            }
            const double distance = poseTranslationDistance(
                segment_start_pose_[arm], segment_end_pose_[arm]);
            const double angle = poseAngularDistance(segment_start_pose_[arm],
                                                     segment_end_pose_[arm]);
            measured = std::max(measured,
                                distance / max_tcp_linear_velocity_m_s_);
            measured = std::max(measured,
                                angle / max_tcp_angular_velocity_rad_s_);
          }
          segment_duration_ns_ = std::max<std::uint64_t>(
              planner_period_ns_, static_cast<std::uint64_t>(
                                       std::llround(measured * 1.0e9)));
          segment_start_ns_ = tick_start;
          active_cartesian_sequence_ = target.sequence;
          failed_cartesian_sequence_ = 0;
          cartesian_segment_complete_ = false;
          segment_active_ = target.active;
          segment_activation_hold_ = activation_hold;
          if (reanchor_failed) {
            // Holding the previous safe command is preferable to guessing a
            // seed when feedback disappeared exactly at an enable edge.  A
            // later waypoint will retry after the I/O watchdog has recovered.
            // Keep the edge inactive so that retrying that later waypoint
            // performs the feedback re-anchor again.
            segment_active_ = {false, false};
            segment_activation_hold_ = {false, false};
            failed_cartesian_sequence_ = target.sequence;
          }
          for (std::size_t arm = 0; arm < kArmCount; ++arm) {
            if (!target.active[arm]) {
              segment_start_pose_[arm] = planner_pose_[arm];
              segment_end_pose_[arm] = planner_pose_[arm];
            }
          }
        }
        if (target.sequence == failed_cartesian_sequence_ ||
            cartesian_segment_complete_) {
          command.hold = true;
        } else {
          const double elapsed = static_cast<double>(
              tick_start - segment_start_ns_) / 1.0e9;
          const double alpha = clamp01(
              (elapsed + static_cast<double>(planner_period_ns_) / 1.0e9) /
              (static_cast<double>(std::max<std::uint64_t>(1, segment_duration_ns_)) /
               1.0e9));
          const Pose left_pose = interpolatePose(segment_start_pose_[0],
                                                 segment_end_pose_[0], alpha);
          const Pose right_pose = interpolatePose(segment_start_pose_[1],
                                                  segment_end_pose_[1], alpha);
          const ArmVector left_ref{planner_q_[0], planner_q_[1], planner_q_[2],
                                   planner_q_[3], planner_q_[4], planner_q_[5],
                                   planner_q_[6]};
          const ArmVector right_ref{planner_q_[7], planner_q_[8], planner_q_[9],
                                    planner_q_[10], planner_q_[11], planner_q_[12],
                                    planner_q_[13]};
          ArmVector left_solution = left_ref;
          ArmVector right_solution = right_ref;
          int left_status = kIkOk;
          int right_status = kIkOk;
          const bool left_ok = !segment_active_[0] ||
                               segment_activation_hold_[0] ||
                               solveArmIK(0, left_pose, left_ref, left_solution,
                                          left_status);
          const bool right_ok = !segment_active_[1] ||
                                segment_activation_hold_[1] ||
                                solveArmIK(1, right_pose, right_ref,
                                           right_solution, right_status);
          last_ik_status_.store(left_ok && right_ok ? kIkOk
                                                     : (left_ok ? right_status
                                                                 : left_status),
                                std::memory_order_release);
          if (!left_ok || !right_ok) {
            ik_failure_count_.fetch_add(1, std::memory_order_acq_rel);
            failed_cartesian_sequence_ = target.sequence;
            command.hold = true;
          } else {
            ik_success_count_.fetch_add(1, std::memory_order_acq_rel);
            for (std::size_t i = 0; i < kArmDof; ++i) {
              command.q[i] = left_solution[i];
              command.q[kArmDof + i] = right_solution[i];
            }
            planner_q_ = command.q;
            planner_pose_[0] = fkArm(0, left_solution);
            planner_pose_[1] = fkArm(1, right_solution);
            command.source_sequence = target.sequence;
            if (alpha >= 1.0 - 1.0e-9) {
              consumed_sequence_.store(target.sequence,
                                       std::memory_order_release);
              cartesian_segment_complete_ = true;
              mode_.store(ControllerMode::Hold, std::memory_order_release);
            } else if (mode_.load(std::memory_order_acquire) !=
                       ControllerMode::Fault) {
              mode_.store(ControllerMode::Streaming,
                          std::memory_order_release);
            }
          }
        }
      } else {
        command.hold = true;
      }

      const auto last_arrival = last_target_arrival_ns_.load(std::memory_order_acquire);
      const bool input_timed_out =
          last_arrival != 0 && tick_start > last_arrival + command_timeout_ns_;
      if (input_timed_out) {
        bool was_timed_out = command_timed_out_.exchange(
            true, std::memory_order_acq_rel);
        if (!was_timed_out) {
          command_timeout_count_.fetch_add(1, std::memory_order_acq_rel);
        }
        command.q = planner_q_;
        command.hold = true;
        // Treat a timeout as an implicit release.  If input resumes after a
        // network/producer gap, the first new active target will therefore
        // take the measured-feedback re-anchor path above rather than
        // continuing a stale Cartesian segment.
        segment_active_ = {false, false};
        segment_activation_hold_ = {false, false};
        cartesian_segment_complete_ = false;
        if (mode_.load(std::memory_order_acquire) != ControllerMode::Fault) {
          mode_.store(ControllerMode::Hold, std::memory_order_release);
        }
      }
    }
    command.timestamp_ns = tick_start;
    publishCommand(command);
    // The first planner command is a measured-position hold.  Only after it
    // has been published may the I/O thread consume the command mailbox;
    // before this point it mirrors its own measured feedback instead of the
    // zero-filled startup placeholder.
    if (planner_seeded_from_feedback_) {
      planner_command_ready_.store(true, std::memory_order_release);
      status_cv_.notify_all();
    }
    TianjiState state{};
    if (have_feedback && feedback_snapshot.valid &&
        allFinite(feedback_snapshot.q) && allFinite(feedback_snapshot.dq)) {
      state.q = feedback_snapshot.q;
      state.dq = feedback_snapshot.dq;
    } else {
      state.q = planner_q_;
      state.dq.fill(0.0);
    }
    state.last_command_q = command.q;
    state.tcp_left = planner_pose_[0];
    state.tcp_right = planner_pose_[1];
    state.timestamp_ns = tick_start;
    state.fault_code = fault_code_.load(std::memory_order_acquire);
    state.valid = simulate_ || io_ready_.load(std::memory_order_acquire);
    publishState(state);

    next_tick += std::chrono::nanoseconds(planner_period_ns_);
    const auto current_time = std::chrono::steady_clock::now();
    if (current_time > next_tick +
                            std::chrono::nanoseconds(planner_period_ns_)) {
      scheduler_degraded_.store(true, std::memory_order_release);
      scheduler_warning_count_.fetch_add(1, std::memory_order_acq_rel);
      // Skip missed ticks; never replay historical samples in a burst.
      next_tick = current_time;
    }
    std::this_thread::sleep_until(next_tick);
  }
}

void NativeTianjiController::updateHardwareState(std::uint64_t now_ns) {
  DCSS dcss{};
  if (!OnGetBuf(&dcss)) {
    setFault(10, "Marvin SDK OnGetBuf failed");
    return;
  }
  TianjiState feedback{};
  feedback.timestamp_ns = now_ns;
  feedback.valid = true;
  int error = 0;
  for (std::size_t arm = 0; arm < kArmCount; ++arm) {
    error = std::max(error, static_cast<int>(dcss.m_State[arm].m_ERRCode));
    const int serial = dcss.m_Out[arm].m_OutFrameSerial;
    if (serial > 0 && serial != io_feedback_serial_[arm]) {
      io_feedback_serial_[arm] = serial;
      io_feedback_stable_frames_[arm] =
          std::min<unsigned int>(io_feedback_stable_frames_[arm] + 1U,
                                 kRequiredFreshFeedbackFrames);
      io_feedback_change_ns_[arm] = now_ns;
    }
    for (std::size_t joint = 0; joint < kArmDof; ++joint) {
      const double q_deg = dcss.m_Out[arm].m_FB_Joint_Pos[joint];
      const double dq_deg = dcss.m_Out[arm].m_FB_Joint_Vel[joint];
      if (!finiteValue(q_deg) || !finiteValue(dq_deg)) {
        setFault(11, "Marvin SDK returned NaN/Inf joint feedback");
        return;
      }
      feedback.q[arm * kArmDof + joint] = q_deg * kDegToRad;
      feedback.dq[arm * kArmDof + joint] = dq_deg * kDegToRad;
      io_feedback_q_[arm * kArmDof + joint] = feedback.q[arm * kArmDof + joint];
      io_feedback_dq_[arm * kArmDof + joint] = feedback.dq[arm * kArmDof + joint];
    }
  }
  feedback.fault_code = error;
  feedback_mailbox_.write(feedback);
  if (!feedback_ready_.exchange(true, std::memory_order_acq_rel)) {
    // Keep the I/O-side fallback command anchored to the actual measured
    // position.  This assignment happens in the sole I/O thread, so it is
    // race-free and prevents an initial command from driving to zero.
    io_last_sent_q_ = feedback.q;
  }
  if (error != 0) {
    // ``m_ERRCode`` is an arm-level controller status, not the per-servo
    // hexadecimal code documented by the vendor.  Read the seven-axis
    // diagnostics as well, otherwise a value such as ``2`` is impossible to
    // interpret from the Python log.  This path is exceptional, so the SDK
    // diagnostic calls and small formatting buffer are acceptable here.
    long servo_error[kArmCount][kArmDof]{};
    OnGetServoErr_A(servo_error[0]);
    OnGetServoErr_B(servo_error[1]);
    char message[256]{};
    std::snprintf(
        message, sizeof(message),
        "Tianji arm fault: A(state=%d,err=%d,servo=[0x%lx,0x%lx,0x%lx,0x%lx,0x%lx,0x%lx,0x%lx]) "
        "B(state=%d,err=%d,servo=[0x%lx,0x%lx,0x%lx,0x%lx,0x%lx,0x%lx,0x%lx])",
        dcss.m_State[0].m_CurState, dcss.m_State[0].m_ERRCode,
        servo_error[0][0], servo_error[0][1], servo_error[0][2],
        servo_error[0][3], servo_error[0][4], servo_error[0][5],
        servo_error[0][6], dcss.m_State[1].m_CurState,
        dcss.m_State[1].m_ERRCode, servo_error[1][0], servo_error[1][1],
        servo_error[1][2], servo_error[1][3], servo_error[1][4],
        servo_error[1][5], servo_error[1][6]);
    setFault(error, message);
  }
}

void NativeTianjiController::updateSimulatedState(const JointCommand &command,
                                                  std::uint64_t now_ns) {
  // A first-order state mirror gives tests deterministic feedback without
  // adding a second timing source.  Hardware never enters this branch.
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const double error = command.q[i] - io_feedback_q_[i];
    io_feedback_dq_[i] = error * static_cast<double>(io_hz_);
    io_feedback_q_[i] += error * 0.25;
  }
  TianjiState feedback{};
  feedback.q = io_feedback_q_;
  feedback.dq = io_feedback_dq_;
  feedback.timestamp_ns = now_ns;
  feedback.valid = true;
  feedback.fault_code = 0;
  feedback_mailbox_.write(feedback);
  feedback_ready_.store(true, std::memory_order_release);
}

void NativeTianjiController::ioLoop() {
  configureRealtimeThread(false);
  bool setup_ok = true;
  constexpr std::uint64_t kStartupTimeoutNs = 3'000'000'000ULL;
  constexpr std::uint64_t kCommandSlotTimeoutNs = 100'000'000ULL;

  // Configure Marvin's process-side logger before OnLinkTo starts its
  // asynchronous timer callback.  The SDK enables this logger by default and
  // would otherwise print every OnSetJointCmdPos_* call.  Its explicit
  // [ERROR]/[WARNING] diagnostics are not guarded by this flag; wrapper faults
  // are surfaced through TianjiStatus as well.  Do not touch the vendor
  // singleton in simulation mode.
  if (!simulate_) {
    if (vendorVerboseLoggingEnabled()) {
      OnLocalLogOn();
    } else {
      OnLocalLogOff();
    }
  }

  // Marvin's timer thread owns the actual UDP send.  OnSetSend() only marks
  // the shared command slot as pending; until the timer consumes it,
  // OnClearSet() returns false.  Treating that expected one-millisecond
  // interval as a hard error made startup and streaming depend on scheduler
  // luck (and was the source of intermittent "controller not running"
  // reports).  All retries stay inside this SDK/I/O thread and are bounded so
  // a genuinely wedged SDK still trips the native fault path.
  auto wait_for_command_slot = [&](std::uint64_t deadline_ns) -> bool {
    while (!stop_requested_.load(std::memory_order_acquire)) {
      if (OnClearSet()) {
        return true;
      }
      if (nowNs() >= deadline_ns) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  };

  auto send_joint_command = [&](const JointVector &q,
                                std::uint64_t timeout_ns) -> bool {
    const auto deadline = nowNs() + timeout_ns;
    while (!stop_requested_.load(std::memory_order_acquire)) {
      if (!wait_for_command_slot(deadline)) {
        return false;
      }
      double left[kArmDof]{};
      double right[kArmDof]{};
      for (std::size_t i = 0; i < kArmDof; ++i) {
        left[i] = q[i] * kRadToDeg;
        right[i] = q[kArmDof + i] * kRadToDeg;
      }
      if (OnSetJointCmdPos_A(left) && OnSetJointCmdPos_B(right) &&
          OnSetSend()) {
        return true;
      }
      // A partial append is discarded on the next iteration.  This is safe
      // because this is the only thread that accesses the vendor command
      // buffer.
      if (nowNs() >= deadline) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  };

  auto measured_feedback_is_safe = [&]() -> bool {
    if (!feedback_ready_.load(std::memory_order_acquire) ||
        !allFinite(io_feedback_q_)) {
      return false;
    }
    for (std::size_t arm = 0; arm < kArmCount; ++arm) {
      ArmVector measured{};
      for (std::size_t joint = 0; joint < kArmDof; ++joint) {
        measured[joint] = io_feedback_q_[arm * kArmDof + joint];
      }
      if (!jointWithinLimits(arm, measured) ||
          !j67WithinLimits(arm, measured)) {
        return false;
      }
    }
    return true;
  };

  // The controller reads its own current feedback when entering joint
  // impedance.  Do not write a position command while it is still in its old
  // mode: although the value is measured and therefore not a zero target,
  // some firmware can interpret that command through the previous control
  // law.  The first host position command is sent only after both arms report
  // joint-impedance PD, and is always the latest measured q.
  auto send_joint_hold = [&]() -> bool {
    if (!measured_feedback_is_safe() ||
        !send_joint_command(io_feedback_q_, kCommandSlotTimeoutNs)) {
      return false;
    }
    io_last_sent_q_ = io_feedback_q_;
    io_has_sent_command_ = true;
    return true;
  };

  if (!simulate_) {
    std::array<unsigned char, 4> ip{};
    if (!parseIp(robot_ip_, ip)) {
      setFault(1, "invalid Tianji controller IP address");
      setup_ok = false;
    } else {
      // Do not use the vendor Connect() convenience wrapper here.  It only
      // waits five milliseconds for its first frame, so an otherwise healthy
      // controller can randomly fail during process startup.  The lower-level
      // link API gives this worker ownership of the startup deadline and lets
      // us anchor the first command to measured feedback before enabling PD.
      vendor_session_created_ = true;
      vendor_connected_ = OnLinkTo(ip[0], ip[1], ip[2], ip[3]);
      if (!vendor_connected_) {
        setFault(2, "Marvin SDK OnLinkTo failed (UDP port may be occupied)");
        setup_ok = false;
      } else {
        // Match the vendor Python examples: OnLinkTo does not clear a
        // latched arm/servo fault.  A previous emergency stop can therefore
        // leave one arm in ARM_STATE_ERROR and make the next launch appear
        // dead until another client happens to call check_error_and_clear().
        // Clear both sides immediately after link ownership is acquired,
        // before waiting for fresh feedback or entering impedance mode.
        if (!OnClearSet()) {
          setFault(2, "Marvin SDK OnClearSet failed while clearing startup errors");
          setup_ok = false;
        } else {
          OnClearErr_A();
          OnClearErr_B();
          if (!OnSetSend()) {
            setFault(2, "Marvin SDK OnSetSend failed while clearing startup errors");
            setup_ok = false;
          }
        }
      }
      if (setup_ok && vendor_connected_) {
        const auto deadline = nowNs() + kFeedbackStartupTimeoutNs;
        int previous_serial[kArmCount]{0, 0};
        unsigned int stable_frames[kArmCount]{0U, 0U};
        // OnLinkTo starts the vendor timer immediately, but the controller
        // may still be returning a packet generated before this process took
        // ownership of the socket.  Drain/ignore the first snapshot and use
        // only serials observed after the link call.
        bool observed_packet[kArmCount]{false, false};
        while (!stop_requested_.load(std::memory_order_acquire) &&
               nowNs() < deadline &&
               (stable_frames[0] < kRequiredFreshFeedbackFrames ||
                stable_frames[1] < kRequiredFreshFeedbackFrames)) {
          DCSS dcss{};
          if (!OnGetBuf(&dcss)) {
            setFault(10, "Marvin SDK OnGetBuf failed while waiting for feedback");
            setup_ok = false;
            break;
          }
          const int left_serial = dcss.m_Out[0].m_OutFrameSerial;
          const int right_serial = dcss.m_Out[1].m_OutFrameSerial;
          const int serials[kArmCount] = {left_serial, right_serial};
          const auto fresh_time = nowNs();
          for (std::size_t arm = 0; arm < kArmCount; ++arm) {
            if (serials[arm] > 0 && !observed_packet[arm]) {
              observed_packet[arm] = true;
              previous_serial[arm] = serials[arm];
              continue;
            }
            if (serials[arm] > 0 && serials[arm] != previous_serial[arm]) {
              stable_frames[arm] = std::min<unsigned int>(
                  stable_frames[arm] + 1U, kRequiredFreshFeedbackFrames);
              io_feedback_serial_[arm] = serials[arm];
              io_feedback_stable_frames_[arm] = stable_frames[arm];
              io_feedback_change_ns_[arm] = fresh_time;
            } else if (serials[arm] <= 0) {
              stable_frames[arm] = 0U;
              io_feedback_stable_frames_[arm] = 0U;
            }
            previous_serial[arm] = serials[arm];
          }
          if (stable_frames[0] < kRequiredFreshFeedbackFrames ||
              stable_frames[1] < kRequiredFreshFeedbackFrames) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        }
        if (setup_ok &&
            (stable_frames[0] < kRequiredFreshFeedbackFrames ||
             stable_frames[1] < kRequiredFreshFeedbackFrames)) {
          setFault(2,
                   "Marvin SDK link established but feedback did not advance from both arms");
          setup_ok = false;
        }
      }
    }
    if (setup_ok && vendor_connected_) {
      double stiffness[kArmDof]{};
      double damping[kArmDof]{};
      for (std::size_t i = 0; i < kArmDof; ++i) {
        stiffness[i] = pd_joint_stiffness_[i];
        damping[i] = pd_joint_damping_[i];
      }

      // Do not configure or send a target until a real feedback frame has
      // arrived.  This is deliberately done in the SDK/I/O thread so no
      // Python-side snapshot can race the vendor singleton.
      updateHardwareState(nowNs());
      if (fault_code_.load(std::memory_order_acquire) != 0 ||
          !feedback_ready_.load(std::memory_order_acquire)) {
        setup_ok = false;
      }

      if (setup_ok && !measured_feedback_is_safe()) {
        setFault(3, "Marvin SDK returned an unsafe measured joint position during startup");
        setup_ok = false;
      }

      // SetImpJointMode itself checks the low-speed flag, but its five
      // millisecond internal window is shorter than a real arm's settling
      // time.  Wait for stationary feedback and retry transient rejections
      // within one bounded startup window instead of failing nondeterministic-
      // ally on the first call.
      auto configure_arm = [&](char arm, std::size_t arm_index) -> bool {
        const auto deadline = nowNs() + kStartupTimeoutNs;
        while (nowNs() < deadline && !stop_requested_.load(std::memory_order_acquire)) {
          DCSS dcss{};
          if (!OnGetBuf(&dcss)) {
            setFault(10, "Marvin SDK OnGetBuf failed while configuring PD mode");
            return false;
          }
          if (dcss.m_State[arm_index].m_ERRCode != 0) {
            setFault(static_cast<int>(dcss.m_State[arm_index].m_ERRCode),
                     "Tianji controller reported an arm fault during startup");
            return false;
          }
          if (dcss.m_Out[arm_index].m_LowSpdFlag == 1) {
            // SetImpJointMode performs the vendor-side command-slot and
            // arm/servo checks itself.  An extra OnClearSet() here races the
            // SDK timer and was a source of intermittent startup failures.
            if (SetImpJointMode(arm, velocity_ratio_, acceleration_ratio_, stiffness,
                                damping)) {
              return true;
            }
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
      };

      if (setup_ok && !configure_arm('A', 0)) {
        setFault(3, "Marvin SDK arm A PD mode configuration timed out");
        setup_ok = false;
      }
      if (setup_ok && !configure_arm('B', 1)) {
        setFault(3, "Marvin SDK arm B PD mode configuration timed out");
        setup_ok = false;
      }

      if (setup_ok) {
        // FX_OnSetVelEstStep only appends a parameter write to the current
        // command buffer.  Both arms must be committed as one transaction;
        // calling the two helpers without OnClearSet/OnSetSend silently left
        // the estimator disabled in the previous implementation.
        bool estimator_ok = wait_for_command_slot(nowNs() + kCommandSlotTimeoutNs);
        if (estimator_ok) {
          estimator_ok = FX_OnSetVelEstStep('A', 5);
        }
        if (estimator_ok) {
          estimator_ok = FX_OnSetVelEstStep('B', 5);
        }
        if (estimator_ok) {
          estimator_ok = OnSetSend();
        }
        if (!estimator_ok) {
          setFault(3, "Marvin SDK velocity estimator configuration failed");
          setup_ok = false;
        }
      }

      // Re-anchor once more after the mode/estimator transaction.  This also
      // makes the first PD target deterministic if the controller adjusted a
      // command buffer while switching modes.
      if (setup_ok && !send_joint_hold()) {
        setFault(3, "Marvin SDK rejected the post-configuration position hold");
        setup_ok = false;
      }
      if (setup_ok) {
        // Preserve the old adapter's short estimator/mode settling window, but
        // keep it entirely inside the I/O worker.  During this interval only
        // the latest measured position is sent; no planner target can move an
        // arm before the feedback-to-command path has settled.
        const auto settle_deadline = nowNs() + kPdEstimatorSettleNs;
        auto next_hold = nowNs();
        while (!stop_requested_.load(std::memory_order_acquire) &&
               nowNs() < settle_deadline) {
          const auto tick = nowNs();
          updateHardwareState(tick);
          if (fault_code_.load(std::memory_order_acquire) != 0) {
            setup_ok = false;
            break;
          }
          if (tick >= next_hold) {
            if (!send_joint_hold()) {
              setFault(3, "Marvin SDK rejected the estimator settling hold");
              setup_ok = false;
              break;
            }
            next_hold = tick + command_period_ns_;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (setup_ok && !stop_requested_.load(std::memory_order_acquire)) {
          DCSS dcss{};
          if (!OnGetBuf(&dcss) ||
              dcss.m_State[0].m_CurState != ARM_STATE_TORQ ||
              dcss.m_State[1].m_CurState != ARM_STATE_TORQ ||
              dcss.m_In[0].m_ImpType != ARM_IMP_JOINT ||
              dcss.m_In[1].m_ImpType != ARM_IMP_JOINT) {
            setFault(3, "Tianji controller did not remain in dual-arm joint impedance mode");
            setup_ok = false;
          }
        }
      }
    }
  } else {
    io_feedback_q_.fill(0.0);
    io_feedback_dq_.fill(0.0);
    JointCommand simulation_hold{};
    simulation_hold.q.fill(0.0);
    simulation_hold.hold = true;
    updateSimulatedState(simulation_hold, nowNs());
  }
  if (!setup_ok) {
    io_ready_.store(false, std::memory_order_release);
    startup_failed_.store(true, std::memory_order_release);
    status_cv_.notify_all();
  } else {
    if (!simulate_) {
      updateHardwareState(nowNs());
    }
    if (fault_code_.load(std::memory_order_acquire) != 0 ||
        stop_requested_.load(std::memory_order_acquire)) {
      setup_ok = false;
      io_ready_.store(false, std::memory_order_release);
      startup_failed_.store(true, std::memory_order_release);
      status_cv_.notify_all();
    } else {
      io_ready_.store(true, std::memory_order_release);
      mode_.store(ControllerMode::Streaming, std::memory_order_release);
      status_cv_.notify_all();
    }
  }

  std::uint64_t next_tick = nowNs();
  std::uint64_t next_command = next_tick;
  // The controller publishes feedback at roughly 1 kHz, but a short burst of
  // host scheduling/network jitter is normal.  Use a 500 ms hard watchdog,
  // matching the command timeout, instead of faulting after a few missed
  // packets.  Startup still requires three advancing frames.
  const auto feedback_stale_timeout_ns =
      std::max<std::uint64_t>(command_timeout_ns_, 500'000'000ULL);
  while (!stop_requested_.load(std::memory_order_acquire) && setup_ok) {
    const auto tick = nowNs();
    if (!simulate_) {
      updateHardwareState(tick);
      if (fault_code_.load(std::memory_order_acquire) != 0 ||
          stop_requested_.load(std::memory_order_acquire)) {
        break;
      }
      if (io_feedback_change_ns_[0] == 0 || io_feedback_change_ns_[1] == 0 ||
          tick > io_feedback_change_ns_[0] + feedback_stale_timeout_ns ||
          tick > io_feedback_change_ns_[1] + feedback_stale_timeout_ns) {
        setFault(10, "Marvin SDK feedback serial stopped advancing");
        break;
      }
    }
    JointCommand command{};
    // Until the planner has published its feedback-seeded first command,
    // mirror the measured position locally.  In particular, never consume
    // the zero-filled startup mailbox on a physical controller.
    if (!planner_command_ready_.load(std::memory_order_acquire)) {
      command.q = io_feedback_q_;
      command.hold = true;
      command.timestamp_ns = tick;
    } else if (!readLatestCommand(command)) {
      command.q = io_last_sent_q_;
      command.hold = true;
    }
    if (!allFinite(command.q)) {
      setFault(11, "non-finite native joint command");
      break;
    }
    // The planner owns all normal velocity shaping.  The I/O worker keeps one
    // deliberately coarse hard guard so a stale mailbox, unit conversion
    // error, or unseeded command can never turn into a large physical jump.
    bool command_limits_ok = true;
    for (std::size_t arm = 0; arm < kArmCount; ++arm) {
      ArmVector arm_command{};
      for (std::size_t joint = 0; joint < kArmDof; ++joint) {
        arm_command[joint] = command.q[arm * kArmDof + joint];
      }
      command_limits_ok = command_limits_ok &&
                          jointWithinLimits(arm, arm_command) &&
                          j67WithinLimits(arm, arm_command);
    }
    if (!command_limits_ok) {
      setFault(11, "native joint command exceeds configured/vendor limits");
      break;
    }
    if (io_has_sent_command_) {
      double largest_jump = 0.0;
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        largest_jump = std::max(largest_jump,
                                std::abs(command.q[joint] - io_last_sent_q_[joint]));
      }
      if (largest_jump > kMaxCommandJumpRad) {
        // A discontinuous candidate is rejected as a command, not promoted
        // to a new physical motion and not escalated to a terminal fault.
        // The planner will get another seed/waypoint on the next native tick;
        // holding here avoids both a dangerous jump and needless intermittent
        // session failures when a Pink/Pinocchio IK waypoint is briefly invalid.
        command.q = io_last_sent_q_;
        command.hold = true;
      }
    }
    if (simulate_) {
      updateSimulatedState(command, tick);
    }
    if (tick >= next_command) {
      bool sent = true;
      if (!simulate_) {
        // Allow a short bounded wait for Marvin's asynchronous timer to
        // release the previous command slot.  A transient busy slot is not a
        // robot fault; an unrecoverable transaction failure still stops the
        // stream through the hard safety path below.
        sent = send_joint_command(command.q,
                                  std::max<std::uint64_t>(command_period_ns_,
                                                          kCommandSlotTimeoutNs));
      }
      if (!sent) {
        setFault(12, "Marvin SDK joint command transaction failed");
        break;
      }
      io_last_sent_q_ = command.q;
      io_has_sent_command_ = true;
      last_command_sent_ns_.store(tick, std::memory_order_release);
      next_command = tick + command_period_ns_;
    }
    next_tick += io_period_ns_;
    if (tick > next_tick + io_period_ns_) {
      scheduler_degraded_.store(true, std::memory_order_release);
      scheduler_warning_count_.fetch_add(1, std::memory_order_acq_rel);
      next_tick = tick;
    }
    std::this_thread::sleep_until(
        std::chrono::steady_clock::time_point(std::chrono::nanoseconds(next_tick)));
  }

  // An unexpected worker exit before the first safe command is a startup
  // failure, not a successful idle state.  Mark it before releasing the SDK
  // so Python receives a useful error instead of "not running" on its first
  // action.
  if (setup_ok && !planner_command_ready_.load(std::memory_order_acquire) &&
      !stop_requested_.load(std::memory_order_acquire) &&
      fault_code_.load(std::memory_order_acquire) == 0) {
    setFault(13, "Tianji I/O thread exited before the first safe command");
  }
  cleanupVendorSession(emergency_requested_.load(std::memory_order_acquire));
  io_ready_.store(false, std::memory_order_release);
  running_.store(false, std::memory_order_release);
  status_cv_.notify_all();
}

void NativeTianjiController::cleanupVendorSession(bool emergency) noexcept {
  if (!vendor_session_created_) {
    return;
  }

  // Cleanup remains owned by the SDK I/O worker, including exception paths.
  // OnEMG_AB already disables both arms; a second Disable transaction during
  // fault cleanup is redundant and known to time out on some firmware.
  // Keep each vendor operation in its own guarded block.  In particular, a
  // firmware timeout from Disable() must not skip OnRelease(): leaving the
  // timer/shared-memory handle alive was one of the causes of the next launch
  // seeing an apparently random "controller not running" state.  Cleanup is
  // best effort and never replaces the original worker fault.
  if (vendor_connected_) {
    try {
      if (emergency) {
        (void)OnEMG_AB();
      } else {
        try {
          (void)Disable('A');
        } catch (...) {
          // Continue to the second arm and OnRelease below.
        }
        try {
          (void)Disable('B');
        } catch (...) {
          // Continue to OnRelease even when one arm's firmware timed out.
        }
      }
    } catch (...) {
      // Continue to OnRelease even if one arm's disable transaction fails.
    }
  }
  try {
    (void)OnRelease();
  } catch (...) {
    // Best-effort noexcept cleanup: preserve the original worker fault in
    // TianjiStatus rather than terminating the Python process here.
  }
  vendor_session_created_ = false;
  vendor_connected_ = false;
}

} // namespace libpytianji
