#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <mutex>

namespace libpytianji {

// The native planner uses the same 4x4 row-major pose convention as the
// Python adapter.  These aliases intentionally stay independent of the
// vendor SDK types: no SDK object is ever passed to the Pinocchio solver.
using PinkPose = std::array<double, 16>;
using PinkArmVector = std::array<double, 7>;

struct PinkIKDiagnostics {
  bool success{false};
  int iterations{0};
  double position_error{0.0};
  double orientation_error{0.0};
};

// Streaming defaults intentionally match libtron2rt's mapping of Pink
// options into tron2-ik's native solver: a warm-started 200/300 Hz Cartesian
// stream needs bounded latency rather than offline, sub-micron convergence.
struct PinkIKOptions {
  int max_iterations{80};
  double dt{0.1};
  double damping{1.0e-4};
  double step_size{1.0};
  double max_step_norm{0.15};
  double position_tolerance{1.0e-4};
  double orientation_tolerance{1.0e-3};
  double position_cost{1.0};
  double orientation_cost{1.0};
  double frame_gain{1.0};
  // Pink/tron2-ik leave the posture task disabled by default.  The previous
  // 1e-3 regularizer biased small Cartesian updates enough to miss the
  // 1e-4 m streaming tolerance; continuity comes from the warm-start seed and
  // the native safety gate unless a caller explicitly enables posture bias.
  double posture_weight{0.0};
  double posture_gain{1.0};
};

// A small, allocation-free-at-solve-time Pinocchio implementation of Pink's
// task semantics.  This is an independent Tianji implementation: tron2-ik is
// used as a design/reference implementation only; no tron2-ik object or
// source file is linked into this package.  The Python Pink package uses
// qpsolvers for an interactive/non-real-time path.  libpytianji cannot invoke
// Python from its real-time planner worker, so it embeds the same warm-started Cartesian
// quadratic objective and constrained numerical solve natively.
class PinkIKModel {
 public:
  explicit PinkIKModel(PinkIKOptions options = {});
  ~PinkIKModel();

  PinkIKModel(const PinkIKModel &) = delete;
  PinkIKModel &operator=(const PinkIKModel &) = delete;

  PinkPose forwardKinematics(std::size_t arm,
                            const PinkArmVector &q) const;

  // Solve one arm for one planner waypoint.  ``max_step_rad`` is the
  // per-joint velocity envelope for one integrated Pink step.  The solver
  // uses it as a box constraint on the numerical velocity; the controller
  // applies the same envelope once more to the final candidate before it is
  // published, matching the two-stage bound used by libtron2rt.
  bool solve(std::size_t arm, const PinkPose &target,
             const PinkArmVector &reference,
             const PinkArmVector &max_step_rad, PinkArmVector &solution,
             PinkIKDiagnostics &diagnostics) const;

  static const char *backendName() noexcept { return "pink-pinocchio-native"; }

 private:
  struct ArmModel;

  static std::unique_ptr<ArmModel> buildArmModel(std::size_t arm);
  static void validateOptions(const PinkIKOptions &options);
  static bool validPose(const PinkPose &pose) noexcept;
  static bool finiteVector(const PinkArmVector &values) noexcept;

  PinkPose forwardKinematicsUnlocked(std::size_t arm,
                                     const PinkArmVector &q) const;
  bool solveUnlocked(std::size_t arm, const PinkPose &target,
                     const PinkArmVector &reference,
                     const PinkArmVector &max_step_rad,
                     PinkArmVector &solution,
                     PinkIKDiagnostics &diagnostics) const;

  mutable std::array<std::unique_ptr<ArmModel>, 2> arms_;
  PinkIKOptions options_{};
  mutable std::mutex mutex_;
};

}  // namespace libpytianji
