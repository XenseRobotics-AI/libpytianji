#include "pink_ik.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

// Pinocchio's joint collection has 25 alternatives.  Its compatibility
// header must be included before any Boost.MPL header (multibody.hpp includes
// boost::variant internally); otherwise Boost selects its 20-type generated
// list and Pinocchio 4 fails during template instantiation.
#include <pinocchio/deprecated/pinocchio/container/boost-container-limits.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/spatial.hpp>

namespace libpytianji {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kHalfPi = kPi / 2.0;
using SE3 = pinocchio::SE3;
using Model = pinocchio::Model;
using Data = pinocchio::Data;
using Vector7 = Eigen::Matrix<double, 7, 1>;
using Matrix67 = Eigen::Matrix<double, 6, 7>;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;
using Matrix7 = Eigen::Matrix<double, 7, 7>;

// Pink hands a small strictly-convex velocity QP to qpsolvers.  The RT
// package cannot call a Python backend from its planner thread, so solve the
// same box-constrained quadratic locally.  There are only seven variables per
// Tianji arm; this deterministic active-set implementation is allocation-free
// and uses Eigen's LDLT for each free-set solve.  The objective is
//   1/2 vᵀ H v - rhsᵀ v,  lower <= v <= upper.
bool solveBoxQP(const Matrix7 &hessian, const Vector7 &rhs,
                const Vector7 &lower, const Vector7 &upper,
                Vector7 &solution) noexcept {
  constexpr double kTolerance = 1.0e-10;
  constexpr int kMaxActiveSetIterations = 64;
  std::array<int, 7> active{};  // -1: lower, 0: free, +1: upper
  Vector7 state = Vector7::Zero();

  if (!hessian.allFinite() || !rhs.allFinite() || !lower.allFinite() ||
      !upper.allFinite()) {
    return false;
  }
  for (int index = 0; index < 7; ++index) {
    if (!(lower[index] < upper[index])) {
      return false;
    }
  }

  for (int iteration = 0; iteration < kMaxActiveSetIterations; ++iteration) {
    // Eliminate fixed variables from the free-variable system while retaining
    // a full 7x7 matrix.  This avoids dynamic Eigen allocations in the RT
    // loop and keeps the active values exact at their bounds.
    Matrix7 system = hessian;
    Vector7 vector = rhs;
    for (int index = 0; index < 7; ++index) {
      if (active[static_cast<std::size_t>(index)] == 0) {
        continue;
      }
      const double value = active[static_cast<std::size_t>(index)] < 0
                               ? lower[index]
                               : upper[index];
      state[index] = value;
      for (int row = 0; row < 7; ++row) {
        if (active[static_cast<std::size_t>(row)] == 0) {
          vector[row] -= hessian(row, index) * value;
        }
        if (row != index) {
          system(row, index) = 0.0;
          system(index, row) = 0.0;
        }
      }
      system(index, index) = 1.0;
      vector[index] = value;
    }

    Eigen::LDLT<Matrix7> factor(system);
    if (factor.info() != Eigen::Success) {
      return false;
    }
    state = factor.solve(vector);
    if (!state.allFinite()) {
      return false;
    }

    // Add the most-violated free variable to the active set.
    double largest_violation = kTolerance;
    int violated_index = -1;
    int violated_side = 0;
    for (int index = 0; index < 7; ++index) {
      if (active[static_cast<std::size_t>(index)] != 0) {
        continue;
      }
      if (state[index] < lower[index] - kTolerance) {
        const double violation = lower[index] - state[index];
        if (violation > largest_violation) {
          largest_violation = violation;
          violated_index = index;
          violated_side = -1;
        }
      } else if (state[index] > upper[index] + kTolerance) {
        const double violation = state[index] - upper[index];
        if (violation > largest_violation) {
          largest_violation = violation;
          violated_index = index;
          violated_side = 1;
        }
      }
    }
    if (violated_index >= 0) {
      active[static_cast<std::size_t>(violated_index)] = violated_side;
      continue;
    }

    // Check KKT multipliers for variables currently fixed at a bound.  At a
    // lower bound the gradient must be non-negative; at an upper bound it
    // must be non-positive.  Release the most-violated bound and resolve.
    const Vector7 gradient = hessian * state - rhs;
    double largest_multiplier_violation = kTolerance;
    int release_index = -1;
    for (int index = 0; index < 7; ++index) {
      const int side = active[static_cast<std::size_t>(index)];
      const double violation =
          side < 0 ? -gradient[index] : side > 0 ? gradient[index] : 0.0;
      if (violation > largest_multiplier_violation) {
        largest_multiplier_violation = violation;
        release_index = index;
      }
    }
    if (release_index >= 0) {
      active[static_cast<std::size_t>(release_index)] = 0;
      continue;
    }

    solution = state;
    return true;
  }

  // A positive-definite Hessian should converge well within the bound above.
  // If round-off causes an active-set tie, return a projected finite solution
  // rather than allowing a transient QP bookkeeping issue to stop the arm.
  const Eigen::LDLT<Matrix7> factor(hessian);
  if (factor.info() != Eigen::Success) {
    return false;
  }
  solution = factor.solve(rhs);
  if (!solution.allFinite()) {
    return false;
  }
  for (int index = 0; index < 7; ++index) {
    solution[index] = std::clamp(solution[index], lower[index], upper[index]);
  }
  return true;
}

bool finite(double value) noexcept { return std::isfinite(value); }

Eigen::Matrix3d rpy(double roll, double pitch, double yaw) {
  // URDF fixed-axis RPY convention: Rz(yaw) * Ry(pitch) * Rx(roll).
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

SE3 pose(double x, double y, double z, double roll, double pitch,
         double yaw) {
  return SE3(rpy(roll, pitch, yaw), Eigen::Vector3d(x, y, z));
}

SE3 poseFromArray(const PinkPose &values) {
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      matrix(row, column) = values[static_cast<std::size_t>(row * 4 + column)];
    }
  }
  return SE3(matrix.block<3, 3>(0, 0), matrix.block<3, 1>(0, 3));
}

PinkPose poseToArray(const SE3 &value) {
  PinkPose result{};
  const Eigen::Matrix3d rotation = value.rotation();
  const Eigen::Vector3d translation = value.translation();
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      result[static_cast<std::size_t>(row * 4 + column)] =
          rotation(row, column);
    }
    result[static_cast<std::size_t>(row * 4 + 3)] = translation[row];
  }
  result[12] = 0.0;
  result[13] = 0.0;
  result[14] = 0.0;
  result[15] = 1.0;
  return result;
}

Vector7 toEigen(const PinkArmVector &values) {
  Vector7 result;
  for (int index = 0; index < 7; ++index) {
    result[index] = values[static_cast<std::size_t>(index)];
  }
  return result;
}

PinkArmVector fromEigen(const Vector7 &values) {
  PinkArmVector result{};
  for (int index = 0; index < 7; ++index) {
    result[static_cast<std::size_t>(index)] = values[index];
  }
  return result;
}

Eigen::Vector3d rotationError(const Eigen::Matrix3d &current,
                              const Eigen::Matrix3d &target) {
  // log3 is expressed in the current frame.  LOCAL_WORLD_ALIGNED Jacobians
  // use world axes, so rotate the error into that same frame.
  return current * pinocchio::log3(current.transpose() * target);
}

void update(Model &model, Data &data, const Vector7 &q) {
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);
}

}  // namespace

struct PinkIKModel::ArmModel {
  Model model;
  Data data;
  pinocchio::FrameIndex tcp_frame{0};
  std::array<int, 7> q_index{};
  PinkArmVector lower{};
  PinkArmVector upper{};

  ArmModel() : data(model) {}
};

std::unique_ptr<PinkIKModel::ArmModel> PinkIKModel::buildArmModel(
    std::size_t arm) {
  if (arm >= 2) {
    throw std::invalid_argument("Pink IK arm index must be 0 or 1");
  }

  auto result = std::make_unique<ArmModel>();
  auto &model = result->model;

  // The Marvin URDF geometry was independently transcribed into this model.
  // Its TCP frame is expressed in the robot/world frame, while Tianji's
  // Cartesian API uses an arm-local SDK frame.  This fixed transform was
  // obtained by comparing the independent model against the SDK FK at several
  // non-zero configurations.  It is constant, so baking it into the first
  // joint placement preserves the SDK pose convention without a runtime SDK
  // FK call.
  const bool left = arm == 0;
  const SE3 sdk_from_urdf =
      left ? pose(0.0, 1.121, -0.037, kHalfPi, 0.0, 0.0)
           : pose(0.0, -1.121, -0.037, -kHalfPi, 0.0, 0.0);
  const SE3 stand = pose(0.0, 0.0, 0.981, 0.0, 0.0, 0.0);
  const SE3 arm_base = left ? pose(0.0, 0.037, 0.140, -1.5708, 0.0, 0.0)
                            : pose(0.0, -0.037, 0.140, 1.5708, 0.0, 0.0);

  constexpr std::array<double, 7> lower{{-3.1067, -2.0944, -3.1067,
                                          -2.5307, -3.1067, -1.0472,
                                          -1.5708}};
  constexpr std::array<double, 7> upper{{3.1067, 2.0944, 3.1067, 1.0472,
                                          3.1067, 1.0472, 1.5708}};
  const std::array<SE3, 7> joint_origins{{
      pose(0.0, 0.0, 0.1745, 0.0, 0.0, 0.0),
      pose(0.0, 0.0, 0.0, 1.5708, 0.0, 0.0),
      pose(0.0, 0.287, 0.0, -1.5708, 0.0, 0.0),
      pose(0.018, 0.0, 0.0, -1.5708, 0.0, 3.1416),
      pose(0.018, -0.314, 0.0, -1.5708, 0.0, 3.1416),
      pose(0.0, 0.0, 0.0, 1.5708, -1.5708, 0.0),
      pose(0.0, 0.0, 0.0, 1.5708, -1.5708, 0.0),
  }};
  const SE3 tcp_origin = pose(0.0, -0.095, 0.0, 1.5708, -1.5708, 0.0);

  pinocchio::JointIndex parent = 0;
  for (std::size_t index = 0; index < 7; ++index) {
    SE3 placement = joint_origins[index];
    if (index == 0) {
      placement = sdk_from_urdf * stand * arm_base * placement;
    }
    const std::string name = "Joint" + std::to_string(index + 1) +
                             (left ? "_L" : "_R");
    const auto joint = model.addJoint(parent, pinocchio::JointModelRZ(),
                                      placement, name);
    model.addJointFrame(joint);
    const int q_index = model.joints[joint].idx_q();
    result->q_index[index] = q_index;
    model.lowerPositionLimit[q_index] = lower[index];
    model.upperPositionLimit[q_index] = upper[index];
    result->lower[index] = lower[index];
    result->upper[index] = upper[index];
    parent = joint;
  }
  result->tcp_frame = model.addFrame(
      pinocchio::Frame("TCP_Link_" + std::string(left ? "L" : "R"), parent,
                       tcp_origin, pinocchio::OP_FRAME));
  result->data = Data(model);
  return result;
}

void PinkIKModel::validateOptions(const PinkIKOptions &options) {
  if (options.max_iterations < 1) {
    throw std::invalid_argument("Pink IK max_iterations must be positive");
  }
  const std::array<double, 11> nonnegative{{
      options.dt,
      options.damping,
      options.step_size,
      options.max_step_norm,
      options.position_tolerance,
      options.orientation_tolerance,
      options.position_cost,
      options.orientation_cost,
      options.frame_gain,
      options.posture_weight,
      options.posture_gain,
  }};
  for (double value : nonnegative) {
    if (!finite(value) || value < 0.0) {
      throw std::invalid_argument(
          "Pink IK options must be finite and non-negative");
    }
  }
  if (options.dt <= 0.0 || options.damping <= 0.0 ||
      options.step_size <= 0.0 || options.max_step_norm <= 0.0 ||
      options.position_tolerance <= 0.0 ||
      options.orientation_tolerance <= 0.0 ||
      options.position_cost <= 0.0 || options.orientation_cost <= 0.0 ||
      options.frame_gain <= 0.0 || options.frame_gain > 1.0 ||
      options.posture_gain > 1.0) {
    throw std::invalid_argument("Pink IK options contain an invalid range");
  }
}

PinkIKModel::PinkIKModel(PinkIKOptions options) : options_(options) {
  validateOptions(options_);
  arms_[0] = buildArmModel(0);
  arms_[1] = buildArmModel(1);
}

PinkIKModel::~PinkIKModel() = default;

bool PinkIKModel::finiteVector(const PinkArmVector &values) noexcept {
  for (double value : values) {
    if (!finite(value)) {
      return false;
    }
  }
  return true;
}

bool PinkIKModel::validPose(const PinkPose &values) noexcept {
  if (!std::all_of(values.begin(), values.end(), finite)) {
    return false;
  }
  if (std::abs(values[12]) > 1.0e-6 || std::abs(values[13]) > 1.0e-6 ||
      std::abs(values[14]) > 1.0e-6 || std::abs(values[15] - 1.0) > 1.0e-6) {
    return false;
  }
  Eigen::Matrix3d rotation;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      rotation(row, column) = values[static_cast<std::size_t>(row * 4 + column)];
    }
  }
  const double determinant = rotation.determinant();
  return finite(determinant) && std::abs(determinant - 1.0) < 2.0e-3 &&
         (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() <
             2.0e-3;
}

PinkPose PinkIKModel::forwardKinematicsUnlocked(
    std::size_t arm, const PinkArmVector &q) const {
  if (arm >= arms_.size() || !arms_[arm] || !finiteVector(q)) {
    throw std::invalid_argument("invalid arm or joint vector for Pink FK");
  }
  auto &model = arms_[arm]->model;
  auto &data = arms_[arm]->data;
  Vector7 values = toEigen(q);
  for (int index = 0; index < 7; ++index) {
    values[index] = std::clamp(values[index],
                                arms_[arm]->lower[static_cast<std::size_t>(index)],
                                arms_[arm]->upper[static_cast<std::size_t>(index)]);
  }
  update(model, data, values);
  return poseToArray(data.oMf[arms_[arm]->tcp_frame]);
}

PinkPose PinkIKModel::forwardKinematics(std::size_t arm,
                                        const PinkArmVector &q) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return forwardKinematicsUnlocked(arm, q);
}

bool PinkIKModel::solveUnlocked(std::size_t arm, const PinkPose &target_values,
                                const PinkArmVector &reference,
                                const PinkArmVector &max_step_rad,
                                PinkArmVector &solution,
                                PinkIKDiagnostics &diagnostics) const {
  diagnostics = PinkIKDiagnostics{};
  if (arm >= arms_.size() || !arms_[arm] || !validPose(target_values) ||
      !finiteVector(reference) || !finiteVector(max_step_rad)) {
    return false;
  }
  for (double value : max_step_rad) {
    if (value <= 0.0) {
      return false;
    }
  }

  auto &arm_model = *arms_[arm];
  auto &model = arm_model.model;
  auto &data = arm_model.data;
  const SE3 target = poseFromArray(target_values);
  Vector7 q = toEigen(reference);
  Vector7 seed = q;
  // Match Pink's native migration semantics: Pinocchio joint limits constrain
  // the numerical iteration and the velocity box bounds each integrated task
  // step.  The caller still checks/clips the final candidate relative to its
  // last published command, because one solve intentionally runs several
  // numerical iterations and can therefore span multiple RT cycles.
  Vector7 trust_lower;
  Vector7 trust_upper;
  for (int index = 0; index < 7; ++index) {
    const auto i = static_cast<std::size_t>(index);
    q[index] = std::clamp(q[index], arm_model.lower[i], arm_model.upper[i]);
    seed[index] = q[index];
    trust_lower[index] = arm_model.lower[i];
    trust_upper[index] = arm_model.upper[i];
  }
  // Match Pink's failure contract: even when the requested pose cannot be
  // reached, return the best finite warm-start state reached so far.  The
  // native controller will hold it (and report -1) rather than consuming an
  // uninitialized/zero candidate.
  solution = fromEigen(q);

  const auto evaluate = [&](const Vector7 &state, Vector6 *error,
                            double &position_error,
                            double &orientation_error) {
    update(model, data, state);
    const SE3 current = data.oMf[arm_model.tcp_frame];
    const Eigen::Vector3d position = target.translation() - current.translation();
    const Eigen::Vector3d orientation =
        rotationError(current.rotation(), target.rotation());
    position_error = position.norm();
    orientation_error = orientation.norm();
    if (error != nullptr) {
      error->head<3>() = position;
      error->tail<3>() = orientation;
    }
  };

  double position_error = 0.0;
  double orientation_error = 0.0;
  Vector6 error;
  evaluate(q, &error, position_error, orientation_error);
  auto converged = [&]() {
    return position_error <= options_.position_tolerance &&
           orientation_error <= options_.orientation_tolerance;
  };
  if (converged()) {
    solution = fromEigen(q);
    diagnostics.success = true;
    diagnostics.position_error = position_error;
    diagnostics.orientation_error = orientation_error;
    return true;
  }

  Matrix6 weight = Matrix6::Zero();
  weight.block<3, 3>(0, 0).diagonal().setConstant(options_.position_cost);
  weight.block<3, 3>(3, 3).diagonal().setConstant(options_.orientation_cost);

  for (int iteration = 1; iteration <= options_.max_iterations; ++iteration) {
    // ``computeJointJacobians`` was called by evaluate above; refresh it after
    // every accepted step as Pink does for a warm-started configuration.
    Matrix67 jacobian;
    jacobian.setZero();
    pinocchio::getFrameJacobian(model, data, arm_model.tcp_frame,
                                pinocchio::LOCAL_WORLD_ALIGNED, jacobian);

    Matrix7 hessian = jacobian.transpose() * weight * jacobian;
    // Pink's global ``damping`` parameter is a positive diagonal term of the
    // QP Hessian.  Keep it linear here rather than squaring it as a physical
    // viscosity coefficient; otherwise the streaming default 1e-4 would be
    // reduced to 1e-8 and provide almost no singularity conditioning.
    hessian.diagonal().array() += options_.damping;
    // Pink's FrameTask target is a desired task *velocity*: the pose error is
    // divided by dt before the QP is assembled.  Keeping this scaling here is
    // important when the streaming dt differs from the host planner period;
    // the subsequent manifold integration multiplies the solved velocity by
    // the same dt and therefore preserves Pink's per-step behavior.
    const Vector6 desired_task_velocity =
        options_.frame_gain * error / options_.dt;
    Vector7 rhs = jacobian.transpose() * weight * desired_task_velocity;
    if (options_.posture_weight > 0.0) {
      hessian.diagonal().array() += options_.posture_weight;
      rhs += options_.posture_weight * options_.posture_gain *
             (seed - q) / options_.dt;
    }
    Vector7 velocity;
    Vector7 velocity_lower;
    Vector7 velocity_upper;
    for (int index = 0; index < 7; ++index) {
      const auto i = static_cast<std::size_t>(index);
      const double limit = max_step_rad[i] / options_.dt;
      // Pink's configuration-limit task is expressed in velocity space for
      // each integrated step.  Intersect that position-derived interval with
      // the caller's per-cycle trust region before solving the QP; the later
      // clamp is then only a round-off guard, not the primary limit mechanism.
      velocity_lower[index] =
          std::max(-limit, (trust_lower[index] - q[index]) / options_.dt);
      velocity_upper[index] =
          std::min(limit, (trust_upper[index] - q[index]) / options_.dt);
      if (velocity_lower[index] > velocity_upper[index]) {
        diagnostics.iterations = iteration;
        diagnostics.position_error = position_error;
        diagnostics.orientation_error = orientation_error;
        return false;
      }
    }
    if (!solveBoxQP(hessian, rhs, velocity_lower, velocity_upper, velocity)) {
      solution = fromEigen(q);
      diagnostics.iterations = iteration;
      diagnostics.position_error = position_error;
      diagnostics.orientation_error = orientation_error;
      return false;
    }

    // Keep each Newton iteration bounded.  Joint limits are enforced after
    // integrating on Pinocchio's configuration manifold, just as Pink does;
    // the outer controller separately limits the final candidate relative to
    // its last published command.
    Vector7 step = options_.step_size * options_.dt * velocity;
    const double norm = step.norm();
    const double max_norm = options_.max_step_norm;
    if (norm > max_norm) {
      step *= max_norm / norm;
    }
    q = pinocchio::integrate(model, q, step);
    for (int index = 0; index < 7; ++index) {
      q[index] = std::clamp(q[index], trust_lower[index], trust_upper[index]);
    }

    evaluate(q, &error, position_error, orientation_error);
    diagnostics.iterations = iteration;
    diagnostics.position_error = position_error;
    diagnostics.orientation_error = orientation_error;
    if (converged()) {
      diagnostics.success = true;
      solution = fromEigen(q);
      return true;
    }
  }

  solution = fromEigen(q);
  return false;
}

bool PinkIKModel::solve(std::size_t arm, const PinkPose &target,
                        const PinkArmVector &reference,
                        const PinkArmVector &max_step_rad,
                        PinkArmVector &solution,
                        PinkIKDiagnostics &diagnostics) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return solveUnlocked(arm, target, reference, max_step_rad, solution,
                       diagnostics);
}

}  // namespace libpytianji
