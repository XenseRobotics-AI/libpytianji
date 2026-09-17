#include "native_controller.hpp"
#include "pink_ik.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <array>
#include <stdexcept>
#include <string>

namespace py = pybind11;
using namespace libpytianji;

namespace {

template <typename T, std::size_t N>
std::array<T, N> toArray(const py::handle &value, const char *name) {
  if (value.is_none()) {
    throw std::invalid_argument(std::string(name) + " cannot be None");
  }
  const auto sequence = py::reinterpret_borrow<py::sequence>(value);
  if (sequence.size() != static_cast<py::ssize_t>(N)) {
    throw std::invalid_argument(std::string(name) + " must contain " +
                                std::to_string(N) + " values");
  }
  std::array<T, N> result{};
  for (std::size_t index = 0; index < N; ++index) {
    result[index] = py::cast<T>(sequence[index]);
  }
  return result;
}

Pose toPose(const py::handle &value, const char *name) {
  return toArray<double, kPoseElements>(value, name);
}

ArmVector toArm(const py::handle &value, const char *name) {
  return toArray<double, kArmDof>(value, name);
}

std::array<double, 3> toVec3(const py::handle &value, const char *name) {
  return toArray<double, 3>(value, name);
}

std::array<bool, 2> toMask(const py::handle &value) {
  if (value.is_none()) {
    return {true, true};
  }
  return toArray<bool, 2>(value, "active_mask");
}

py::dict stateToDict(const TianjiState &state) {
  py::dict result;
  result["q"] = state.q;
  result["dq"] = state.dq;
  result["last_command_q"] = state.last_command_q;
  result["tcp_left"] = state.tcp_left;
  result["tcp_right"] = state.tcp_right;
  result["timestamp_ns"] = state.timestamp_ns;
  result["fault_code"] = state.fault_code;
  result["valid"] = state.valid;
  return result;
}

py::dict statusToDict(const TianjiStatus &status) {
  py::dict result;
  result["mode"] = NativeTianjiController::modeName(status.mode);
  result["target_sequence"] = status.target_sequence;
  result["consumed_sequence"] = status.consumed_sequence;
  result["trajectory_sequence"] = status.trajectory_sequence;
  result["trajectory_done_sequence"] = status.trajectory_done_sequence;
  result["ik_success_count"] = status.ik_success_count;
  result["ik_failure_count"] = status.ik_failure_count;
  result["command_timeout_count"] = status.command_timeout_count;
  result["scheduler_warning_count"] = status.scheduler_warning_count;
  result["target_age_ns"] = status.target_age_ns;
  result["last_ik_status"] = status.last_ik_status;
  result["fault_code"] = status.fault_code;
  result["timestamp_ns"] = status.timestamp_ns;
  result["planner_hz"] = status.planner_hz;
  result["io_hz"] = status.io_hz;
  result["command_period_ms"] = status.command_period_ms;
  result["running"] = status.running;
  result["trajectory_active"] = status.trajectory_active;
  result["scheduler_degraded"] = status.scheduler_degraded;
  result["command_timed_out"] = status.command_timed_out;
  result["feedback_ready"] = status.feedback_ready;
  result["command_ready"] = status.command_ready;
  result["fault_message"] = std::string(status.fault_message.data());
  return result;
}

} // namespace

PYBIND11_MODULE(_native, module) {
  module.doc() =
      "Native Tianji planner/I/O backend. Vendor units stay inside C++.";
  // Increment when the Python adapter's safety/hand-off contract changes.
  // Refusing an older wheel is preferable to silently mixing a new adapter
  // with a native binary that can still accept stale enable-edge targets.
  // Version 6 replaces the vendor analytical IK/FK calls in the planner with
  // the native Pink/Pinocchio backend.  Refuse an older wheel rather than
  // silently running the old branch-selecting solver on a new adapter.
  // Version 7 makes tool_kine the shared SDK and Pink TCP definition.
  module.attr("API_VERSION") = 7;
  module.attr("IK_BACKEND") = PinkIKModel::backendName();

  // Hardware-free diagnostics for the exact native model used by the
  // planner.  Keeping a hardware-free FK/IK validation surface makes it
  // possible to verify a wheel after installation without opening a Tianji
  // socket.
  py::class_<PinkIKModel>(module, "PinkIKModel")
      .def(py::init<>())
      .def(py::init(
               [](const py::handle &left_tool_kine,
                  const py::handle &right_tool_kine) {
                 return new PinkIKModel(
                     toArray<double, 6>(left_tool_kine, "left_tool_kine"),
                     toArray<double, 6>(right_tool_kine, "right_tool_kine"));
               }),
           py::arg("left_tool_kine"), py::arg("right_tool_kine"))
      .def_static("backend_name", &PinkIKModel::backendName)
      .def("forward_kinematics",
           [](const PinkIKModel &model, std::size_t arm,
              const py::handle &q) {
             return model.forwardKinematics(arm, toArm(q, "q"));
           },
           py::arg("arm"), py::arg("q"))
      .def("solve",
           [](const PinkIKModel &model, std::size_t arm,
              const py::handle &target, const py::handle &reference,
              const py::handle &max_step_rad) {
             PinkArmVector solution{};
             PinkIKDiagnostics diagnostics{};
             const bool ok = model.solve(
                 arm, toPose(target, "target"), toArm(reference, "reference"),
                 toArm(max_step_rad, "max_step_rad"), solution, diagnostics);
             py::dict result;
             result["q"] = solution;
             result["success"] = ok;
             result["iterations"] = diagnostics.iterations;
             result["position_error"] = diagnostics.position_error;
             result["orientation_error"] = diagnostics.orientation_error;
             return result;
           },
           py::arg("arm"), py::arg("target"), py::arg("reference"),
           py::arg("max_step_rad"));

  py::enum_<ControllerMode>(module, "ControllerMode")
      .value("STOPPED", ControllerMode::Stopped)
      .value("STARTING", ControllerMode::Starting)
      .value("STREAMING", ControllerMode::Streaming)
      .value("HOME", ControllerMode::Home)
      .value("HOLD", ControllerMode::Hold)
      .value("FAULT", ControllerMode::Fault)
      .export_values();

  py::class_<TianjiState>(module, "TianjiState")
      .def_readonly("q", &TianjiState::q)
      .def_readonly("dq", &TianjiState::dq)
      .def_readonly("last_command_q", &TianjiState::last_command_q)
      .def_readonly("tcp_left", &TianjiState::tcp_left)
      .def_readonly("tcp_right", &TianjiState::tcp_right)
      .def_readonly("timestamp_ns", &TianjiState::timestamp_ns)
      .def_readonly("fault_code", &TianjiState::fault_code)
      .def_readonly("valid", &TianjiState::valid)
      .def("as_dict", &stateToDict);

  py::class_<TianjiStatus>(module, "TianjiStatus")
      .def_readonly("mode", &TianjiStatus::mode)
      .def_readonly("target_sequence", &TianjiStatus::target_sequence)
      .def_readonly("consumed_sequence", &TianjiStatus::consumed_sequence)
      .def_readonly("trajectory_sequence", &TianjiStatus::trajectory_sequence)
      .def_readonly("trajectory_done_sequence", &TianjiStatus::trajectory_done_sequence)
      .def_readonly("ik_success_count", &TianjiStatus::ik_success_count)
      .def_readonly("ik_failure_count", &TianjiStatus::ik_failure_count)
      .def_readonly("command_timeout_count", &TianjiStatus::command_timeout_count)
      .def_readonly("scheduler_warning_count", &TianjiStatus::scheduler_warning_count)
      .def_readonly("target_age_ns", &TianjiStatus::target_age_ns)
      .def_readonly("last_ik_status", &TianjiStatus::last_ik_status)
      .def_readonly("fault_code", &TianjiStatus::fault_code)
      .def_readonly("timestamp_ns", &TianjiStatus::timestamp_ns)
      .def_readonly("planner_hz", &TianjiStatus::planner_hz)
      .def_readonly("io_hz", &TianjiStatus::io_hz)
      .def_readonly("command_period_ms", &TianjiStatus::command_period_ms)
      .def_readonly("running", &TianjiStatus::running)
      .def_readonly("trajectory_active", &TianjiStatus::trajectory_active)
      .def_readonly("scheduler_degraded", &TianjiStatus::scheduler_degraded)
      .def_readonly("command_timed_out", &TianjiStatus::command_timed_out)
      .def_readonly("feedback_ready", &TianjiStatus::feedback_ready)
      .def_readonly("command_ready", &TianjiStatus::command_ready)
      .def_property_readonly("fault_message", [](const TianjiStatus &status) {
        return std::string(status.fault_message.data());
      })
      .def("as_dict", &statusToDict);

  py::class_<NativeTianjiController>(module, "NativeTianjiController")
      .def(py::init(
               [](const std::string &robot_ip,
                  const std::string &kinematics_config_path, int planner_hz,
                  int io_hz, double command_timeout_s,
                  double max_tcp_linear_velocity_m_s,
                  double max_tcp_angular_velocity_rad_s, int velocity_ratio,
                  int acceleration_ratio, const py::handle &left_joint_min_rad,
                  const py::handle &left_joint_max_rad,
                  const py::handle &right_joint_min_rad,
                  const py::handle &right_joint_max_rad,
                  bool use_application_joint_limits, int realtime_priority,
                  int cpu_affinity, bool simulate,
                  double max_joint_velocity_deg_s,
                  double ik_safety_velocity_scale, bool enable_workspace_check,
                  const py::handle &workspace_min_m,
                  const py::handle &workspace_max_m,
                  bool enable_inter_arm_distance_check,
                  double min_tcp_distance_m,
                  const py::handle &pd_joint_stiffness,
                  const py::handle &pd_joint_damping,
                  const py::handle &left_tool_kine,
                  const py::handle &left_tool_dyn,
                  const py::handle &right_tool_kine,
                  const py::handle &right_tool_dyn) {
                 return new NativeTianjiController(
                     robot_ip, kinematics_config_path, planner_hz, io_hz,
                     command_timeout_s, max_tcp_linear_velocity_m_s,
                     max_tcp_angular_velocity_rad_s, velocity_ratio,
                     acceleration_ratio, toArm(left_joint_min_rad, "left_joint_min_rad"),
                     toArm(left_joint_max_rad, "left_joint_max_rad"),
                     toArm(right_joint_min_rad, "right_joint_min_rad"),
                     toArm(right_joint_max_rad, "right_joint_max_rad"),
                     use_application_joint_limits, realtime_priority, cpu_affinity,
                     simulate, max_joint_velocity_deg_s, ik_safety_velocity_scale,
                     enable_workspace_check,
                     toVec3(workspace_min_m, "workspace_min_m"),
                     toVec3(workspace_max_m, "workspace_max_m"),
                     enable_inter_arm_distance_check, min_tcp_distance_m,
                     toArm(pd_joint_stiffness, "pd_joint_stiffness"),
                     toArm(pd_joint_damping, "pd_joint_damping"),
                     toArray<double, 6>(left_tool_kine, "left_tool_kine"),
                     toArray<double, 10>(left_tool_dyn, "left_tool_dyn"),
                     toArray<double, 6>(right_tool_kine, "right_tool_kine"),
                     toArray<double, 10>(right_tool_dyn, "right_tool_dyn"));
               }),
           py::arg("robot_ip"), py::arg("kinematics_config_path"),
           py::arg("planner_hz") = 300, py::arg("io_hz") = 1000,
           py::arg("command_timeout_s") = 0.5,
           py::arg("max_tcp_linear_velocity_m_s") = 0.5,
           py::arg("max_tcp_angular_velocity_rad_s") = 1.0,
           py::arg("velocity_ratio") = 100, py::arg("acceleration_ratio") = 100,
           py::arg("left_joint_min_rad") = NativeTianjiController::DefaultJointLowerLimit(),
           py::arg("left_joint_max_rad") = NativeTianjiController::DefaultJointUpperLimit(),
           py::arg("right_joint_min_rad") = NativeTianjiController::DefaultJointLowerLimit(),
           py::arg("right_joint_max_rad") = NativeTianjiController::DefaultJointUpperLimit(),
           py::arg("use_application_joint_limits") = false,
           py::arg("realtime_priority") = 0, py::arg("cpu_affinity") = -1,
           py::arg("simulate") = false, py::arg("max_joint_velocity_deg_s") = 180.0,
           py::arg("ik_safety_velocity_scale") = 1.0,
           py::arg("enable_workspace_check") = false,
           py::arg("workspace_min_m") = std::array<double, 3>{-1.2, -1.2, 0.0},
           py::arg("workspace_max_m") = std::array<double, 3>{1.2, 1.2, 1.5},
           py::arg("enable_inter_arm_distance_check") = false,
           py::arg("min_tcp_distance_m") = 0.18,
           py::arg("pd_joint_stiffness") = NativeTianjiController::DefaultJointStiffness(),
           py::arg("pd_joint_damping") = NativeTianjiController::DefaultJointDamping(),
           py::arg("left_tool_kine") = std::array<double, 6>{},
           py::arg("left_tool_dyn") = std::array<double, 10>{},
           py::arg("right_tool_kine") = std::array<double, 6>{},
           py::arg("right_tool_dyn") = std::array<double, 10>{})
      .def("start", &NativeTianjiController::start,
           "Start native planner and SDK I/O threads.",
           py::call_guard<py::gil_scoped_release>())
      .def("submit_cartesian_targets",
           [](NativeTianjiController &controller, const py::handle &left,
              const py::handle &right, const py::handle &active) {
             const auto left_pose = toPose(left, "left_pose");
             const auto right_pose = toPose(right, "right_pose");
             const auto mask = toMask(active);
             py::gil_scoped_release release;
             return controller.submitCartesianTargets(left_pose, right_pose, mask);
           },
           py::arg("left_pose"), py::arg("right_pose"),
           py::arg("active_mask") = py::none())
      .def("request_joint_trajectory",
           [](NativeTianjiController &controller, const py::handle &left,
              const py::handle &right, double duration_s) {
             const auto left_q = toArm(left, "left_q");
             const auto right_q = toArm(right, "right_q");
             py::gil_scoped_release release;
             return controller.requestJointTrajectory(left_q, right_q, duration_s);
           },
           py::arg("left_q"), py::arg("right_q"), py::arg("duration_s"))
      .def("wait_for_trajectory", &NativeTianjiController::waitForTrajectory,
           py::arg("timeout_s"), py::call_guard<py::gil_scoped_release>())
      .def("get_state", &NativeTianjiController::getState)
      .def("get_status", &NativeTianjiController::getStatus)
      .def("cancel", &NativeTianjiController::cancel)
      .def("stop", &NativeTianjiController::stop,
           py::call_guard<py::gil_scoped_release>())
      .def("estop", &NativeTianjiController::estop,
           py::call_guard<py::gil_scoped_release>())
      .def_static("identity_pose", &NativeTianjiController::identityPose)
      .def_static("interpolate_pose", &NativeTianjiController::interpolatePose,
                  py::arg("start"), py::arg("end"), py::arg("alpha"))
      .def_static("pose_translation_distance",
                  &NativeTianjiController::poseTranslationDistance)
      .def_static("pose_angular_distance",
                  &NativeTianjiController::poseAngularDistance)
      .def_static("minimum_jerk", &NativeTianjiController::minimumJerk);
}
