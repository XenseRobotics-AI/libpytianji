# libpytianji

`libpytianji` is the native real-time control backend used by LeRobot's
`TianjiArm` adapter. It keeps the vendor Marvin SDK calls and the control
calculation chain out of Python:

- a C++ planner thread runs at 300 Hz and performs arrival-time-based
  Cartesian interpolation and per-sample warm-started Pink-style IK.  Each
  Cartesian task is assembled as a fixed-size, box-constrained velocity QP;
  the small active-set solver is embedded in the extension for deterministic
  RT latency;
- a separate C++ I/O thread polls the controller at 1 kHz and sends atomic
  dual-arm PD joint commands every 5 ms;
- pybind11 exposes a small latest-target mailbox API, state snapshots, Home
  trajectories, cancellation, and emergency stop.

The package links the headers and shared libraries in the sibling vendor
nested `vendor/tianji_arm` checkout, plus the C++ Pinocchio runtime used by the
native Pink migration. Its model/task code is independent of `tron2-ik`; that
project is only the reference for translating Pink's Python control flow into
a deterministic C++ RT loop. The wheel includes the Tianji runtime libraries,
the default `ccs_m6_40.MvKDCfg` kinematics configuration, and the Marvin
`marvin_m6.urdf` model used by the hardware-free IK viewer. After installation,
runtime control and IK do not import `tianji_arm` or depend on the vendor
checkout; the checkout is needed only while building the wheel from source.

It is normally installed as a wheel by:

```bash
bash setup_env.sh --install tianji
```

For a wheel that has already been built (or published to an internal index),
installation is the ordinary Python package flow:

```bash
python -m pip install libpytianji-0.1.4-*.whl
```

This installs the `pin` runtime declared in the package metadata and places
the native controller, Tianji vendor libraries, kinematics config, URDF, and
meshes under `site-packages/libpytianji/`. The native API exposes controller
startup/shutdown, Cartesian target streaming, joint trajectories, state/status
snapshots, cancellation, stop/e-stop, and hardware-free FK/IK diagnostics.
The package does not expose every unrelated vendor demo feature (for example
PVT files, force-control modes, tool I/O, or physical gripper drivers); those
remain outside the LeRobot control contract.

For a local build, use the active Python 3.12 environment and set
`TIANJI_SDK_ROOT` when the vendor checkout is elsewhere:

```bash
cd third_party/libpytianji
TIANJI_SDK_ROOT=/path/to/tianji_arm \
  python -m pip wheel --no-build-isolation --no-deps . -w dist
```

If Pinocchio is installed outside the active interpreter's `cmeel.prefix`,
set `LIBPYTIANJI_PINOCCHIO_PREFIX` to that C++ installation prefix. A direct
`pip install` from this source tree therefore requires the Tianji SDK checkout
at build time; installing a built wheel requires only Python, `pin`, and the
wheel's bundled native Tianji libraries.

The URDF's referenced STL meshes are included as package data as well, so the
optional MeshCat viewer can render the model after installation without the
source `robosuite-xense` checkout.

The Python boundary uses metres, radians, and row-major 4x4 homogeneous poses;
vendor millimetre/degree conversions remain entirely in C++.

## Runtime logging

The vendor libraries expose binary switches rather than severity levels.
`libpytianji` turns the vendor kinematics/FK logging and Marvin local logging
off by default, so
normal FK/IK and per-command messages do not flood the terminal or delay the
real-time threads.  Some vendor error text is controlled by the same binary
switch, while Marvin diagnostics that bypass the switch remain visible; the
adapter therefore reports native IK rejects, Cartesian-input timeouts, and
scheduler overruns through rate-limited Python warnings.  To temporarily
restore the complete raw vendor trace for a focused investigation, run:

```bash
LIBPYTIANJI_VENDOR_LOG=1 lerobot-teleoperate ...
```

`LIBPYTIANJI_VENDOR_KINE_LOG=1` remains accepted as a compatibility alias.

## Hardware-free model sandbox

The LeRobot package exposes a lightweight MeshCat + Qt FK/IK viewer for the
Marvin URDF. It never loads the native controller or opens a robot socket:

```bash
lerobot-tianji-sim --headless --no-meshcat
uv pip install -e '.[tianji-sim]'
lerobot-tianji-sim --no-open-browser
```

The first command validates both TCP frames and warm-started FK→IK without a
display. The optional GUI provides joint and Cartesian sliders. Production
teleoperation uses the independent native `pink-pinocchio-native` backend; it
does not invoke Python Pink/qpsolvers or acquire the Python GIL in the 300 Hz
planner.

The native IK allows up to 500 iterations per arm and exits early once both
position error is at most 0.1 mm and orientation error is at most 0.001 rad.
Each velocity QP retains its 64-iteration active-set limit and existing
joint-position and velocity bounds. The IK iteration limit is a C++ default,
not a Python constructor option; changing it requires rebuilding the extension.
This is an iteration budget, not a wall-clock deadline: difficult or unreachable
targets can consume the full budget, so check dual-arm planner timing at 300 Hz.

The Tianji streaming defaults use a Hessian damping coefficient of `1e-3`
(the `lambda^2` term in damped least squares) and a frame gain of `0.5`.
These penalize large joint velocities and moderate each linearized correction.
They do not impose an acceleration limit or guarantee continuity across IK
failures. Position/orientation tolerances and the physical safety gates are
unchanged. The values are model-specific tuning, not published cross-wrist
constants. Sources for the method:

- [Pink task gain and damping](https://github.com/stephane-caron/pink/blob/main/pink/tasks/task.py)
- [Pink global Tikhonov regularization](https://github.com/stephane-caron/pink/blob/main/pink/solve_ik.py)
- [Robotics Toolbox LM damping methods](https://github.com/petercorke/robotics-toolbox-python/blob/main/src/roboticstoolbox/robot/IK.py)

Run `python third_party/libpytianji/benchmarks/ik_singularity.py` from the
repository root to compare rebuilt versions without connecting hardware.
The 600-waypoint extended-arm stress case includes infeasible targets and
models only numerical IK plus output velocity clipping, not full control.
On the development machine, increasing damping from `1e-4` to `1e-3` and
reducing gain from `1` to `0.5` reduced joint-increment-change RMS from about
0.0787 to 0.0532 degrees and the maximum from 1.20 to 0.616 degrees on both
arms. Failures remained comparable (177 versus 178 of 600); this does not
establish hardware safety or eliminate singularity failures. Ordinary bent-arm
tracking retained the 0.1 mm tolerance with no failures. A trial of relaxed
convergence tolerances was rejected because it increased stop/start motion.

To validate the exact production model after a wheel install:

```bash
python - <<'PY'
import libpytianji
model = libpytianji.PinkIKModel()
print(libpytianji.IK_BACKEND, model.forward_kinematics(0, [0.0] * 7)[3::4])
PY
```

`NativeTianjiController` uses each arm's `tool_kine` as the single TCP
definition for both Marvin `SetTool` and the native Pink/Pinocchio FK/IK
frame. The convention is flange-relative
`[x_mm, y_mm, z_mm, A_deg, B_deg, C_deg]`, with
`R = Rz(C) * Ry(B) * Rx(A)`. A zero vector selects the factory flange center.
