"""Offline numerical stress test; no robot connection or hardware safety gate.

Run in the lerobot-xensehand environment after rebuilding the native extension.
Both arms follow 600 synthetic Cartesian waypoints with a 300 Hz / 180 deg/s
output envelope. The extended case deliberately includes difficult/unreachable
targets. Its Jacobian singular values mix metres and radians and are only a
within-model comparison. This is not a simulation of actuator dynamics or the
controller's target-sequence handling, vendor limits, or J6/J7 coupled limits.
"""

import json
import time

import libpytianji
import numpy as np
from scipy.spatial.transform import Rotation

m = libpytianji.PinkIKModel()
step = np.full(7, np.pi / 300)


def fk(arm, q):
    return np.array(m.forward_kinematics(arm, q.tolist())).reshape(4, 4)


def sigma(arm, q):
    t = fk(arm, q)
    j = np.empty((6, 7))
    for i in range(7):
        p = q.copy()
        p[i] += 1e-6
        u = fk(arm, p)
        j[:3, i] = (u[:3, 3] - t[:3, 3]) / 1e-6
        j[3:, i] = Rotation.from_matrix(u[:3, :3] @ t[:3, :3].T).as_rotvec() / 1e-6
    return np.linalg.svd(j, compute_uv=False)[-1]


for arm in [0, 1]:
    for label, seed in [
        ("bent", [0.21, -0.37, 0.16, -0.82, 0.19, -0.31, 0.12]),
        ("extended", [0.1, -0.1, 0.1, -0.04, 0.1, 0.0, 0.0]),
    ]:
        seed = np.array(seed)
        q = seed.copy()
        base = fk(arm, q)
        samples = []
        times = []
        fail = 0
        err = []
        for i in range(600):
            target = base.copy()
            target[:3, 3] += np.array(
                [0.002 * np.sin(i * 0.04), 0.001 * np.sin(i * 0.07), 0.001 * np.cos(i * 0.04)]
            )
            start = time.perf_counter_ns()
            r = m.solve(arm, target.ravel().tolist(), q.tolist(), step.tolist())
            times.append((time.perf_counter_ns() - start) / 1e6)
            if r["success"]:
                q = q + np.clip(np.array(r["q"]) - q, -step, step)
            else:
                fail += 1
            samples.append(q.copy())
            err.append(np.linalg.norm(fk(arm, q)[:3, 3] - target[:3, 3]))
        dq = np.diff(samples, axis=0)
        ddq = np.diff(dq, axis=0)
        print(
            json.dumps(
                {
                    "arm": arm,
                    "case": label,
                    "sigma_min": sigma(arm, seed),
                    "failures": fail,
                    "max_delta_deg": float(np.abs(dq).max() * 180 / np.pi),
                    "rms_delta_change_deg": float(np.sqrt(np.mean(ddq**2)) * 180 / np.pi),
                    "max_delta_change_deg": float(np.abs(ddq).max() * 180 / np.pi),
                    "max_position_error_mm": max(err) * 1000,
                    "p99_ms": float(np.percentile(times, 99)),
                }
            )
        )
