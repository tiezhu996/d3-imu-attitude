"""Acceptance tests for short inertial attitude replay.

Expected trajectories come from closed-form quaternion kinematics in this file.
They do not import or execute the submitted solution while computing expectations.
"""

import json
import math
import os
import signal
import subprocess
import tempfile
from pathlib import Path

ENTRY = "/app/main.py"
PYTHON = "/usr/local/bin/python3"
CANDIDATE_USER = "runner"
CANDIDATE_ENV = {
    "PATH": "/usr/local/bin:/usr/bin:/bin",
    "HOME": "/home/runner",
    "LANG": "C.UTF-8",
    "TZ": "UTC",
}
CALL_TIMEOUT_SEC = 30
ROTATION_TOL = 2.0e-5


def run_candidate(*args, timeout=CALL_TIMEOUT_SEC):
    """Run the candidate as runner and terminate its process group on timeout."""
    as_root = os.getuid() == 0
    proc = subprocess.Popen(
        [PYTHON, ENTRY, *[str(a) for a in args]],
        cwd="/app",
        env=CANDIDATE_ENV,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
        user=CANDIDATE_USER if as_root else None,
        group=CANDIDATE_USER if as_root else None,
        extra_groups=[] if as_root else None,
    )
    try:
        out, err = proc.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.communicate()
        raise TimeoutError(f"{ENTRY} exceeded {timeout}s")
    return subprocess.CompletedProcess(proc.args, proc.returncode, out, err)


def run_case(payload):
    folder = Path(tempfile.mkdtemp(prefix="imu-"))
    folder.chmod(0o755)
    case = folder / "input.json"
    case.write_text(json.dumps(payload), encoding="utf-8")
    case.chmod(0o644)
    proc = run_candidate(case)
    assert proc.returncode == 0, f"exit={proc.returncode} stderr={proc.stderr[-800:]}"
    return json.loads(proc.stdout)


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]


def qexp(vector):
    x, y, z = vector
    angle = math.sqrt(x * x + y * y + z * z)
    if angle < 1e-14:
        return [1.0, 0.5 * x, 0.5 * y, 0.5 * z]
    scale = math.sin(0.5 * angle) / angle
    return [math.cos(0.5 * angle), scale * x, scale * y, scale * z]


def qnormalize(q):
    norm = math.sqrt(sum(value * value for value in q))
    return [value / norm for value in q]


def rotation_error(got, want):
    a = qnormalize([float(value) for value in got])
    b = qnormalize(want)
    dot = abs(sum(a[i] * b[i] for i in range(4)))
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


def checkpoint_q(result, stamp):
    rows = [row for row in result["attitudes"] if abs(float(row["t"]) - stamp) < 1e-12]
    assert len(rows) == 1, f"checkpoint {stamp} count={len(rows)}"
    return rows[0]["q_wxyz"]


def uniform_samples(end, step, gyro_at):
    count = int(round(end / step))
    stamps = [round(i * step, 12) for i in range(count + 1)]
    return [{"t": stamp, "gyro": [float(v) for v in gyro_at(stamp)]} for stamp in stamps]


def zero_frame(end=0.2, step=0.1):
    return {
        "run_id": "contract",
        "initial_quaternion": [1.0, 0.0, 0.0, 0.0],
        "samples": uniform_samples(end, step, lambda _t: [0.0, 0.0, 0.0]),
        "static_windows": [[0.0, 0.1], [0.1, 0.2]],
        "checkpoints": [0.2],
    }


def constant_body_case(q0, rate, motion_start, motion_end, step):
    end = motion_end + 0.2 + step

    def gyro_at(stamp):
        if motion_start <= stamp < motion_end:
            return rate
        return [0.0, 0.0, 0.0]

    return {
        "run_id": "body-rate",
        "initial_quaternion": q0,
        "samples": uniform_samples(end, step, gyro_at),
        "static_windows": [[0.0, 0.1], [motion_end + 0.1, end]],
        "checkpoints": [motion_end],
    }


def coarse_rate_case(rate, motion_start, motion_end, step):
    end = motion_end + 0.2 + step

    def gyro_at(stamp):
        if motion_start <= stamp < motion_end:
            return rate
        return [0.0, 0.0, 0.0]

    return {
        "run_id": "coarse",
        "initial_quaternion": [1.0, 0.0, 0.0, 0.0],
        "samples": uniform_samples(end, step, gyro_at),
        "static_windows": [[0.0, 0.1], [motion_end + 0.1, end]],
        "checkpoints": [motion_end],
    }


def drifting_bias_case():
    step = 0.001
    end = 3.0
    first = [0.010, -0.004, 0.007]
    last = [0.016, -0.001, 0.010]
    window_first = [0.1, 0.4]
    window_last = [2.6, 2.9]
    motion_start = 0.6
    motion_end = 2.5

    def bias_at(stamp):
        if stamp < window_first[0]:
            return [0.0, 0.0, 0.0]
        if stamp <= window_first[1]:
            return list(first)
        if stamp >= window_last[0]:
            return list(last)
        ratio = (stamp - window_first[1]) / (window_last[0] - window_first[1])
        return [first[i] + ratio * (last[i] - first[i]) for i in range(3)]

    def gyro_at(stamp):
        bias = bias_at(stamp)
        body_z = 0.45 if motion_start <= stamp < motion_end else 0.0
        return [bias[0], bias[1], bias[2] + body_z]

    return {
        "run_id": "bias-drift",
        "initial_quaternion": [1.0, 0.0, 0.0, 0.0],
        "samples": uniform_samples(end, step, gyro_at),
        "static_windows": [window_first, window_last],
        "checkpoints": [motion_end],
    }, first, last


def expected_drift_attitude(payload):
    samples = payload["samples"]
    first = payload["static_windows"][0]
    last = payload["static_windows"][-1]
    start_mean = [sum(row["gyro"][i] for row in samples if first[0] <= row["t"] <= first[1]) /
                  len([row for row in samples if first[0] <= row["t"] <= first[1]]) for i in range(3)]
    last_rows = [row for row in samples if last[0] <= row["t"] <= last[1]]
    end_mean = [sum(row["gyro"][i] for row in last_rows) / len(last_rows) for i in range(3)]

    def bias_at(stamp):
        if stamp < first[0]:
            return [0.0, 0.0, 0.0]
        if stamp <= first[1]:
            return start_mean
        if stamp >= last[0]:
            return end_mean
        ratio = (stamp - first[1]) / (last[0] - first[1])
        return [start_mean[i] + ratio * (end_mean[i] - start_mean[i]) for i in range(3)]

    q = [1.0, 0.0, 0.0, 0.0]
    for index in range(len(samples) - 1):
        current = samples[index]
        following = samples[index + 1]
        dt = following["t"] - current["t"]
        bias = bias_at(current["t"])
        omega = [current["gyro"][i] - bias[i] for i in range(3)]
        q = qnormalize(qmul(q, qexp([omega[0] * dt, omega[1] * dt, omega[2] * dt])))
        if following["t"] == 2.5:
            break
    return q


# ---------- gate: entry contract ----------

def test_prints_contract_fields():
    """「程序向标准输出写一个 JSON 对象，字段为 `run_id`、`bias_start`、`bias_end` 和 `attitudes`，不要输出其它文字」"""
    got = run_case(zero_frame())
    assert set(got) == {"run_id", "bias_start", "bias_end", "attitudes"}
    assert got["run_id"] == "contract"
    assert len(got["attitudes"]) == 1


def test_wrong_argument_count_exits_2():
    """「参数个数不对时退出码为 2，不输出 JSON」"""
    proc = run_candidate()
    assert proc.returncode == 2
    assert proc.stdout.strip() == ""


# ---------- difficulty_1: body-frame composition ----------

def test_attitude_uses_body_rotation_after_initial_heading():
    """「姿态表示从机体系到世界系的旋转」"""
    q0 = [0.8191520442889918, 0.0, 0.0, 0.573576436351046]
    rate = [0.9, 0.4, -0.2]
    duration = 0.7
    payload = constant_body_case(q0, rate, 0.2, 0.9, 0.0005)
    got = checkpoint_q(run_case(payload), 0.9)
    want = qnormalize(qmul(q0, qexp([rate[i] * duration for i in range(3)])))
    assert rotation_error(got, want) < ROTATION_TOL, rotation_error(got, want)


def test_cross_axis_body_rate_preserves_rotation_order():
    """「分量在机体系下给出」"""
    q0 = [0.5, 0.5, 0.5, 0.5]
    rate = [0.8, -0.5, 1.2]
    duration = 0.65
    payload = constant_body_case(q0, rate, 0.2, 0.85, 0.0005)
    got = checkpoint_q(run_case(payload), 0.85)
    want = qnormalize(qmul(q0, qexp([rate[i] * duration for i in range(3)])))
    assert rotation_error(got, want) < ROTATION_TOL, rotation_error(got, want)


def test_rotated_initial_frame_keeps_body_axis_motion():
    """「每个采样值保持到下一个采样时刻」"""
    q0 = [0.9238795325112867, 0.3826834323650898, 0.0, 0.0]
    rate = [0.3, 0.9, -0.4]
    duration = 0.8
    payload = constant_body_case(q0, rate, 0.2, 1.0, 0.0005)
    got = checkpoint_q(run_case(payload), 1.0)
    want = qnormalize(qmul(q0, qexp([rate[i] * duration for i in range(3)])))
    assert rotation_error(got, want) < ROTATION_TOL, rotation_error(got, want)


# ---------- difficulty_2: time-varying gyro bias ----------

def test_start_bias_uses_first_stationary_window():
    """「`bias_start` 取首段静止数据的均值」"""
    payload, first, _last = drifting_bias_case()
    got = run_case(payload)["bias_start"]
    assert max(abs(got[i] - first[i]) for i in range(3)) < 1e-12


def test_end_bias_uses_last_stationary_window():
    """「`bias_end` 取末段静止数据的均值」"""
    payload, _first, last = drifting_bias_case()
    got = run_case(payload)["bias_end"]
    assert max(abs(got[i] - last[i]) for i in range(3)) < 1e-12


def test_attitude_removes_linearly_drifting_bias():
    """「在首段结束到末段开始之间线性变化」"""
    payload, _first, _last = drifting_bias_case()
    got = checkpoint_q(run_case(payload), 2.5)
    want = expected_drift_attitude(payload)
    assert rotation_error(got, want) < 2.0e-5, rotation_error(got, want)


# ---------- difficulty_3: finite rotation propagation ----------

def test_exact_rotation_across_one_coarse_interval():
    """「每个采样值保持到下一个采样时刻」"""
    rate = [0.0, 0.0, 9.0]
    payload = coarse_rate_case(rate, 0.2, 1.2, 0.2)
    got = checkpoint_q(run_case(payload), 1.2)
    want = qexp([rate[i] * 1.0 for i in range(3)])
    assert rotation_error(got, want) < 2.0e-4, rotation_error(got, want)


def test_fast_moderate_rate_rotation_uses_exact_step():
    """「最后一个采样点不参与积分」"""
    rate = [0.0, 7.4, 0.0]
    payload = coarse_rate_case(rate, 0.3, 1.2, 0.15)
    got = checkpoint_q(run_case(payload), 1.2)
    want = qexp([rate[i] * 0.9 for i in range(3)])
    assert rotation_error(got, want) < 2.0e-4, rotation_error(got, want)


def test_full_turn_high_rate_keeps_attitude():
    """「采样间隔较大的高速试验偏差最明显」"""
    rate = [0.0, 0.0, 2.0 * math.pi]
    payload = coarse_rate_case(rate, 0.2, 1.2, 0.1)
    got = checkpoint_q(run_case(payload), 1.2)
    want = qexp([rate[i] * 1.0 for i in range(3)])
    assert rotation_error(got, want) < 2.0e-4, rotation_error(got, want)
