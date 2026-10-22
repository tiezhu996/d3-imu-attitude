"""Acceptance tests for calibrated short inertial attitude replay."""

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


def run_candidate(*args, timeout=CALL_TIMEOUT_SEC):
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
    folder = Path(tempfile.mkdtemp(prefix="imu-cal-"))
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


def matvec(matrix, vector):
    return [sum(matrix[row][column] * vector[column] for column in range(3)) for row in range(3)]


def driving_rates(index):
    return [
        0.86 * math.sin(0.37 * index + 0.2),
        0.66 * math.sin(0.53 * index + 1.1),
        0.74 * math.cos(0.29 * index + 0.7),
    ]


def conditioned_rates(index):
    theta = 0.31 * index + 0.4
    epsilon = 1.0e-5
    return [
        0.9 * math.sin(theta),
        0.9 * math.sin(theta + epsilon),
        0.9 * math.cos(theta + 2.0 * epsilon) + epsilon * math.sin(7.0 * theta),
    ]


def calibration_rows(matrix, bias, coefficient, temperatures, count, noise=0.0, spikes=None, rate_mode="normal"):
    spikes = spikes or {}
    rows = []
    for index in range(count):
        true_rate = conditioned_rates(index) if rate_mode == "conditioned" else driving_rates(index)
        temperature = temperatures[index % len(temperatures)]
        centered = temperature - 25.0
        raw = matvec(matrix, true_rate)
        for axis in range(3):
            raw[axis] += bias[axis] + coefficient[axis] * centered
            raw[axis] += noise * math.sin((index + 1) * (axis + 2) * 0.43)
        if index in spikes:
            raw = [raw[axis] + spikes[index][axis] for axis in range(3)]
        rows.append({
            "raw_gyro": raw,
            "true_rate": true_rate,
            "temperature_c": temperature,
        })
    return rows


def replay_samples(matrix, bias, coefficient, rate, motion_start, motion_end, temperature, step=0.0005):
    total = motion_end + 0.2 + step

    def raw_at(stamp):
        true_rate = rate if motion_start <= stamp < motion_end else [0.0, 0.0, 0.0]
        raw = matvec(matrix, true_rate)
        centered = temperature - 25.0
        return [raw[i] + bias[i] + coefficient[i] * centered for i in range(3)]

    count = int(round(total / step))
    return [
        {
            "t": round(index * step, 12),
            "raw_gyro": raw_at(round(index * step, 12)),
            "temperature_c": temperature,
        }
        for index in range(count + 1)
    ]


def make_payload(matrix, bias, coefficient, temperatures, count, replay_rate,
                 q0, duration, temperature_replay, noise=0.0, spikes=None, rate_mode="normal"):
    motion_start = 0.2
    motion_end = motion_start + duration
    payload = {
        "run_id": "calibrated-replay",
        "calibration": {
            "samples": calibration_rows(
                matrix, bias, coefficient, temperatures, count, noise=noise, spikes=spikes,
                rate_mode=rate_mode
            )
        },
        "initial_quaternion": q0,
        "samples": replay_samples(
            matrix, bias, coefficient, replay_rate, motion_start, motion_end, temperature_replay
        ),
        "checkpoints": [motion_end],
    }
    expected = qnormalize(qmul(q0, qexp([replay_rate[i] * duration for i in range(3)])))
    return payload, expected, motion_end


def matrix_case(case_id):
    if case_id == 1:
        matrix = [
            [1.03, 0.02, -0.01],
            [-0.015, 0.97, 0.025],
            [0.01, -0.02, 1.05],
        ]
        q0 = [0.8191520442889918, 0.0, 0.0, 0.573576436351046]
        rate = [0.9, 0.4, -0.2]
        duration = 0.7
    elif case_id == 2:
        matrix = [
            [0.94, -0.03, 0.018],
            [0.022, 1.06, -0.012],
            [-0.017, 0.014, 0.96],
        ]
        q0 = [0.5, 0.5, 0.5, 0.5]
        rate = [0.8, -0.5, 1.2]
        duration = 0.65
    else:
        matrix = [
            [1.08, 0.012, 0.025],
            [-0.021, 0.93, 0.019],
            [0.016, -0.024, 1.04],
        ]
        q0 = [0.9238795325112867, 0.3826834323650898, 0.0, 0.0]
        rate = [0.3, 0.9, -0.4]
        duration = 0.8
    bias = [0.012, -0.008, 0.006]
    payload, expected, stamp = make_payload(
        matrix, bias, [0.0, 0.0, 0.0], [25.0], 90, rate, q0, duration, 25.0,
        noise=0.0, rate_mode="conditioned" if case_id == 2 else "normal"
    )
    return payload, matrix, expected, stamp


def temperature_case():
    matrix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    bias = [0.010, -0.006, 0.004]
    coefficient = [0.0016, -0.0021, 0.0012]
    temperatures = [15.0 + 0.5 * index for index in range(41)]
    payload, expected, stamp = make_payload(
        matrix, bias, coefficient, temperatures, 100, [0.55, -0.35, 0.45],
        [0.7071067811865476, 0.0, 0.7071067811865475, 0.0], 0.8, 31.0, noise=0.00002
    )
    return payload, bias, coefficient, expected, stamp


def outlier_case():
    matrix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    bias = [0.009, -0.007, 0.005]
    spikes = {
        11: [1.3, -0.8, 0.7],
        29: [-0.9, 1.4, -1.1],
        47: [1.1, 0.9, -0.8],
        66: [-1.2, -0.7, 1.3],
    }
    payload, expected, stamp = make_payload(
        matrix, bias, [0.0, 0.0, 0.0], [25.0], 80, [0.62, 0.38, -0.44],
        [0.9659258262890683, 0.25881904510252074, 0.0, 0.0], 0.75, 25.0,
        noise=0.00002, spikes=spikes
    )
    return payload, bias, sorted(spikes), expected, stamp


# ---------- gate: entry contract ----------

def test_prints_contract_fields():
    """「程序向标准输出写一个 JSON 对象，字段为 `run_id`、`calibration` 和 `attitudes`，不要输出其它文字」"""
    payload, _matrix, _expected, _stamp = matrix_case(1)
    got = run_case(payload)
    assert set(got) == {"run_id", "calibration", "attitudes"}
    assert set(got["calibration"]) == {
        "matrix", "bias_at_25c", "bias_temp_coefficient", "rejected_calibration_indices"
    }


def test_wrong_argument_count_exits_2():
    """「参数个数不对时退出码为 2，不输出 JSON」"""
    proc = run_candidate()
    assert proc.returncode == 2
    assert proc.stdout.strip() == ""


# ---------- difficulty_1: full cross-axis calibration matrix ----------

def test_matrix_recovers_cross_axis_gain():
    """「`matrix` 把真实机体系角速度映射为陀螺原始读数」"""
    payload, want, _expected, _stamp = matrix_case(1)
    got = run_case(payload)["calibration"]["matrix"]
    assert max(abs(got[row][column] - want[row][column]) for row in range(3) for column in range(3)) < 5.0e-5


def test_matrix_inverse_drives_attitude():
    """「姿态表示从机体系到世界系的旋转」"""
    payload, _matrix, expected, stamp = matrix_case(2)
    got = checkpoint_q(run_case(payload), stamp)
    assert rotation_error(got, expected) < 2.0e-4, rotation_error(got, expected)


def test_matrix_handles_rotated_initial_frame():
    """「每个采样值保持到下一个采样时刻」"""
    payload, _matrix, expected, stamp = matrix_case(3)
    got = checkpoint_q(run_case(payload), stamp)
    assert rotation_error(got, expected) < 2.0e-4, rotation_error(got, expected)


# ---------- difficulty_2: temperature-dependent bias ----------

def test_bias_at_reference_temperature():
    """「`bias_at_25c` 是 25°C 的零偏」"""
    payload, bias, _coefficient, _expected, _stamp = temperature_case()
    got = run_case(payload)["calibration"]["bias_at_25c"]
    assert max(abs(got[i] - bias[i]) for i in range(3)) < 5.0e-5


def test_temperature_coefficient_is_recovered():
    """「`bias_temp_coefficient` 是零偏随摄氏温度的变化率」"""
    payload, _bias, coefficient, _expected, _stamp = temperature_case()
    got = run_case(payload)["calibration"]["bias_temp_coefficient"]
    assert max(abs(got[i] - coefficient[i]) for i in range(3)) < 2.0e-5


def test_temperature_correction_drives_attitude():
    """「回放样本带有各自的温度读数」"""
    payload, _bias, _coefficient, expected, stamp = temperature_case()
    got = checkpoint_q(run_case(payload), stamp)
    assert rotation_error(got, expected) < 2.0e-4, rotation_error(got, expected)


# ---------- difficulty_3: outlier rejection ----------

def test_rejected_indices_are_exact():
    """「`rejected_calibration_indices` 按 0 开始的原始顺序列出异常标定样本」"""
    payload, _bias, rejected, _expected, _stamp = outlier_case()
    got = run_case(payload)["calibration"]["rejected_calibration_indices"]
    assert got == rejected


def test_outlier_rejection_recovers_bias():
    """「校准样本可能出现孤立尖峰」"""
    payload, bias, _rejected, _expected, _stamp = outlier_case()
    got = run_case(payload)["calibration"]["bias_at_25c"]
    assert max(abs(got[i] - bias[i]) for i in range(3)) < 6.0e-5


def test_outlier_rejection_drives_attitude():
    """「拟合校准模型时不计入异常标定样本」"""
    payload, _bias, _rejected, expected, stamp = outlier_case()
    got = checkpoint_q(run_case(payload), stamp)
    assert rotation_error(got, expected) < 2.0e-4, rotation_error(got, expected)
