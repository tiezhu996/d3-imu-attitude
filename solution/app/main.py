#!/usr/bin/env python3
"""Replay a short inertial run from sampled body rates."""

import json
import math
import sys


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]


def quat_normalize(q):
    n = math.sqrt(sum(v * v for v in q))
    if n == 0.0:
        raise ValueError("zero quaternion")
    return [v / n for v in q]


def canonical(q):
    q = quat_normalize(q)
    if q[0] < 0.0:
        return [-v for v in q]
    return q


def quat_exp(rotation_vector):
    ax, ay, az = rotation_vector
    angle = math.sqrt(ax * ax + ay * ay + az * az)
    if angle < 1.0e-12:
        return quat_normalize([1.0, 0.5 * ax, 0.5 * ay, 0.5 * az])
    scale = math.sin(0.5 * angle) / angle
    return [math.cos(0.5 * angle), scale * ax, scale * ay, scale * az]


def window_mean(samples, window):
    start, end = window
    rows = [s["gyro"] for s in samples if start <= s["t"] <= end]
    if not rows:
        raise ValueError("empty stationary window")
    return [sum(row[i] for row in rows) / len(rows) for i in range(3)]


def interpolate_bias(bias_start, bias_end, start_time, end_time, stamp):
    if stamp <= start_time:
        return list(bias_start)
    if stamp >= end_time:
        return list(bias_end)
    fraction = (stamp - start_time) / (end_time - start_time)
    return [
        bias_start[i] + fraction * (bias_end[i] - bias_start[i])
        for i in range(3)
    ]


def run(data):
    samples = data["samples"]
    windows = data["static_windows"]
    checkpoints = {round(float(t), 12) for t in data["checkpoints"]}
    q = quat_normalize(data["initial_quaternion"])

    first_window = windows[0]
    last_window = windows[-1]
    bias_start = window_mean(samples, first_window)
    bias_end = window_mean(samples, last_window)
    drift_start = float(first_window[1])
    drift_end = float(last_window[0])

    calibration_start = float(first_window[0])

    def bias_at(stamp):
        stamp = float(stamp)
        if stamp < calibration_start:
            return [0.0, 0.0, 0.0]
        return interpolate_bias(
            bias_start, bias_end, drift_start, drift_end, stamp
        )

    attitudes = []
    if 0.0 in checkpoints:
        attitudes.append({"t": 0.0, "q_wxyz": canonical(q)})

    for index in range(len(samples) - 1):
        current = samples[index]
        following = samples[index + 1]
        dt = float(following["t"]) - float(current["t"])
        if dt <= 0.0:
            raise ValueError("sample times must increase")

        offset = bias_at(current["t"])
        omega = [
            float(current["gyro"][i]) - offset[i]
            for i in range(3)
        ]
        step = quat_exp([omega[0] * dt, omega[1] * dt, omega[2] * dt])
        q = quat_normalize(quat_mul(q, step))

        stamp = round(float(following["t"]), 12)
        if stamp in checkpoints:
            attitudes.append({"t": stamp, "q_wxyz": canonical(q)})

    return {
        "run_id": data["run_id"],
        "bias_start": bias_start,
        "bias_end": bias_end,
        "attitudes": attitudes,
    }


def main():
    if len(sys.argv) != 2:
        raise SystemExit(2)
    with open(sys.argv[1], "r", encoding="utf-8") as handle:
        data = json.load(handle)
    print(json.dumps(run(data), ensure_ascii=False, separators=(",", ":")))


if __name__ == "__main__":
    main()
