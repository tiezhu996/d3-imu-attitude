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


def window_mean(samples, window):
    start, end = window
    rows = [s["gyro"] for s in samples if start <= s["t"] <= end]
    if not rows:
        raise ValueError("empty stationary window")
    return [sum(row[i] for row in rows) / len(rows) for i in range(3)]


def run(data):
    samples = data["samples"]
    windows = data["static_windows"]
    checkpoints = {round(float(t), 12) for t in data["checkpoints"]}
    q = quat_normalize(data["initial_quaternion"])

    constant = [float(value) for value in samples[0]["gyro"]]

    attitudes = []
    if 0.0 in checkpoints:
        attitudes.append({"t": 0.0, "q_wxyz": list(q)})

    for index in range(len(samples) - 1):
        current = samples[index]
        following = samples[index + 1]
        dt = float(following["t"]) - float(current["t"])
        if dt <= 0.0:
            raise ValueError("sample times must increase")

        omega = [float(current["gyro"][i]) - constant[i] for i in range(3)]
        small_step = [
            1.0,
            0.5 * omega[0] * dt,
            0.5 * omega[1] * dt,
            0.5 * omega[2] * dt,
        ]
        q = quat_normalize(quat_mul(small_step, q))

        stamp = round(float(following["t"]), 12)
        if stamp in checkpoints:
            attitudes.append({"t": stamp, "q_wxyz": list(q)})

    return {
        "run_id": data["run_id"],
        "bias_start": constant,
        "bias_end": constant,
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
