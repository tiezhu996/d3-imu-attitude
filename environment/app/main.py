#!/usr/bin/env python3
"""Replay a short inertial run using a calibration table and raw gyro samples."""

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
    norm = math.sqrt(sum(value * value for value in q))
    if norm == 0.0:
        raise ValueError("zero quaternion")
    return [value / norm for value in q]


def quat_exp(vector):
    x, y, z = vector
    angle = math.sqrt(x * x + y * y + z * z)
    if angle < 1.0e-12:
        return quat_normalize([1.0, 0.5 * x, 0.5 * y, 0.5 * z])
    scale = math.sin(0.5 * angle) / angle
    return [math.cos(0.5 * angle), scale * x, scale * y, scale * z]


def canonical(q):
    q = quat_normalize(q)
    if q[0] < 0.0:
        return [-value for value in q]
    return q


def matvec(matrix, vector):
    return [
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    ]


def identity_matrix():
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


def invert_matrix(matrix):
    a, b, c = matrix[0]
    d, e, f = matrix[1]
    g, h, i = matrix[2]
    determinant = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(determinant) < 1.0e-12:
        raise ValueError("singular calibration matrix")
    return [
        [(e * i - f * h) / determinant, (c * h - b * i) / determinant, (b * f - c * e) / determinant],
        [(f * g - d * i) / determinant, (a * i - c * g) / determinant, (c * d - a * f) / determinant],
        [(d * h - e * g) / determinant, (b * g - a * h) / determinant, (a * e - b * d) / determinant],
    ]


def estimate_calibration(rows, nominal_matrix):
    matrix = [[float(value) for value in row] for row in nominal_matrix]
    residuals = []
    for row in rows:
        predicted = matvec(matrix, row["true_rate"])
        residuals.append([row["raw_gyro"][i] - predicted[i] for i in range(3)])
    bias = [sum(item[i] for item in residuals) / len(residuals) for i in range(3)]
    return matrix, bias, [0.0, 0.0, 0.0], []


def corrected_rate(raw, temperature, matrix, bias, temp_coefficient, inverse):
    centered = temperature - 25.0
    value = [
        raw[i] - bias[i] - temp_coefficient[i] * centered
        for i in range(3)
    ]
    return matvec(inverse, value)


def run(data):
    matrix, bias, temp_coefficient, rejected = estimate_calibration(
        data["calibration"]["samples"], data["calibration"]["nominal_matrix"]
    )
    inverse = invert_matrix(matrix)

    samples = data["samples"]
    checkpoints = {round(float(value), 12) for value in data["checkpoints"]}
    q = quat_normalize(data["initial_quaternion"])
    attitudes = []
    if 0.0 in checkpoints:
        attitudes.append({"t": 0.0, "q_wxyz": canonical(q)})

    for index in range(len(samples) - 1):
        current = samples[index]
        following = samples[index + 1]
        dt = float(following["t"]) - float(current["t"])
        if dt <= 0.0:
            raise ValueError("sample times must increase")
        rate = corrected_rate(
            current["raw_gyro"],
            float(current["temperature_c"]),
            matrix,
            bias,
            temp_coefficient,
            inverse,
        )
        step = quat_exp([rate[0] * dt, rate[1] * dt, rate[2] * dt])
        q = quat_normalize(quat_mul(q, step))
        stamp = round(float(following["t"]), 12)
        if stamp in checkpoints:
            attitudes.append({"t": stamp, "q_wxyz": canonical(q)})

    return {
        "run_id": data["run_id"],
        "calibration": {
            "matrix": matrix,
            "bias_at_25c": bias,
            "bias_temp_coefficient": temp_coefficient,
            "rejected_calibration_indices": rejected,
        },
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
