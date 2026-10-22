#!/usr/bin/env python3
"""Replay a short inertial run using a robust calibration fit and raw gyro samples."""

import json
import math
import sys

OUTLIER_THRESHOLD = 0.15
RIDGE = 1.0e-12


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


def solve_linear(matrix, target):
    size = len(target)
    work = [list(matrix[row]) + [float(target[row])] for row in range(size)]
    for _ in range(size):
        diagonal = max(range(_ , size), key=lambda row: abs(work[row][_]))
        work[_], work[diagonal] = work[diagonal], work[_]
        pivot = work[_][_]
        if abs(pivot) < 1.0e-14:
            raise ValueError("singular normal equations")
        for column in range(_, size + 1):
            work[_][column] /= pivot
        for row in range(size):
            if row == _:
                continue
            factor = work[row][_]
            for column in range(_, size + 1):
                work[row][column] -= factor * work[_][column]
    return [work[row][size] for row in range(size)]


def least_squares(features, target):
    size = len(features[0])
    normal = [[0.0] * size for _ in range(size)]
    rhs = [0.0] * size
    for row, value in zip(features, target):
        for left in range(size):
            rhs[left] += row[left] * value
            for right in range(size):
                normal[left][right] += row[left] * row[right]
    for index in range(size):
        normal[index][index] += RIDGE
    return solve_linear(normal, rhs)


def fit_matrix_bias(rows):
    features = [[*row["true_rate"], 1.0] for row in rows]
    matrix = []
    bias = []
    for axis in range(3):
        target = [row["raw_gyro"][axis] for row in rows]
        coefficients = least_squares(features, target)
        matrix.append(coefficients[:3])
        bias.append(coefficients[3])
    return matrix, bias


def fit_bias_temperature(rows, matrix):
    residuals = [
        [row["raw_gyro"][axis] - matvec(matrix, row["true_rate"])[axis] for axis in range(3)]
        for row in rows
    ]
    temperatures = [row["temperature_c"] - 25.0 for row in rows]
    if max(temperatures) - min(temperatures) < 1.0e-9:
        return [sum(item[axis] for item in residuals) / len(residuals) for axis in range(3)], [0.0, 0.0, 0.0]
    features = [[1.0, value] for value in temperatures]
    bias = []
    coefficient = []
    for axis in range(3):
        target = [item[axis] for item in residuals]
        fitted = least_squares(features, target)
        bias.append(fitted[0])
        coefficient.append(fitted[1])
    return bias, coefficient


def fit_joint(rows):
    features = [[*row["true_rate"], 1.0, row["temperature_c"] - 25.0] for row in rows]
    matrix = []
    bias = []
    coefficient = []
    for axis in range(3):
        target = [row["raw_gyro"][axis] for row in rows]
        fitted = least_squares(features, target)
        matrix.append(fitted[:3])
        bias.append(fitted[3])
        coefficient.append(fitted[4])
    return matrix, bias, coefficient


def mean_residual_bias(rows, matrix):
    total = [0.0, 0.0, 0.0]
    for row in rows:
        predicted = matvec(matrix, row["true_rate"])
        for axis in range(3):
            total[axis] += row["raw_gyro"][axis] - predicted[axis]
    return [value / len(rows) for value in total]


def detect_outliers(rows, matrix, bias, coefficient):
    rejected = []
    for position, row in enumerate(rows):
        predicted = matvec(matrix, row["true_rate"])
        centered = row["temperature_c"] - 25.0
        error = max(
            abs(
                row["raw_gyro"][axis]
                - predicted[axis]
                - bias[axis]
                - coefficient[axis] * centered
            )
            for axis in range(3)
        )
        if error > OUTLIER_THRESHOLD:
            rejected.append(position)
    return rejected


def estimate_calibration(rows):
    rejected = []
    matrix_mode = "fitted"
    temperature_mode = "fit"
    outlier_mode = "detect"
    matrix = identity_matrix()
    bias = [0.0, 0.0, 0.0]
    coefficient = [0.0, 0.0, 0.0]
    for _ in range(6):
        clean = [row for position, row in enumerate(rows) if position not in rejected]
        if matrix_mode == "fitted" and temperature_mode == "fit":
            matrix, bias, coefficient = fit_joint(clean)
        elif matrix_mode == "fitted":
            matrix, _matrix_bias = fit_matrix_bias(clean)
            bias = mean_residual_bias(clean, matrix)
            coefficient = [0.0, 0.0, 0.0]
        elif temperature_mode == "fit":
            matrix = identity_matrix()
            bias, coefficient = fit_bias_temperature(clean, matrix)
        else:
            matrix = identity_matrix()
            bias = mean_residual_bias(clean, matrix)
            coefficient = [0.0, 0.0, 0.0]
        if outlier_mode == "detect":
            new_rejected = detect_outliers(rows, matrix, bias, coefficient)
        else:
            new_rejected = []
        if new_rejected == rejected:
            break
        rejected = new_rejected
    clean = [row for position, row in enumerate(rows) if position not in rejected]
    if matrix_mode == "fitted" and temperature_mode == "fit":
        matrix, bias, coefficient = fit_joint(clean)
    elif matrix_mode == "fitted":
        matrix, _matrix_bias = fit_matrix_bias(clean)
        bias = mean_residual_bias(clean, matrix)
        coefficient = [0.0, 0.0, 0.0]
    elif temperature_mode == "fit":
        matrix = identity_matrix()
        bias, coefficient = fit_bias_temperature(clean, matrix)
    else:
        matrix = identity_matrix()
        bias = mean_residual_bias(clean, matrix)
        coefficient = [0.0, 0.0, 0.0]
    return matrix, bias, coefficient, rejected


def corrected_rate(raw, temperature, bias, coefficient, inverse):
    centered = temperature - 25.0
    value = [
        raw[axis] - bias[axis] - coefficient[axis] * centered
        for axis in range(3)
    ]
    return matvec(inverse, value)


def run(data):
    matrix, bias, coefficient, rejected = estimate_calibration(
        data["calibration"]["samples"]
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
            bias,
            coefficient,
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
            "bias_temp_coefficient": coefficient,
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
