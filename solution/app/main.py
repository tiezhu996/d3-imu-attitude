#!/usr/bin/env python3
"""Replay a short inertial run using a robust calibration fit and raw gyro samples."""

import json
import math
import sys
from decimal import Decimal, localcontext

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
    with localcontext() as context:
        context.prec = 60
        a, b, c = [Decimal(str(value)) for value in matrix[0]]
        d, e, f = [Decimal(str(value)) for value in matrix[1]]
        g, h, i = [Decimal(str(value)) for value in matrix[2]]
        determinant = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
        if determinant == 0:
            raise ValueError("singular calibration matrix")
        inverse = [
            [(e * i - f * h) / determinant, (c * h - b * i) / determinant, (b * f - c * e) / determinant],
            [(f * g - d * i) / determinant, (a * i - c * g) / determinant, (c * d - a * f) / determinant],
            [(d * h - e * g) / determinant, (b * g - a * h) / determinant, (a * e - b * d) / determinant],
        ]
        return [[float(value) for value in row] for row in inverse]


def least_squares(features, target):
    rows = len(features)
    columns = len(features[0])
    with localcontext() as context:
        context.prec = 60
        work = [
            [Decimal(str(features[row][column])) for column in range(columns)]
            + [Decimal(str(target[row]))]
            for row in range(rows)
        ]
        tiny = Decimal("1e-50")
        for active in range(columns):
            tail = [work[row][active] for row in range(active, rows)]
            norm = sum(value * value for value in tail).sqrt()
            if norm < tiny:
                raise ValueError("rank deficient design")
            alpha = -norm if not tail[0] < 0 else norm
            vector = tail[:]
            vector[0] -= alpha
            vector_norm = sum(value * value for value in vector).sqrt()
            if vector_norm < tiny:
                continue
            vector = [value / vector_norm for value in vector]
            for column in range(active, columns + 1):
                dot = sum(
                    vector[index] * work[active + index][column]
                    for index in range(rows - active)
                )
                for index in range(rows - active):
                    work[active + index][column] -= Decimal(2) * vector[index] * dot
        result = [Decimal(0)] * columns
        for row in range(columns - 1, -1, -1):
            pivot = work[row][row]
            if abs(pivot) < tiny:
                raise ValueError("rank deficient triangular system")
            rhs = work[row][columns] - sum(
                work[row][column] * result[column]
                for column in range(row + 1, columns)
            )
            result[row] = rhs / pivot
        return [float(value) for value in result]


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


def estimate_calibration(rows, nominal_matrix):
    rejected = []
    matrix_mode = "fitted"
    temperature_mode = "fit"
    outlier_mode = "detect"
    matrix = [[float(value) for value in row] for row in nominal_matrix]
    bias = [0.0, 0.0, 0.0]
    coefficient = [0.0, 0.0, 0.0]
    for _ in range(6):
        clean = [row for position, row in enumerate(rows) if position not in rejected]
        temperature_span = max(row["temperature_c"] for row in clean) - min(row["temperature_c"] for row in clean)
        if matrix_mode == "fitted" and temperature_mode == "fit" and not temperature_span < 1.0e-9:
            matrix, bias, coefficient = fit_joint(clean)
        elif matrix_mode == "fitted":
            matrix, _matrix_bias = fit_matrix_bias(clean)
            bias = mean_residual_bias(clean, matrix)
            coefficient = [0.0, 0.0, 0.0]
        elif temperature_mode == "fit":
            matrix = [[float(value) for value in row] for row in nominal_matrix]
            bias, coefficient = fit_bias_temperature(clean, matrix)
        else:
            matrix = [[float(value) for value in row] for row in nominal_matrix]
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
    temperature_span = max(row["temperature_c"] for row in clean) - min(row["temperature_c"] for row in clean)
    if matrix_mode == "fitted" and temperature_mode == "fit" and not temperature_span < 1.0e-9:
        matrix, bias, coefficient = fit_joint(clean)
    elif matrix_mode == "fitted":
        matrix, _matrix_bias = fit_matrix_bias(clean)
        bias = mean_residual_bias(clean, matrix)
        coefficient = [0.0, 0.0, 0.0]
    elif temperature_mode == "fit":
        matrix = [[float(value) for value in row] for row in nominal_matrix]
        bias, coefficient = fit_bias_temperature(clean, matrix)
    else:
        matrix = [[float(value) for value in row] for row in nominal_matrix]
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
        if not dt > 0.0:
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
