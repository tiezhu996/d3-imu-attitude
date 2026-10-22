#!/usr/bin/env bash
# Install the reference implementation and optionally restore one or more seeded errors.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
install -m 0644 "$HERE/app/main.py" /app/main.py

SKIP="${HTB_SKIP:-}" /usr/local/bin/python3 - /app/main.py <<'PY'
import os
import sys
from pathlib import Path

REVERT = {
    "difficulty_1": (
        "q = quat_normalize(quat_mul(q, step))",
        "q = quat_normalize(quat_mul(step, q))",
    ),
    "difficulty_2": (
        """    first_window = windows[0]
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
        )""",
        """    constant = [float(value) for value in samples[0]["gyro"]]
    bias_start = constant
    bias_end = constant

    def bias_at(_stamp):
        return constant""",
    ),
    "difficulty_3": (
        "        step = quat_exp([omega[0] * dt, omega[1] * dt, omega[2] * dt])",
        """        step = [
            1.0,
            0.5 * omega[0] * dt,
            0.5 * omega[1] * dt,
            0.5 * omega[2] * dt,
        ]""",
    ),
}
skip = [item for item in os.environ.get("SKIP", "").split(",") if item]
unknown = sorted(set(skip) - set(REVERT))
if unknown:
    sys.exit(f"unknown difficulty id: {unknown}")
path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
for key in skip:
    fixed, original = REVERT[key]
    hits = text.count(fixed)
    if hits != 1:
        sys.exit(f"{key} patch matched {hits} times")
    text = text.replace(fixed, original)
path.write_text(text, encoding="utf-8")
PY
chown runner:runner /app/main.py 2>/dev/null || true
