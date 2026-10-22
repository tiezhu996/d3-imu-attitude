#!/usr/bin/env bash
# Install the reference implementation and optionally restore one or more seeded estimation modes.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
install -m 0644 "$HERE/app/main.py" /app/main.py

SKIP="${HTB_SKIP:-}" /usr/local/bin/python3 - /app/main.py <<'PY'
import os
import sys
from pathlib import Path

REVERT = {
    "difficulty_1": (
        '    matrix_mode = "fitted"',
        '    matrix_mode = "identity"',
    ),
    "difficulty_2": (
        '    temperature_mode = "fit"',
        '    temperature_mode = "mean"',
    ),
    "difficulty_3": (
        '    outlier_mode = "detect"',
        '    outlier_mode = "none"',
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
