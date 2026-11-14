import csv
import json

import pytest

from src.position_controller_revised_version import EXIT_OK, main

HEADER = ["time", "joint", "target", "position", "velocity", "error",
          "raw_control", "applied_control", "saturated"]

VALID_JOINTS = [
    ("arm_right_shoulder_pitch_joint", 0.25),
    ("arm_left_elbow_pitch_joint", 0.5),    # starts at its lower limit (0.0)
    ("leg_right_knee_pitch_joint", 0.5),    # starts at its lower limit (0.0)
]


def run(tmp_path, name, *extra):
    """Run the controller, return (csv rows, meta dict). Fails the test on a non-zero exit."""
    log = tmp_path / f"{name}.csv"
    rc = main(["--log-file", str(log), *extra])
    assert rc == EXIT_OK
    rows = list(csv.DictReader(log.open()))
    meta = json.loads((tmp_path / f"{name}.csv.meta.json").read_text())
    return rows, meta


def col(rows, name):
    return [float(r[name]) for r in rows]


@pytest.mark.parametrize("joint,target", VALID_JOINTS)
def test_three_joints_without_source_changes(tmp_path, joint, target):
    rows, meta = run(tmp_path, "run", "--joint", joint, "--target", str(target),
                     "--duration", "0.5", "--freeze-base")

    assert list(rows[0]) == HEADER
    # 250 rows logged before each 2 ms step (t = 0 ... 0.498) + 1 final row at t = 0.5
    assert len(rows) == 251
    assert float(rows[0]["time"]) == 0.0
    assert float(rows[-1]["time"]) == pytest.approx(0.5)
    assert all(r["joint"] == joint for r in rows)

    # Closed loop moved the joint toward the target (P control may not reach it exactly).
    errors = col(rows, "error")
    assert abs(errors[-1]) < abs(errors[0])

    assert meta["stop_reason"] is None
    assert meta["Kp"] == 5.0                     # default gain is recorded
    assert meta["final_position"] == pytest.approx(float(rows[-1]["position"]))


def test_control_is_bounded_and_saturation_is_logged(tmp_path):
    max_control = 2.0
    rows, meta = run(tmp_path, "sat", "--Kp", "100", "--max-control", str(max_control),
                     "--duration", "0.5", "--freeze-base")

    for r in rows:
        raw, applied = float(r["raw_control"]), float(r["applied_control"])
        assert abs(applied) <= max_control + 1e-12
        assert int(r["saturated"]) == int(raw != applied)
    assert meta["saturated_steps"] > 0          # Kp=100 must actually hit the limit


def test_controller_stays_on_near_the_target(tmp_path):
    """Regression test for version 1, which cut the torque to 0 once |error| <= margin."""
    rows, _ = run(tmp_path, "near", "--Kp", "50", "--max-control", "20",
                  "--duration", "1.0", "--freeze-base")
    near = [r for r in rows if 0 < abs(float(r["error"])) <= 0.05]
    assert near, "joint never came within the margin of the target"
    assert all(float(r["applied_control"]) != 0.0 for r in near)


def test_different_gains_produce_measurable_differences(tmp_path):
    """Higher Kp -> smaller steady-state error against gravity (frozen base, no saturation)."""
    tail_errors = []
    for kp in (5.0, 15.0, 50.0):
        rows, meta = run(tmp_path, f"kp{kp:g}", "--Kp", str(kp), "--target", "0.5",
                         "--max-control", "20", "--duration", "2.0", "--freeze-base")
        assert meta["Kp"] == kp
        last = rows[-251:]                       # last 0.5 s, less sensitive than one sample
        tail_errors.append(sum(abs(float(r["error"])) for r in last) / len(last))
    assert tail_errors[0] > tail_errors[1] > tail_errors[2]


def test_runs_are_reproducible(tmp_path):
    a, _ = run(tmp_path, "a", "--freeze-base", "--duration", "0.3")
    b, _ = run(tmp_path, "b", "--freeze-base", "--duration", "0.3")
    assert a == b
