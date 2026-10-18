import csv
import json

import pytest

from src.position_controller import EXIT_INVALID_REQUEST, EXIT_OK, main

VALID_JOINTS = [
    ("arm_right_shoulder_pitch_joint", 0.25),
    ("arm_left_elbow_pitch_joint", 0.5),    # starts at its lower limit (0.0)
    ("leg_right_knee_pitch_joint", 0.5),    # starts at its lower limit (0.0)
]


@pytest.mark.parametrize("joint,target", VALID_JOINTS)
def test_three_joints_without_source_changes(tmp_path, joint, target):
    log = tmp_path / "Mile3_Experiment.csv"
    rc = main(["--joint", joint, "--target", str(target), "--duration", "0.5",
               "--freeze-base", "--log-file", str(log)])
    assert rc == EXIT_OK
    rows = list(csv.DictReader(log.open()))
    assert list(rows[0]) == ["time", "joint", "target", "position", "velocity", "error", "control"]
    assert len(rows) == 251                      # t=0 row + 250 steps of 2 ms
    assert all(r["joint"] == joint for r in rows)
    peak = max(float(r["position"]) for r in rows)
    assert peak >= target                     # joint actually reached the target
    meta = json.loads((tmp_path / "Mile3_Experiment.csv.meta.json").read_text())
    meta["cutoff"].startswith("target reached")

@pytest.mark.parametrize("Proportional_Gain", [
    ["--Kp", "5.0"],
   ["--Kp", "10.0"],
   ["--Kp", "15.0"],
])
def test_three_joints_with_different_proportional_gain(tmp_path, Proportional_Gain):
    log = tmp_path / "Mile3_Experiment.csv"
    assert main(["--joint", "arm_right_shoulder_pitch_joint", "--target", "1.0", "--control", "1.0","--duration", "2.0",
                 "--log-file", str(log)]+ Proportional_Gain) == EXIT_OK

def test_runs_are_reproducible(tmp_path):
    a, b = tmp_path / "a.csv", tmp_path / "b.csv"
    for log in (a, b):
        assert main(["--freeze-base", "--duration", "0.3", "--log-file", str(log)]) == EXIT_OK
    assert a.read_text() == b.read_text()