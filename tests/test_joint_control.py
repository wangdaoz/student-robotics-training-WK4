import csv
import json

import pytest

from src.joint_control import EXIT_INVALID_REQUEST, EXIT_OK, main

VALID_JOINTS = [
    ("arm_right_shoulder_pitch_joint", 0.25),
    ("arm_left_elbow_pitch_joint", 0.5),     # starts at its lower limit (0.0)
    ("leg_right_knee_pitch_joint", 0.5),     # starts at its lower limit (0.0)
]


@pytest.mark.parametrize("joint,target", VALID_JOINTS)
def test_three_joints_without_source_changes(tmp_path, joint, target):
    log = tmp_path / "run.csv"
    rc = main(["--joint", joint, "--target", str(target), "--duration", "0.5",
               "--freeze-base", "--log-file", str(log)])
    assert rc == EXIT_OK
    rows = list(csv.DictReader(log.open()))
    assert list(rows[0]) == ["time", "joint", "target", "position", "velocity", "error", "control"]
    assert len(rows) == 251                      # t=0 row + 250 steps of 2 ms
    assert all(r["joint"] == joint for r in rows)
    peak = max(float(r["position"]) for r in rows)
    assert peak >= target                       # joint actually reached the target
    meta = json.loads((tmp_path / "run.csv.meta.json").read_text())
    assert meta["cutoff"].startswith("target reached")


@pytest.mark.parametrize("extra", [
    ["--joint", "not_a_joint"],
    ["--joint", "base_freejoint"],               # free joint: not single-DOF
    ["--target", "1.6"],                         # beyond shoulder pitch upper limit - margin
    ["--target", "nan"],
    ["--control", "25"],                         # exceeds forcerange +/-20
    ["--control", "-1"],
    ["--duration", "0"],
    ["--duration", "inf"],
])
def test_invalid_requests_fail_without_writing_a_log(tmp_path, extra):
    log = tmp_path / "run.csv"
    assert main(extra + ["--log-file", str(log)]) == EXIT_INVALID_REQUEST
    assert not log.exists()


def test_runs_are_reproducible(tmp_path):
    a, b = tmp_path / "a.csv", tmp_path / "b.csv"
    for log in (a, b):
        assert main(["--freeze-base", "--duration", "0.3", "--log-file", str(log)]) == EXIT_OK
    assert a.read_text() == b.read_text()
