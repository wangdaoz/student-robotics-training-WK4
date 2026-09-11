import mujoco
import pytest
from src.joint_inventory import build_actuator_index, joint_jsontable, actuator_jsontable, HUMANOID_LITE_PATH, MODEL_PATH

@pytest.fixture(scope = "module")
def model():
    """Load once and reuse across tests -- compiling the model is the
    expensive part, and every test below depends on it succeeding."""
    return mujoco.MjModel.from_xml_path(MODEL_PATH)

def test_free_base_joint_seven_qpos_six_dof(model):
    joints = joint_jsontable(model, build_actuator_index(model))
    matches = [joint for joint in joints if joint["Joint Name"] == "base_freejoint"]
    assert len(matches) == 1, "Expected exactly one joint named 'base_freejoint'"
    joint = matches[0]
    assert joint["Joint Type"] == "0/free (floating base)"
    assert joint["n_qpos"] == 7
    assert joint["n_dof"] == 6
    assert joint["Driven by"] == ["None"]

def test_actuator_forcerange_explicit_no_ctrlrange(model):
    actuators = actuator_jsontable(model)
    for actuator in actuators:
        assert actuator["Force Range Explicitly Defined"] == "Yes"
        assert actuator["Ctrl Range Explicitly Defined"] != "Yes"

def test_every_actuator_drives_a_joint(model):
    joint_to_actuators = build_actuator_index(model)
    actuators = actuator_jsontable(model)
    joints = joint_jsontable(model, joint_to_actuators)
    joint_names = {joint["Joint Name"] for joint in joints}
    for actuator in actuators:
        if actuator["Drives Joint"] != "N/A":
            assert actuator["Drives Joint"] in joint_names

def test_no_unexpected_unactuated_joints(model):
    joint_to_actuators = build_actuator_index(model)
    joints = joint_jsontable(model, joint_to_actuators)
    for joint in joints:
        assert joint["Driven by"] != ["None"] or joint["Joint Type"] == "0/free (floating base)"

# From Claude Code's suggestion, added a test to check that the provenance of force ranges matches the raw XML.
def test_forcerange_provenance_matches_raw_xml(model):
    from src.joint_inventory import build_provenance_map
    expected = build_provenance_map(HUMANOID_LITE_PATH, "forcerange", tag="motor")
    actuators = actuator_jsontable(model)
    for actuator in actuators:
        name = actuator["Actuator Name"]
        if expected.get(name) == "explicit":
            assert actuator["Force Range Explicitly Defined"] == "Yes"

# Improvement taken from Claude Code's suggestion: use pytest.mark.parametrize to test known joint ranges with expected values.
@pytest.mark.parametrize("joint_name,expected_range", [
    ("arm_left_shoulder_pitch_joint", (-1.571,  0.785)),
    ("arm_right_shoulder_pitch_joint", (-0.785,  1.571)),
])
def test_known_joint_ranges(model, joint_name, expected_range):
    joints = joint_jsontable(model, build_actuator_index(model))
    joint = next(j for j in joints if j["Joint Name"] == joint_name)
    lo, hi = (float(x) for x in joint["Joint Range"].split(","))
    assert (lo, hi) == pytest.approx(expected_range, abs=1e-3)

# after check by Claude Code, removed the main() function and the if __name__ == "__main__": block, since pytest will automatically discover and run the test functions in this file.
# def main():
#     model = model()
#     test_free_base_joint_seven_qpos_six_dof(model)
#     test_actuator_forcerange_explicit_no_ctrlrange(model)
#     test_every_actuator_drives_a_joint(model)
#     test_no_unexpected_unactuated_joints(model)
#     test_forcerange_provenance_matches_raw_xml(model)
#     test_known_joint_ranges(model, "<joint_name>", (<lo>, <hi>))

# if __name__ == "__main__":
#     main()