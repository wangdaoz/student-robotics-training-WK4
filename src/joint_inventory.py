"""
Milestone 1 - Engineering Tasks 1, 2
Inventory Fields:
•	Joint name and MuJoCo joint type.
•	Joint ID, qpos address, and dof/qvel address.
•	Joint range where applicable.
•	Actuator name and actuator ID.
•	Actuator-to-joint transmission mapping.
•	Actuator control-input range (ctrlrange), actuator force/torque range (forcerange), and whether each is explicitly defined or inherited/defaulted.
•	Initial qpos/qvel.
"""
import csv
import json
import os
import time
import mujoco

MODEL_PATH = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", 
             "models/berkeley/Berkeley-Humanoid-Lite-Assets/data/robots/berkeley_humanoid/berkeley_humanoid_lite/mjcf", "bhl_scene.xml"))

OUTPUT_PATH = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "evidence/logs/", "joint_actuator_inventory.csv"))

JOINT_TRANSMISSION_TYPES = {int(mujoco.mjtTrn.mjTRN_JOINT), int(mujoco.mjtTrn.mjTRN_JOINTINPARENT)}

# mjtJoint enum -> (readable label, #qpos slots, #qvel/dof slots)
JOINT_TYPE_INFO = {
    mujoco.mjtJoint.mjJNT_FREE:   ("free (floating base)", 7, 6),
    mujoco.mjtJoint.mjJNT_BALL:   ("ball", 4, 3),
    mujoco.mjtJoint.mjJNT_SLIDE:  ("slide", 1, 1),
    mujoco.mjtJoint.mjJNT_HINGE:  ("hinge", 1, 1),
}

def build_actuator_index(model):

    joint_to_actuators = {}
    for ac_id in range(model.nu):
        if model.actuator_trntype[ac_id] not in JOINT_TRANSMISSION_TYPES:
            continue
        
        j_id = model.actuator_trnid[ac_id, 0]
        ac_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ac_id)
        joint_to_actuators.setdefault(j_id, []).append(ac_name)

    return joint_to_actuators

def joint_table(model, joint_to_actuators):
    with open(OUTPUT_PATH, "w", newline="") as file:
        writer = csv.writer(file)

        # Section header
        writer.writerow(["JOINT INVENTORY"])

        # CSV header
        writer.writerow([
            "Joint Name",
            "Joint Type",
            "Joint ID",
            "qpos_addr n_qpos",
            "dof_addr n_dof",
            "Joint Range",
            "Driven by"
        ])

        for j_id in range(model.njnt):
            joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j_id)
            joint_type = model.jnt_type[j_id]
            label, n_qpos, n_dof = JOINT_TYPE_INFO.get(joint_type, ("unknown", 0, 0))
            qpos_addr = model.jnt_qposadr[j_id]
            qdof_addr = model.jnt_dofadr[j_id]

            if joint_type == mujoco.mjtJoint.mjJNT_FREE:
                joint_range_str = "N/A (free joint)"
            elif model.jnt_limited[j_id]:
                lo, hi = model.jnt_range[j_id]
                joint_range_str = f"{lo: .3f}, {hi: .3f}"
            else:
                joint_range_str = "N/A (unlimited)"

            drivers = ", ".join(joint_to_actuators.get(j_id, ["None"]))

            # Write one CSV row
            writer.writerow([
                joint_name,
                f"{joint_type}/{label}",
                j_id,
                f"{qpos_addr} ({n_qpos} slots)",
                f"{qdof_addr} ({n_dof} slots)",
                joint_range_str,
                drivers
            ])

def actuator_table(model):
    with open(OUTPUT_PATH, "a", newline="") as file:
        writer = csv.writer(file)

        # Section header
        writer.writerow(["ACTUATOR INVENTORY"])

        # CSV header
        writer.writerow([
            "Actuator Name",
            "Actuator Type",
            "Actuator ID",
            "Drives Joint",
            "Joint ID",
            "Force Range (N*m)",
            "Force Range Explicitly Defined",
            "Ctrl Range",
            "Ctrl Range Explicitly Defined"
        ])

        for ac_id in range(model.nu):
            actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ac_id)
            actuator_type = model.actuator_trntype[ac_id]

            if model.actuator_trntype[ac_id] in JOINT_TRANSMISSION_TYPES:
                joint_id = model.actuator_trnid[ac_id, 0]
                joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            else:
                joint_id = None
                joint_name = "N/A"
        
            if model.actuator_forcelimited[ac_id] == True:
                lo_force, hi_force = model.actuator_forcerange[ac_id]
                force_str = f"{lo_force: .3f}, {hi_force: .3f}"
                force_explicit = "Yes"
            else:
                force_str = "Unlimited/Not Defaulted"
                force_explicit = "No"

            if model.actuator_ctrllimited[ac_id] == True:
                lo_ctrl, hi_ctrl = model.actuator_ctrlrange[ac_id]
                ctrl_str = f"{lo_ctrl: .3f}, {hi_ctrl: .3f}"
                ctrl_explicit = "Yes"
            else:
                ctrl_str = "Unlimited/Not Defaulted"
                ctrl_explicit = "No"
        
            # Write one CSV row
            writer.writerow([
                actuator_name,
                actuator_type,
                ac_id,
                joint_name,
                joint_id,
                force_str,
                force_explicit,
                ctrl_str,
                ctrl_explicit
            ])

def main():
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    joint_to_actuators = build_actuator_index(model)
    
    joint_table(model, joint_to_actuators)
    actuator_table(model)

if __name__ == "__main__":
    main()