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
import math
import xml.etree.ElementTree as ET

MODEL_PATH = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", 
             "models/berkeley/Berkeley-Humanoid-Lite-Assets/data/robots/berkeley_humanoid/berkeley_humanoid_lite/mjcf", "bhl_scene.xml"))

HUMANOID_LITE_PATH = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", 
             "models/berkeley/Berkeley-Humanoid-Lite-Assets/data/robots/berkeley_humanoid/berkeley_humanoid_lite/mjcf", "berkeley_humanoid_lite.xml"))           

OUTPUT_CSVPATH = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "evidence/logs/", "joint_actuator_inventory.csv"))
OUTPUT_JSONPATH = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "evidence/logs/", "joint_actuator_inventory.json"))
OUTPUT_MD_SUMMARY_PATH = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "docs", "joint_actuator_map.md"))

JOINT_TRANSMISSION_TYPES = {int(mujoco.mjtTrn.mjTRN_JOINT), int(mujoco.mjtTrn.mjTRN_JOINTINPARENT)}

# mjtJoint enum -> (readable label, #qpos slots, #qvel/dof slots)
JOINT_TYPE_INFO = {
    mujoco.mjtJoint.mjJNT_FREE:   ("free (floating base)", 7, 6),
    mujoco.mjtJoint.mjJNT_BALL:   ("ball", 4, 3),
    mujoco.mjtJoint.mjJNT_SLIDE:  ("slide", 1, 1),
    mujoco.mjtJoint.mjJNT_HINGE:  ("hinge", 1, 1),
}

def compare_two_numbers_values(a, b):
    if a < b:
        return True
    else:
        return False

def build_provenance_map(xml_path, attr, tag="motor"):
    """Returns {actuator_name: 'explicit' | 'inherited' | 'absent'} for a given attr."""
    tree = ET.parse(xml_path)
    root = tree.getroot()

    class_mp = {}
    def walk_defaults(node, parent = None):
        name = node.get("class")
        if name is not None:
            class_mp[name] = (node, parent)
        for child in node.findall("default"):
            walk_defaults(child, name)
    default_root = root.find("default")
    if default_root is not None:
        walk_defaults(default_root)

    result = {}
    for motor in root.iter(tag):
        name = motor.get("name")
        if name is None:
            continue
        if attr in motor.attrib:
            result[name] = "explicit"
            continue
        cls, source = motor.get("class"), "absent"
        while cls is not None:
            node, parent = class_mp[cls]
            sub = node.find(tag)
            if sub is not None and attr in sub.attrib:
                source = "inherited"
                break
            cls = parent
        result[name] = source
    return result

def build_actuator_index(model):

    joint_to_actuators = {}
    for ac_id in range(model.nu):
        if model.actuator_trntype[ac_id] not in JOINT_TRANSMISSION_TYPES:
            continue
        
        j_id = model.actuator_trnid[ac_id, 0]
        ac_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ac_id)
        joint_to_actuators.setdefault(j_id, []).append(ac_name)

    return joint_to_actuators

def format_slice(values):
    """Format a qpos/qvel slice: a bare number for single-slot joints,
    a comma-separated list for multi-slot joints (e.g. free joints)."""
    values = [float(v) for v in values]
    if len(values) == 1:
        return values[0]
    return ", ".join(f"{v: .3f}" for v in values)

def joint_csvtable(model, joint_to_actuators):
    with open(OUTPUT_CSVPATH, "w", newline="") as file:
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
            "Initial qpos",
            "Initial qvel",
            "Joint Range",
            "Driven by"
        ])
        
        data = mujoco.MjData(model)
        for j_id in range(model.njnt):
            joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j_id)
            joint_type = model.jnt_type[j_id]
            label, n_qpos, n_dof = JOINT_TYPE_INFO.get(int(joint_type), ("unknown", 0, 0))
            qpos_addr = model.jnt_qposadr[j_id]
            qdof_addr = model.jnt_dofadr[j_id]

            qpos_slice = data.qpos[qpos_addr : qpos_addr + n_qpos]
            qvel_slice = data.qvel[qdof_addr : qdof_addr + n_dof]

            initial_qpos = format_slice(qpos_slice)
            initial_qvel = format_slice(qvel_slice)

            if joint_type == mujoco.mjtJoint.mjJNT_FREE:
                joint_range_str = "N/A (free joint)"
            elif model.jnt_limited[j_id]:
                lo, hi = model.jnt_range[j_id]
                # assert compare_two_numbers_values(lo, hi), f"Joint '{joint_name}' has invalid range: {lo} >= {hi}"
                if lo >= hi:
                    raise ValueError(f"Warning: Joint '{joint_name}' has invalid range: {lo} >= {hi}.")
                joint_range_str = f"{lo: .3f}, {hi: .3f}"
                # Reject an accidental full rotation -- usually means degrees
                # were typed where radians were expected (see compiler angle).
                # assert (hi - lo) <= math.pi, (
                #     f"'{joint_name}' range spans more than a full turn: "
                #     f"{lo:.3f} to {hi:.3f} -- check units"
                # )
                if (hi - lo) > math.pi:
                    raise ValueError(f"Warning: '{joint_name}' range spans more than a full turn: {lo:.3f} to {hi:.3f} -- check units.")
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
                initial_qpos,
                initial_qvel,
                joint_range_str,
                drivers
            ])

def joint_jsontable(model, joint_to_actuators):
    joints = []

    data = mujoco.MjData(model)
    for j_id in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j_id)
        joint_type = model.jnt_type[j_id]
        label, n_qpos, n_dof = JOINT_TYPE_INFO.get(int(joint_type), ("unknown", 0, 0))
        qpos_addr = model.jnt_qposadr[j_id]
        qdof_addr = model.jnt_dofadr[j_id]

        qpos_slice = data.qpos[qpos_addr : qpos_addr + n_qpos]
        qvel_slice = data.qvel[qdof_addr : qdof_addr + n_dof]

        initial_qpos = [float(v) for v in qpos_slice]
        initial_qvel = [float(v) for v in qvel_slice]

        if joint_type == mujoco.mjtJoint.mjJNT_FREE:
            joint_range_str = "N/A (free joint)"
        elif model.jnt_limited[j_id]:
            lo, hi = model.jnt_range[j_id]
            #assert compare_two_numbers_values(lo, hi), f"Joint '{joint_name}' has invalid range: {lo} >= {hi}"
            if lo >= hi:
                    raise ValueError(f"Warning: Joint '{joint_name}' has invalid range: {lo} >= {hi}.")
            joint_range_str = f'{lo: .3f}, {hi: .3f}'
            # Reject an accidental full rotation -- usually means degrees
            # were typed where radians were expected (see compiler angle).
            # assert (hi - lo) <= math.pi, (
            #     f"'{joint_name}' range spans more than a full turn: "
            #     f"{lo:.3f} to {hi:.3f} -- check units"
            # )
            if (hi - lo) > math.pi:
                    raise ValueError(f"Warning: '{joint_name}' range spans more than a full turn: {lo:.3f} to {hi:.3f} -- check units.")
        else:
            joint_range_str = "N/A (unlimited)"

        drivers = joint_to_actuators.get(j_id, ["None"])

        joints.append({
            "Joint Name": joint_name,
            "Joint Type": f"{joint_type}/{label}",
            "Joint ID": j_id,
            "qpos_addr": int(qpos_addr),
            "n_qpos": n_qpos,
            "dof_addr": int(qdof_addr),
            "n_dof": n_dof,
            "Initial qpos": initial_qpos,
            "Initial qvel": initial_qvel,
            "Joint Range": joint_range_str,
            "Driven by": drivers
        })
    
    """ check the duplicate joint names in the joints list """
    non_duplicatedjoint_names_set = set()
    duplicate_joint_names_set = set()
    for joint in joints:
        joint_name = joint["Joint Name"]
        if joint_name in non_duplicatedjoint_names_set:
            duplicate_joint_names_set.add(joint_name)
            print(f"Warning: Duplicate joint name found: {joint_name}. This may indicate a problem in the model or the inventory generation process!")
        else:
            non_duplicatedjoint_names_set.add(joint_name)

    return joints

def actuator_csvtable(model):
    with open(OUTPUT_CSVPATH, "a", newline="") as file:
        writer = csv.writer(file)

        # Section header
        writer.writerow(["ACTUATOR INVENTORY"])

        # CSV header
        writer.writerow([
            "Actuator Name",
            "Actuator ID",
            "Drives Joint",
            "Joint ID",
            "Force Range (N*m)",
            "Force Range Explicitly Defined",
            "Ctrl Range",
            "Ctrl Range Explicitly Defined"
        ])
        
        force_results = build_provenance_map(HUMANOID_LITE_PATH, "forcerange", tag="motor")
        ctrl_results = build_provenance_map(HUMANOID_LITE_PATH, "ctrlrange", tag="motor")
        for ac_id in range(model.nu):
            actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ac_id)

            if model.actuator_trntype[ac_id] in JOINT_TRANSMISSION_TYPES:
                joint_id = model.actuator_trnid[ac_id, 0]
                joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            else:
                joint_id = None
                joint_name = "N/A"
                print(f"Warning: Actuator '{actuator_name}' (ID {ac_id}) has non-joint transmission type {model.actuator_trntype[ac_id]}.")
            
            if model.actuator_forcelimited[ac_id] == True:
                lo_force, hi_force = model.actuator_forcerange[ac_id]
                # assert compare_two_numbers_values(lo_force, hi_force), f"Actuator '{actuator_name}' has invalid force range: {lo_force} >= {hi_force}"
                if lo_force >= hi_force:
                    raise ValueError(f"Warning: Actuator '{actuator_name}' has invalid force range: {lo_force} >= {hi_force}.")
                force_str = f"{lo_force: .3f}, {hi_force: .3f}"
            else:
                force_str = "Unlimited/Not Defaulted"
            force_explicit = "Yes" if force_results.get(actuator_name) == "explicit" else f"No, {force_results.get(actuator_name)}"

               
            if model.actuator_ctrllimited[ac_id] == True:
                lo_ctrl, hi_ctrl = model.actuator_ctrlrange[ac_id]
                # assert compare_two_numbers_values(lo_ctrl, hi_ctrl), f"Actuator '{actuator_name}' has invalid ctrl range: {lo_ctrl} >= {hi_ctrl}"
                if not lo_ctrl >= hi_ctrl:
                    raise ValueError(f"Warning: Actuator '{actuator_name}' has invalid ctrl range: {lo_ctrl} >= {hi_ctrl}.")
                ctrl_str = f"{lo_ctrl: .3f}, {hi_ctrl: .3f}"
            else:
                ctrl_str = "Unlimited/Not Defaulted"
            ctrl_explicit = "Yes" if ctrl_results.get(actuator_name) == "explicit" else f"No, {ctrl_results.get(actuator_name)}"
        
            # Write one CSV row
            writer.writerow([
                actuator_name,
                ac_id,
                joint_name,
                joint_id,
                force_str,
                force_explicit,
                ctrl_str,
                ctrl_explicit
            ])

def actuator_jsontable(model):
    actuators = []
    
    force_results = build_provenance_map(HUMANOID_LITE_PATH, "forcerange", tag="motor")
    ctrl_results = build_provenance_map(HUMANOID_LITE_PATH, "ctrlrange", tag="motor")
    for ac_id in range(model.nu):
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ac_id)

        if model.actuator_trntype[ac_id] in JOINT_TRANSMISSION_TYPES:
            joint_id = model.actuator_trnid[ac_id, 0]
            joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        else:
            joint_id = None
            joint_name = "N/A"
            print(f"Warning: Actuator '{actuator_name}' (ID {ac_id}) has non-joint transmission type {model.actuator_trntype[ac_id]}.")
        
        
        if model.actuator_forcelimited[ac_id] == True:
            lo_force, hi_force = model.actuator_forcerange[ac_id]
            # assert compare_two_numbers_values(lo_force, hi_force), f"Actuator '{actuator_name}' has invalid force range: {lo_force} >= {hi_force}"
            if lo_force >= hi_force:
                raise ValueError(f"Warning: Actuator '{actuator_name}' has invalid force range: {lo_force} >= {hi_force}.")
            force_range_str = f"{lo_force: .3f}, {hi_force: .3f}"
        else:
            force_range_str = "Unlimited/Not Defaulted"
        force_explicit = "Yes" if force_results.get(actuator_name) == "explicit" else f"No, {force_results.get(actuator_name)}"

        
        if model.actuator_ctrllimited[ac_id] == True:
            lo_ctrl, hi_ctrl = model.actuator_ctrlrange[ac_id]
            # assert compare_two_numbers_values(lo_ctrl, hi_ctrl), f"Actuator '{actuator_name}' has invalid ctrl range: {lo_ctrl} >= {hi_ctrl}"
            if lo_ctrl >= hi_ctrl:
                raise ValueError(f"Warning: Actuator '{actuator_name}' has invalid ctrl range: {lo_ctrl} >= {hi_ctrl}.")
            ctrl_range_str = f"{lo_ctrl: .3f}, {hi_ctrl: .3f}"
        else:
            ctrl_range_str = "Unlimited/Not Defaulted"
        ctrl_explicit = "Yes" if ctrl_results.get(actuator_name) == "explicit" else f"No, {ctrl_results.get(actuator_name)}"

        actuators.append({
            "Actuator Name": actuator_name,
            "Actuator ID": ac_id,
            "Drives Joint": joint_name,
            "Joint ID": (int(joint_id) if joint_id is not None else None),
            "Force Range (N*m)": force_range_str,
            "Force Range Explicitly Defined": force_explicit,
            "Ctrl Range": ctrl_range_str,
            "Ctrl Range Explicitly Defined": ctrl_explicit
        })

    """ check the duplicate actuator names in the actuators list """
    non_duplicatedactuator_names_set = set()
    duplicate_actuator_names_set = set()
    for actuator in actuators:
        actuator_name = actuator["Actuator Name"]
        if actuator_name in non_duplicatedactuator_names_set:
            duplicate_actuator_names_set.add(actuator_name)
            print(f"Warning: Duplicate actuator name found: {actuator_name}. This may indicate a problem in the model or the inventory generation process!")
        else:
            non_duplicatedactuator_names_set.add(actuator_name)

    return actuators

def write_markdown_summary(joints, actuators, output_path):
    with open(output_path, "w", encoding="utf-8") as file:
        file.write("# Joint and Actuator Inventory Summary\n\n")
        file.write(f"Generated on: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")

        file.write("## Joints\n\n")
        for joint in joints:
            file.write(f"### {joint['Joint Name']}\n")
            file.write(f"- Type: {joint['Joint Type']}\n")
            file.write(f"- ID: {joint['Joint ID']}\n")
            file.write(f"- qpos_addr: {joint['qpos_addr']} ({joint['n_qpos']} slots)\n")
            file.write(f"- dof_addr: {joint['dof_addr']} ({joint['n_dof']} slots)\n")
            file.write(f"- Initial qpos: {[f'{v:.3f}' for v in joint['Initial qpos']]}\n")
            file.write(f"- Initial qvel: {[f'{v:.3f}' for v in joint['Initial qvel']]}\n")
            file.write(f"- Joint Range: {joint['Joint Range']}\n")
            file.write(f"- Driven by: {','.join(joint['Driven by'])}\n\n")

        file.write("## Actuators\n\n")
        for actuator in actuators:
            file.write(f"### {actuator['Actuator Name']}\n")
            file.write(f"- ID: {actuator['Actuator ID']}\n")
            file.write(f"- Drives Joint: {actuator['Drives Joint']}\n")
            file.write(f"- Joint ID: {actuator['Joint ID']}\n")
            file.write(f"- Force Range (N*m): {actuator['Force Range (N*m)']}\n")
            file.write(f"- Force Range Explicitly Defined: {actuator['Force Range Explicitly Defined']}\n")
            file.write(f"- Ctrl Range: {actuator['Ctrl Range']}\n")
            file.write(f"- Ctrl Range Explicitly Defined: {actuator['Ctrl Range Explicitly Defined']}\n\n")

def main():
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    joint_to_actuators = build_actuator_index(model)
    
    
    #Task #1, #2: Generate CSV inventory of joints and actuators
    joint_csvtable(model, joint_to_actuators)
    actuator_csvtable(model)
    

    joints = joint_jsontable(model, joint_to_actuators)
    actuators = actuator_jsontable(model)
    
    # Task #2: Generate JSON inventory of joints and actuators
    json_inventory = {
        "joints": joints,
        "actuators": actuators
    }

    with open(OUTPUT_JSONPATH, "w", encoding="utf-8") as file:
        json.dump(json_inventory, file, indent=4)

    print(f"JSON inventory written to: {OUTPUT_JSONPATH}")

    """
    Task #3: Generate Markdown summary of joints and actuators
    """
    write_markdown_summary(joints, actuators, OUTPUT_MD_SUMMARY_PATH)
    print(f"Markdown Summary written to: {OUTPUT_MD_SUMMARY_PATH}")

if __name__ == "__main__":
    main()