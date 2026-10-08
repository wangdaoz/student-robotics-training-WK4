"""
Generalize the Week 3 hard-coded shoulder experiment into a command-line tool that can safely operate different Berkeley actuated 
joints.
"""

import argparse
import csv
import json
import math
import os
import time
import sys

import mujoco
import numpy as np

MODEL_PATH = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", 
                            "models/berkeley/Berkeley-Humanoid-Lite-Assets/data/robots/berkeley_humanoid/berkeley_humanoid_lite/mjcf", 
                            "bhl_scene.xml"))

JOINT_TRANSMISSION_TYPES = {int(mujoco.mjtTrn.mjTRN_JOINT), int(mujoco.mjtTrn.mjTRN_JOINTINPARENT)}

EXIT_OK = 0
EXIT_INVALID_REQUEST = 2

class JointCommandError(Exception):
    """An invalid request: rejected before any simulation or logging happens."""

def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, 
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--joint", type = str, default = "arm_right_shoulder_pitch_joint",
        help = "the target joint to control"
               "the default target joint is arm_right_shoulder_pitch_joint."
    )
    parser.add_argument(
        "--target", type = float, default = 0.25,
        help = "Target joint position (rad for hinge, m for slide). "
                "Must lie inside the joint range minus --margin."
    )
    parser.add_argument(
        "--control", type=float, default=1.0,
        help="Magnitude of the actuator control input (for this model's "
             "<motor> actuators: N*m, since gear=1). The sign is chosen "
             "toward the target. Validated against ctrlrange (if limited) "
             "and against forcerange via gear (if limited)."
    )
    parser.add_argument(
        "--duration", type=float, default=2.0,
        help="Simulated duration in seconds (finite, 0 < d <= 30).",
    )
    parser.add_argument("--margin", type=float, default=0.05,
                        help="Safety margin from each joint limit. Targets inside the margin "
                             "are rejected; the command is cut to zero if the joint enters it.")
    parser.add_argument(
        "--freeze-base", action="store_true",
        help="Weld the floating base to the world so only the selected "
             "joint moves, isolating the measurement from the robot "
             "free-falling under gravity. Documented, code-only change -- "
             "does not modify the official MJCF file on disk.",
    )
    parser.add_argument("--model", default=MODEL_PATH, help="Path to the MJCF scene.")
    parser.add_argument(
        "--log-file", type=str, default="evidence/logs/time-series_state.csv",
        help="Where to write the before/after numerical evidence(CSV).",
    )

    return parser.parse_args(argv)

# --------------------------------------------------------------------------- model loading

def load_model(model_path, freeze_base):
    if not os.path.isfile(model_path):
        raise JointCommandError(f"Model file not found: {model_path}")
    if not freeze_base:
        return mujoco.MjModel.from_xml_path(model_path)
 
    spec = mujoco.MjSpec.from_file(model_path)
    free_joints = [j for j in spec.joints if j.type == mujoco.mjtJoint.mjJNT_FREE]
    if not free_joints:
        raise JointCommandError("--freeze-base requested, but the model has no free joint.")
    for joint in free_joints:
        spec.delete(joint)  # body with no joint is welded to its parent (the world)
    return spec.compile()

def build_actuator_index(model):
    """Reverse map: joint_id -> list of actuator names driving it.
    Built by scanning every actuator once -- MuJoCo only stores the
    actuator->joint direction natively, so the reverse direction has
    to be assembled here, not assumed from naming conventions."""

    joint_to_actuators = {}
    for ac_id in range(model.nu):
        if int(model.actuator_trntype[ac_id]) not in JOINT_TRANSMISSION_TYPES:
            continue
        
        j_id = model.actuator_trnid[ac_id, 0]
        ac_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ac_id)
        joint_to_actuators.setdefault(j_id,[]).append(ac_name)

    return joint_to_actuators

def main():
    args = parse_args()
    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)

    joint_to_actuators = build_actuator_index(model)

    with open(args.log_file, "w", newline = "") as file:
        writer = csv.writer(file)
        # CSV header
        writer.writerow(["time", "joint", "target", "position", "velocity", "error", "control"])

        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, args.joint)
        actuator_names = joint_to_actuators.get(joint_id, [])

        qpos_addr = model.jnt_qposadr[joint_id]
        qvel_addr = model.jnt_dofadr[joint_id]

        if joint_id == -1:
            writer.writerow([time.strftime("%Y-%m-%dT%H:%M:%S"), 
                           args.joint,
                           args.target,
                           data.qpos[qpos_addr],
                           data.qvel[qvel_addr],
                           f"RuntimeError: Joint '{args.joint}' not found in compiled model.",
                           data.ctrl[act_id]
                           ])
            raise RuntimeError(f"Joint '{args.joint}' not found in compiled model.")
        
            return -1
        if not actuator_names:
            writer.writerow([time.strftime("%Y-%m-%dT%H:%M:%S"), 
                           args.joint,
                           args.target,
                           data.qpos[qpos_addr],
                           data.qvel[qvel_addr],
                           f"RuntimeError: No actuators found for joint '{args.joint}' in compiled model.",
                           data.ctrl[act_id]
                           ])
            raise RuntimeError(f"No actuators found for joint '{args.joint}' in compiled model.")
            return -1
    
        
        joint_range = model.jnt_range[joint_id].copy()  # Copy to avoid modifying the model's internal data

        if joint_range[0] >= joint_range[1]:
            writer.writerow([time.strftime("%Y-%m-%dT%H:%M:%S"), 
                           args.joint,
                           args.target,
                           data.qpos[qpos_addr],
                           data.qvel[qvel_addr],
                           f"RuntimeError: Joint '{args.joint}' has invalid range: {joint_range}!",
                           data.ctrl[act_id],
                           data.ctrl[act_id]
                           ])
            raise RuntimeError(f"Joint '{args.joint}' has invalid range: {joint_range}!")
            return -1

        n_actuators = len(actuator_names)
        average_torque = args.target / n_actuators
        for actuator_name in actuator_names:
            ac_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
            actuator_range = model.actuator_forcerange[ac_id]
            if not (actuator_range[0] <= average_torque <= actuator_range[1]):
                writer.writerow([time.strftime("%Y-%m-%dT%H:%M:%S"), 
                           args.joint,
                           args.target,
                           data.qpos[qpos_addr],
                           data.qvel[qvel_addr],
                           f"RuntimeError: Actuator '{actuator_name}' with range {actuator_range} N*m cannot handle torque {average_torque} N*m!",
                           data.ctrl[act_id]
                           ])
                raise RuntimeError(f"Actuator '{actuator_name}' cannot handle torque {average_torque} N*m;"
                                   f"range is {actuator_range} N*m.")
            
                return -1

        if args.freeze_base:
            base_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
            model.body_mocapid[base_body_id] = -1  # no-op safeguard, base stays a free body
            model.opt.gravity[:] = 0.0  # simplest isolation: remove gravity entirely

        mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data) # Using this model and the current state in data, recalculate MuJoCo's derived quantities
   
        for actuator_name in actuator_names:
            act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
            data.ctrl[act_id] = average_torque

        average_n_steps = int(args.duration / model.opt.timestep / n_actuators)

        for actuator in actuator_names:
            for step in range(average_n_steps):
                mujoco.mj_step(model, data)
                writer.writerow([time.strftime("%Y-%m-%dT%H:%M:%S"), 
                           args.joint,
                           args.target,
                           data.qpos[qpos_addr],
                           data.qvel[qvel_addr],
                           "None",
                           data.ctrl[act_id]
                           ]) 
        
        final_qpos = float(data.qpos[qpos_addr])
        final_qvel = float(data.qvel[qvel_addr])
        writer.writerow([time.strftime("%Y-%m-%dT%H:%M:%S"), 
                           args.joint,
                           args.target,
                           final_qpos,
                           final_qvel,
                           "None",
                           data.ctrl[act_id]
                        ])

if __name__ == "__main__":
    main()