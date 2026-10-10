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
SINGLE_DOF_JOINT_TYPES = {int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)}

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

# --------------------------------------------------------------------------- validation
def resolve_joint(model, joint_name):
    """Return (joint_id, actuator_id) or raise JointCommandError."""
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id == -1: # must be checked BEFORE using joint_id as an index (-1 = last joint!)
        raise JointCommandError(f"Unknown joint '{joint_name}'.")
    if int(model.jnt_type[joint_id]) not in SINGLE_DOF_JOINT_TYPES:
        raise JointCommandError(
            f"Joint '{joint_name}' is not a single-DOF hinge/slide joint "
            f"(type {mujoco.mjtJoint(model.jnt_type[joint_id]).name}).")

    actuator_names = build_actuator_index(model).get(joint_id, [])
    if not actuator_names:
        raise JointCommandError(f"Joint '{joint_name}' has no actuator (non-actuated joint).")
    if len(actuator_names) > 1:
        raise JointCommandError(f"Joint '{joint_name}' is driven by {len(actuator_names)} actuators {actuator_names}; "
            "this tool supports exactly one actuator per joint.")

    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_names[0])
    gain_type = int(model.actuator_gaintype[act_id])
    bias_type = int(model.actuator_biastype[act_id])
    if gain_type != int(mujoco.mjtGain.mjGAIN_FIXED) or bias_type != int(mujoco.mjtBias.mjBIAS_NONE):
        raise JointCommandError(
            f"Actuator for '{joint_name}' is not a plain motor (gain/bias types "
            f"{gain_type}/{bias_type}); its ctrl is not a torque command.")
    return joint_id, act_id

def validate_target(model, joint_id, joint_name, target, margin):
    if not math.isfinite(target):
        raise JointCommandError(f"Target must be finite, got {target}.")
    if not model.jnt_limited[joint_id]:
        raise JointCommandError(f"Joint '{joint_name}' has no range; refusing to command it.")
    lo, hi = (float(v) for v in model.jnt_range[joint_id])
    if lo >= hi:
        raise JointCommandError(f"Joint '{joint_name}' has an invalid range [{lo}, {hi}].")
    safe_lo, safe_hi = lo + margin, hi - margin
    if not safe_lo <= target <= safe_hi:
        raise JointCommandError(
            f"Target {target} is outside the safe range of '{joint_name}': "
            f"[{safe_lo:.4f}, {safe_hi:.4f}] (joint range [{lo:.4f}, {hi:.4f}], margin {margin}).")
    return safe_lo, safe_hi

def validate_control(model, act_id, control):
    """ctrl and force are different quantities: ctrlrange bounds the input written
    to data.ctrl; forcerange bounds the actuator output, which for a motor is
    gain * gear * ctrl. A range of [0, 0] with *limited == 0 means 'not limited'."""
    if not math.isfinite(control) or control <= 0:
        raise JointCommandError(f"--control must be a positive finite magnitude, got {control}.")

    if model.actuator_ctrllimited[act_id]:
        c_lo, c_hi = model.actuator_ctrlrange[act_id]
        if not (c_lo <= control and control <= c_hi):
            raise JointCommandError(
                f"Control magnitude {control} exceeds ctrlrange [{c_lo}, {c_hi}].")

    if model.actuator_forcelimited[act_id]:
        gain = float(model.actuator_gainprm[act_id, 0])
        gear = float(model.actuator_gear[act_id, 0])
        force = abs(gain * gear * control)
        f_lo, f_hi = model.actuator_forcerange[act_id]
        if not (f_lo <= force and force <= f_hi):
            raise JointCommandError(
                f"Control {control} produces actuator force {force} (gain {gain}, gear {gear}), "
                f"outside forcerange [{f_lo}, {f_hi}]; MuJoCo would silently clamp it.")


def validate_duration(duration):
    if not math.isfinite(duration) or not 0 < duration <= 30:
        raise JointCommandError(f"Duration must be finite and in (0, 30] s, got {duration}.")

# --------------------------------------------------------------------------- experiment

def run(args):
    validate_duration(args.duration)
    if not 0 <= args.margin:
        raise JointCommandError(f"--margin must be >= 0, got {args.margin}.")
 
    model = load_model(args.model, args.freeze_base)
    joint_id, act_id = resolve_joint(model, args.joint)
    safe_lo, safe_hi = validate_target(model, joint_id, args.joint, args.target, args.margin)
    validate_control(model, act_id, args.control)
    # ---- everything below runs only for a fully validated request ----
 
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
 
    qpos_addr = model.jnt_qposadr[joint_id]
    qvel_addr = model.jnt_dofadr[joint_id]
    initial_error = args.target - float(data.qpos[qpos_addr])
    direction = math.copysign(1.0, initial_error) if initial_error != 0 else 0.0
    n_steps = int(round(args.duration / model.opt.timestep))
 
    log_dir = os.path.dirname(os.path.abspath(args.log_file))
    os.makedirs(log_dir, exist_ok=True)
 
    cutoff_reason = None
    position_at_cutoff = None
    with open(args.log_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "joint", "target", "position", "velocity", "error", "control"])
 
        def log_row():
            pos = float(data.qpos[qpos_addr])
            writer.writerow([f"{data.time:.6f}", args.joint, args.target, pos,
                             float(data.qvel[qvel_addr]), args.target - pos,
                             float(data.ctrl[act_id])])
 
        log_row()  # t = 0, before any command
        for _ in range(n_steps):
            pos = float(data.qpos[qpos_addr])
            if cutoff_reason is None:
                if direction == 0 or (args.target - pos) * direction <= 0:
                    cutoff_reason = f"target reached at t={data.time:.3f}s"
                # Only the limit we are pushing toward matters: several joints (elbow, knee)
                # start exactly at a limit, which is fine as long as we push away from it.
                elif (direction > 0 and pos > safe_hi) or (direction < 0 and pos < safe_lo):
                    cutoff_reason = f"safety margin entered at t={data.time:.3f}s"
                if cutoff_reason:
                    position_at_cutoff = pos
            # Open-loop torque toward the target; zeroed by the cutoff. This is NOT position
            # control (that is Milestone 3) -- the joint will coast/sag after the cutoff.
            data.ctrl[act_id] = 0.0 if cutoff_reason else direction * args.control
            mujoco.mj_step(model, data)
            log_row()
 
    final_pos = float(data.qpos[qpos_addr])
    summary = {
        "argv": sys.argv,
        "joint": args.joint,
        "target": args.target,
        "control_magnitude": args.control,
        "duration": args.duration,
        "margin": args.margin,
        "freeze_base": args.freeze_base,
        "model": os.path.abspath(args.model),
        "mujoco_version": mujoco.__version__,
        "timestep": model.opt.timestep,
        "steps": n_steps,
        "joint_range": [float(v) for v in model.jnt_range[joint_id]],
        "final_position": final_pos,
        "final_error": args.target - final_pos,
        "cutoff": cutoff_reason,
        "position_at_cutoff": position_at_cutoff,
    }
    with open(args.log_file + ".meta.json", "w") as f:
        json.dump(summary, f, indent=2)
    return summary

def main(argv=None):
    args = parse_args(argv)
    try:
        summary = run(args)
    except JointCommandError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return EXIT_INVALID_REQUEST
    at_cut = summary["position_at_cutoff"]
    at_cut = "n/a" if at_cut is None else f"{at_cut:+.4f}"
    print(f"{summary['joint']}: target {summary['target']:+.4f}, at cutoff {at_cut}, "
          f"final {summary['final_position']:+.4f} (error {summary['final_error']:+.4f}), "
          f"cutoff: {summary['cutoff']}. Log: {args.log_file}")
    return EXIT_OK
 
 
if __name__ == "__main__":
    sys.exit(main())