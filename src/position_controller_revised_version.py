"""
Stop using constant torque until the joint reaches a limit.
Use measured joint state to drive a selected joint toward 
a target position.
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
        "--Kp", type=float, default=5.0,
        help="The proportional gain for the position controller."
    )
    parser.add_argument(
        "--max-control", type=float, default=10.0,
        help="Saturation limit: the command sent to the actuator is clipped to "
             "[-max_control, +max_control] (N*m for this model's motors). "
             "Validated against ctrlrange (if limited) and forcerange via gear (if limited)."
    )
    parser.add_argument(
        "--duration", type=float, default=2.0,
        help="Simulated duration in seconds (finite, 0 < d <= 30).",
    )
    parser.add_argument("--margin", type=float, default=0.05,
                        help="Safety margin from each joint limit. Targets inside the margin "
                             "if the joint enters the margin during the run it is recorded as a warning "
                             "(the command is not cut, because P control already pushes away from the limit)."
    )
    parser.add_argument(
        "--freeze-base", action="store_true",
        help="Weld the floating base to the world so only the selected "
             "joint moves, isolating the measurement from the robot "
             "free-falling under gravity. Documented, code-only change -- "
             "does not modify the official MJCF file on disk.",
    )
    parser.add_argument("--model", default=MODEL_PATH, help="Path to the MJCF scene.")
    parser.add_argument(
        "--log-file", type=str, default=None,
        help="Where to write the before/after numerical evidence(CSV)."
             "Default: evidence/logs/mile3_<joint>_t<target>_kp<Kp>.csv, "
             "so runs with different gains do not overwrite each other.",
    )

    return parser.parse_args(argv)

# ------------------------------------------------ Model loading

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
    # Note: biasprm may hold leftover values from <default> classes; MuJoCo ignores
    # them when biastype is NONE, so the type fields are what decide this.
    if gain_type != int(mujoco.mjtGain.mjGAIN_FIXED) or bias_type != int(mujoco.mjtBias.mjBIAS_NONE):
        raise JointCommandError(
            f"Actuator for '{joint_name}' is not a plain motor (gain/bias types "
            f"{gain_type}/{bias_type}); its ctrl is not a torque command."
            "Kp * error cannot be written into it.")
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

def validate_max_control(model, act_id, max_control):
    """The controller may output anything in [-max_control, +max_control], so BOTH
    ends must be legal. ctrlrange bounds the value written to data.ctrl; forcerange
    bounds the actuator output, which for a motor is gain * gear * ctrl.
    A range with *limited == 0 means 'not limited'."""
    if not math.isfinite(max_control) or max_control <= 0:
        raise JointCommandError(f"--max-control must be a positive finite magnitude, got {max_control}.")

    if model.actuator_ctrllimited[act_id]:
        c_lo, c_hi = (float(v) for v in model.actuator_ctrlrange[act_id])
        if not (c_lo <= -max_control and max_control <= c_hi):
            raise JointCommandError(
                f"--max-control {max_control} allows commands outside ctrlrange [{c_lo}, {c_hi}].")

    if model.actuator_forcelimited[act_id]:
        gain = float(model.actuator_gainprm[act_id, 0])
        gear = float(model.actuator_gear[act_id, 0])
        force = abs(gain * gear * max_control)
        f_lo, f_hi = (float(v) for v in model.actuator_forcerange[act_id])
        if not (f_lo <= -force and force <= f_hi):
            raise JointCommandError(
                f"--max-control {max_control} produces actuator force up to {force} "
                f"(gain {gain}, gear {gear}), outside forcerange [{f_lo}, {f_hi}]; "
                "MuJoCo would silently clamp it and the log would not show the real torque.")

def validate_proportional_gain(Kp):
    if not math.isfinite(Kp) or Kp <= 0:
        raise JointCommandError(f"--Kp must be a positive finite gain value, got {Kp}.")

def validate_duration(duration):
    if not math.isfinite(duration) or not 0 < duration <= 30:
        raise JointCommandError(f"Duration must be finite and in (0, 30] s, got {duration}.")

def validate_margin(margin):
    if not math.isfinite(margin) or margin < 0:
        raise JointCommandError(f"--margin must be a finite value >= 0, got {margin}.")

# --------------------------------------------------------------------------- experiment

def classify(model, actuator_id):
    """Return (type_name, meaning_of_ctrl, kp, kv) for actuator i."""
    gaintype = model.actuator_gaintype[actuator_id]
    biastype = model.actuator_biastype[actuator_id]
    gain = model.actuator_gainprm[actuator_id, 0]
    b0, b1, b2 = model.actuator_biasprm[actuator_id, :3]
 
    if gaintype != mujoco.mjtGain.mjGAIN_FIXED:
        return "other (gaintype is not FIXED)", "see MuJoCo docs", None, None
 
    if biastype == mujoco.mjtBias.mjBIAS_NONE:
        return "motor", "torque/force (multiplied by gear)", None, None
 
    if biastype == mujoco.mjtBias.mjBIAS_AFFINE:
        # position servo:  gain = kp, bias = -kp*q - kv*qvel
        if b1 != 0 and abs(b1 + gain) < 1e-9:
            return "position servo", "target joint position (rad)", gain, -b2
        # velocity servo:  gain = kv, bias = -kv*qvel
        if b1 == 0 and abs(b2 + gain) < 1e-9:
            return "velocity servo", "target joint velocity (rad/s)", None, gain
 
    return "general (custom)", "read gainprm/biasprm yourself", None, None

def run(args):
    validate_margin(args.margin)
    validate_proportional_gain(args.Kp)
    validate_duration(args.duration)
    model = load_model(args.model, args.freeze_base)
    joint_id, act_id = resolve_joint(model, args.joint)
    safe_lo, safe_hi = validate_target(model, joint_id, args.joint, args.target, args.margin)
    validate_max_control(model, act_id, args.max_control)
# ---- everything below runs only for a fully validated request ----

    if args.log_file is None:
        args.log_file = os.path.join(
            "evidence", "logs", f"mile3_{args.joint}_t{args.target:g}_kp{args.Kp:g}.csv")

    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

    qpos_addr = model.jnt_qposadr[joint_id]
    qvel_addr = model.jnt_dofadr[joint_id]
    initial_pos = float(data.qpos[qpos_addr])
    n_steps = int(round(args.duration / model.opt.timestep))
    
    os.makedirs(os.path.dirname(os.path.abspath(args.log_file)), exist_ok=True)

    # Limit-margin handling: the target is inside the safe range, so whenever the
    # joint is in the margin, Kp*error already pushes it AWAY from the limit.
    # Cutting the command there would remove the restoring torque, so we only
    # record it (a joint may also legitimately START in the margin, e.g. at 0).
    margin_warning = None
    if not safe_lo <= initial_pos <= safe_hi:
        margin_warning = f"joint started inside the limit margin (position {initial_pos:.4f})"
    left_margin = margin_warning is None
    stop_reason = None     # set if the simulation becomes non-finite
    saturated_steps = 0 

    def compute_control(pos, vel):
        """P control with saturation. Returns (raw, applied, saturated)."""
        nonlocal margin_warning, left_margin, stop_reason
        if not (math.isfinite(pos) and math.isfinite(vel)):
            if stop_reason is None:
                stop_reason = f"non-finite state at t={data.time:.3f}s (simulation unstable)"
            return 0.0, 0.0, False
        inside = safe_lo <= pos <= safe_hi
        if inside:
            left_margin = True
        elif left_margin and margin_warning is None:
            margin_warning = (f"joint entered the limit margin at t={data.time:.3f}s "
                              f"(position {pos:.4f}, safe range [{safe_lo:.4f}, {safe_hi:.4f}])")
        raw = args.Kp * (args.target - pos)
        applied = float(np.clip(raw, -args.max_control, args.max_control))
        return raw, applied, applied != raw

    with open(args.log_file, "w", newline="") as f:
        writer = csv.writer(f)
        # Each row: the state measured at `time` and the control computed from that
        # state, which is then applied during the following timestep.
        writer.writerow(["time", "joint", "target", "position", "velocity", "error", "raw_control", "applied_control", "saturated"])
 
        def step_and_log(do_step):
            nonlocal saturated_steps
            pos = float(data.qpos[qpos_addr])
            vel = float(data.qvel[qvel_addr])
            raw, applied, saturated = compute_control(pos, vel)
            saturated_steps += saturated
            data.ctrl[act_id] = applied
            writer.writerow([f"{data.time:.6f}", args.joint, args.target, pos, vel,
                             args.target - pos, raw, applied, int(saturated)])
            if do_step and stop_reason is None:
                mujoco.mj_step(model, data)
        
        for _ in range(n_steps):
            step_and_log(do_step=True)
            if stop_reason is not None:
                break
        else:
            step_and_log(do_step=False)  # final measured state
    
    final_pos = float(data.qpos[qpos_addr])
    summary = {
        "argv": sys.argv,
        "joint": args.joint,
        "target": args.target,
        "Kp": args.Kp,
        "max_control": args.max_control,
        "duration": args.duration,
        "margin": args.margin,
        "freeze_base": args.freeze_base,
        "model": os.path.abspath(args.model),
        "mujoco_version": mujoco.__version__,
        "timestep": model.opt.timestep,
        "steps": n_steps,
        "joint_range": [float(v) for v in model.jnt_range[joint_id]],
        "initial_position": initial_pos,
        "final_position": final_pos,
        "final_error": args.target - final_pos,
        "saturated_steps": saturated_steps,
        "limit_margin_warning": margin_warning,
        "stop_reason": stop_reason,
        "log_file": args.log_file,
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
    print(f"{summary['joint']}: Kp {summary['Kp']:g}, target {summary['target']:+.4f}, "
          f"final {summary['final_position']:+.4f} (error {summary['final_error']:+.4f}), "
          f"saturated steps {summary['saturated_steps']}/{summary['steps'] + 1}, "
          f"margin warning: {summary['limit_margin_warning'] or 'none'}, "
          f"stopped: {summary['stop_reason'] or 'no'}. Log: {summary['log_file']}")
    return EXIT_OK

if __name__ == "__main__":
    sys.exit(main())