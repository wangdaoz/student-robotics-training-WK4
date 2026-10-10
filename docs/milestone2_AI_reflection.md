### Problems in my first-pass version

    1. Every invalid Request Crashes.
       The error rows call <data.ctrl[act_id]> before act_id exists. So an unknown joint, a free joint or <--target 50> all end in "UnboundLocalError", and my error rows are never written. The <return -1> lines after each raise never run.

    2. The joint ID is used before it's checked.
        <jnt_qposadr[joint_id]> runs before the <joint_id == -1> check. In numpy, -1 means the last joint, so a typo would silently read the wrong joint.
    
    3. "--target" means wrong thing
       The handbook means a joint position (--target 0.25) that is checked against the joint's range, with error = target − position. Your version treats it as a torque and never checks it against the range. As a result, the default run drives the shoulder to its limit (1.5728 rad against a 1.5708 limit). Milestone 0 asks you to stop doing exactly that.

    4. "ctrlrange" and "forcerange" are mixed up
       These actuators are <motor> with gear 1, so ctrl is torque in N·m. ctrlrange is not limited (it compiles to [0, 0]), and forcerange is ±20. Your check against forcerange only works by luck. The new code checks ctrlrange only when it's limited, and checks gain × gear × ctrl against forcerange.

    5. "--freeze-base" doesn't freeze the base
       Setting body_mocapid does nothing, and turning off gravity is not the same as welding the base. The new version removes the free joint at load time and leaves gravity on. The MJCF file on disk is not changed, which fits the handbook's "document the change, don't secretly alter the model" rule.

    6. Smaller issues
       ● The time column was wall-clock time to the nearest second, so it wasn't reproducible. It is now simulation time.
       ● Splitting torque and steps across several actuators made no sense. A joint with more than one actuator is now rejected.
       ● The log folder wasn't created if missing.
       ● An invalid request overwrote the previous evidence CSV.
       ● Only an uncaught exception gave a non-zero exit status.

### What I Learned

    In the modified source file:

      -- line 79: <spec = mujoco.MjSpec.from_file(model_path)>
        
         ● MjSpec

           A MuJoCo class that represents a model's specification;
           can be used to inspect or modify the model structure before compilation

           MjModel represents the compiled model used by MuJoCo for simulation

           < model = spec.compile() >: compile it into a simulation ready model

      -- line 195: <direction = math.copysign(1.0, initial_error) if initial_error != 0 else 0.0>

          ● math.copysign(1.0, initial_error)
             
             -- first parameter is the magnitude we want in the result

             -- 'initial_error'
                the value whose sign determines whether the result is positive or negative
