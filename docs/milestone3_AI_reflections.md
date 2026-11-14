### Problems in my first-pass version && Reflection

 1. The Controller switches off near the target
    When the absolute value of the error is less or equal to the value of margin, the control is set to 0. It is possible that the result of the final error is the margin not the result of the controller. 

 2. The Control was never bound
    --control was checked but never used, so Kp * error could be any size. That fails the “Control is bounded” requirement. MuJoCo also silently limits this joint’s torque to ±20 N·m (its forcerange), so with a high Kp my CSV would show torques the joint never received.
     
    Change the "--control" to "--max-control"; the command is clipped to that limit, and the log has new 'raw_control' and 'saturated' columns so I can see when the limit kicks in.

 3. Kp wasn't saved and The .meta.json file didn’t record Kp, and every run wrote to mile3_data.csv. That’s a problem for “compare at least three Kp values.” Kp is now in the summary, and the default file name includes the joint, target and Kp.

 4. The limit-margin rule didn’t match its help text.
    The help said the command is cut off if the joint enters the limit margin, but the code never did this. Cutting it would be wrong anyway: whenever the joint is near a limit, Kp * error already pushes it back toward the target, so cutting it removes the force that pulls the joint away. It would also have broken joints like the left elbow, which starts at its limit (0 rad). Version 2 records a warning instead. It also stops the run if the simulation blows up (position or velocity is no longer a number).

    In revised version, a warning of stop is recroded. It also stops the run if if the simulation blows up 

### What I Learned
 
In revised version file: position_controller_revised_version.py

● <applied = float(np.clip(raw, -args.max_control, args.max_control))>

   np.clip() restricts a value to a specified minimum and maximum.
   The syntax is: "np.clip(value, minimum, maximum)"

● <left_margin = margin_warning is None>
  
  1. margin_warning is None

     'is' operator checks whether two objects are the same objects; If they are same, output is True. Otherwise, the output is False.

● Line 291 to 296:
<   for _ in range(n_steps):
            step_and_log(do_step=True)
            if stop_reason is not None:
                break
    else:
        step_and_log(do_step=False)  # final measured state>

When a for loop has an else, Python runs the else block only if the loop finished all its iterations without hitting break. The else is attached to the for, not to the if above it.

So the condition for line 285 is: the simulation ran all n_steps steps and never became unstable
   
    ## Normal run
       the loop runs every step, break never happens, so else runs and logs one last row.

    ## Unstable run
       position or velocity became non-finite, stop_reason was set, and break ran. The else is skipped. There’s no point logging another row, because the state is already broken and that row was already written.