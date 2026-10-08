### Milestone
     Milestone 2 — Reusable Single-Joint Command Tool

### Current objective
     write first-pass version of the source file;
     Check the potential errors and revise them with the help of the Claude Code

### Work Completed

    First version of the source file;
    Discover the potential errors in the source file

### Commands or tests run

    '''
       unset PYTHONPATH
       source .venv/bin/activate
       python -m src.joint_control --joint arm_right_shoulder_pitch_joint --target 0.25 --duration 1.0
    '''


### Blockers

### How I investigated

### AI tools used

### What I Learned
    
    When I wrote codes and adjust the sequence of segments of code, I should think about internal logical relation to avoid some unnecessary bugs.

### Next action
    Revise these errors and re-run the source file.