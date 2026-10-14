### Steps && Commands
  
  ## Milestone 2
     
     First pass of the source file:
     '''
         unset PYTHONPATH
         source .venv/bin/activate
         python -m src.position_controller --joint arm_right_shoulder_pitch_joint --target 0.50 --Kp 6.0 --duration 2.0
     '''