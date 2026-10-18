### Steps && Commands
  
  ## Milestone 2
     
     First pass of the source file:
     '''
         unset PYTHONPATH
         source .venv/bin/activate
         python -m src.position_controller --joint arm_right_shoulder_pitch_joint --target 0.50 --Kp 6.0 --duration 2.0
     '''

  ## Milestone 3
    
     First pass of the source file:
     '''
         unset PYTHONPATH
         source .venv/bin/activate
         python -m src.position_controller --joint arm_right_shoulder_pitch_joint --target 0.50 --Kp 6.0 --duration 2.0
         env -u PYTHONPATH python -m pytest tests/test_joint_control.py -v
     '''

     Results:
=================================================================== test session starts ===================================================================
platform linux -- Python 3.12.3, pytest-9.1.1, pluggy-1.6.0 -- /home/kevin-lianhu/student-robotics-training-WK4/.venv/bin/python
cachedir: .pytest_cache
rootdir: /home/kevin-lianhu/student-robotics-training-WK4
collected 7 items                                                                                                                                         

tests/test_position_controller.py::test_three_joints_without_source_changes[arm_right_shoulder_pitch_joint-0.25] PASSED                             [ 14%]
tests/test_position_controller.py::test_three_joints_without_source_changes[arm_left_elbow_pitch_joint-0.5] PASSED                                  [ 28%]
tests/test_position_controller.py::test_three_joints_without_source_changes[leg_right_knee_pitch_joint-0.5] PASSED                                  [ 42%]
tests/test_position_controller.py::test_three_joints_with_different_proportional_gain[Proportional_Gain0] PASSED                                    [ 57%]
tests/test_position_controller.py::test_three_joints_with_different_proportional_gain[Proportional_Gain1] PASSED                                    [ 71%]
tests/test_position_controller.py::test_three_joints_with_different_proportional_gain[Proportional_Gain2] PASSED                                    [ 85%]
tests/test_position_controller.py::test_runs_are_reproducible PASSED                                                                                [100%]

==================================================================== 7 passed in 4.06s ====================================================================