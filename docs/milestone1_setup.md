### Steps && Commands
    
    In root of the local repo:
    -- virtual environment set 
     '''
        unset PYTHONPATH
        python3 -m venv .venv
     '''
    
    -- Install mujoco module
      '''
         python3 --version
         source .venv/bin/activate
         python --version
         pip install --upgrade pip
         pip install mujoco
         pip freeze > requirements.txt
         grep -n "venv" .gitignore
      '''
  - Engineering Tasks
    ● Generate the inventory from the compiled MuJoCo model.
    ● Write it to CSV or JSON.
    ● Create a human-readable summary document from the generated data

      '''
         unset PYTHONPATH
         source .venv/bin/activate
         python src/joint_inventory.py
         deactivate
      '''

    ● Add tests for several known Berkeley joints actuators
      '''
         unset PYTHONPATH
         source .venv/bin/activate
         python -m pytest tests/test_joint_inventory.py -v
         deactivate
      '''

      Expected Results:
================================================================== test session starts ===================================================================
platform linux -- Python 3.12.3, pytest-9.1.1, pluggy-1.6.0 -- /home/kevin-lianhu/student-robotics-training-WK4/.venv/bin/python
cachedir: .pytest_cache
rootdir: /home/kevin-lianhu/student-robotics-training-WK4
collected 7 items                                                                                                                                        

tests/test_joint_inventory.py::test_free_base_joint_seven_qpos_six_dof PASSED                                                                      [ 14%]
tests/test_joint_inventory.py::test_actuator_forcerange_explicit_no_ctrlrange PASSED                                                               [ 28%]
tests/test_joint_inventory.py::test_every_actuator_drives_a_joint PASSED                                                                           [ 42%]
tests/test_joint_inventory.py::test_no_unexpected_unactuated_joints PASSED                                                                         [ 57%]
tests/test_joint_inventory.py::test_forcerange_provenance_matches_raw_xml PASSED                                                                   [ 71%]
tests/test_joint_inventory.py::test_known_joint_ranges[arm_left_shoulder_pitch_joint-expected_range0] PASSED                                       [ 85%]
tests/test_joint_inventory.py::test_known_joint_ranges[arm_right_shoulder_pitch_joint-expected_range1] PASSED                                      [100%]

=================================================================== 7 passed in 1.20s ====================================================================
### Notes for for Future Students

   ● src/joint_inventory.py:

    line 261: <json.dump(json_inventory, file, indent=4)>
      
    call module json's dump method:
                                   
               ● dump(xxx,xxx,xxx)

                    take an object(e.g. json_inventory), convert it into JSON format and write that
                    JSON into the target file, using 4 spaces for identification

               ● dumps(json_inventory indent=4)
                        
                     the object: json_inventory is taken and converted into JSON format and returns it as a string
   
   ● tests/test_joint_inventory.py
    
     line 5: <@pytest.fixture(scope = "module")>

            -- '@'
               The decorator tells Python/pytest:
                "Treat the function immediately below this line as a pytest fixture."

            -- 'pytest'
                A python testing framework

            -- fixture
               A fixture is a function that prepares something needed by your tests—for example, creating a MuJoCo model and simulation data before running tests.

            -- scope = "module"

               The <scope> controls how often pytest creates the fixture

               e.g. @pytest.fixture(scope = "module")

                  create this fixture once for the entire Python test module and reuse it for all tests in that module

            Start pytest
                 │
                 ▼
            Create model fixture
                 │
                 ├──────► test_model_exists()
                 │
                 ├──────► test_model_has_joints()
                 │
                 └──────► test_model_has_bodies()
                 │
                 ▼
            Finish module
                 │
                 ▼
            Destroy fixture

            Common fixture scopes: "function", "class", "module", "package", 
            
            "session": fixture created once for entire test run
 
     line 53: <@pytest.mark.parametrize("joint_name,expected_range", [
    ("arm_left_shoulder_pitch_joint", (-1.571,  0.785)),
    ("arm_right_shoulder_pitch_joint", (-0.785,  1.571)),
    ])>

     -- "joint_name, expected_range"
        defines the two parameters that will be passed to the test function.

     -- the list: [...]

        a list of test cases 