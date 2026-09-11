### Milestone
    Milestone 1 — Build the Berkeley Joint/Actuator Inventory

### Current objective
     Improve the source file: src/joint_inventory.py for tasks #3;
     finish the first pass and AI improvement for task #4
     
### Work Completed

    Revise the potential problems in the source file for tasks #3
    finish the first version and improvement of python3 codes in the source file: tests/test_joint_inventory.py

### Commands or tests run

    Tasks #3:
      '''
         unset PYTHONPATH
         source .venv/bin/activate
         python src/joint_inventory.py
         deactivate
      '''

    Task #4:
      '''
         unset PYTHONPATH
         source .venv/bin/activate
         python -m pytest tests/test_joint_inventory.py -v
         deactivate
      '''

        or

      '''
         source .venv/bin/activate
         env -u PYTHONPATH python -m pytest tests/test_joint_inventory.py -v
         deactivate
      '''

### Blockers

    For task 4, I was not sure whether I should test known joints/actuators based on existing CSV/JSON files existing in previous tasks.

### How I investigated

     I asked Claude Code about my confusion. It explained its analysis combined with requirements in handbook. The specific contents are the following:
                 
                Why that doesn't work: the acceptance criteria for this milestone literally says "Tests prove the mapping for representative joints" — the word "prove" is doing real work there. If your test reads joint_actuator_inventory.csv, pulls a value out of it, and asserts that value equals... itself (or a re-derived version of itself), you've only proven that your generation code is consistent with its own output — not that the output is correct. Concretely: none of the four bugs we found in code review (the joint-type dict lookup always missing, the <include> file parsed as empty, etc.) would have been caught by a CSV-based test, because in every case the bug produced self-consistent but wrong data — a test comparing the CSV against itself would have happily passed the whole time.

                What "prove" actually requires: an expected value that comes from somewhere independent of your generator — i.e., a fact you hand-verified yourself by reading the raw MJCF text or the MuJoCo docs directly (the same independent-verification process the handbook's "Independent First Pass" already has you doing), hardcoded as a literal in the test.

                Concrete shape this should take:
                     
                         Pick a handful of representative joints that exercise different cases — not randomly, deliberately: at least one ordinary hinge joint, the floating-base free joint (multi-slot addressing), and if the model has one, a joint that intentionally has no actuator (the inventory spec calls this out explicitly).

                         For each, write down the ground-truth facts by reading the source yourself — e.g., we already did this together for arm_left_shoulder_pitch_joint: its raw <motor> tag has an explicit forcerange="-20 20" and no ctrlrange anywhere in its default chain. That's a fact you can hardcode with confidence because you traced it by hand, not because the generator told you.

                         In the test, call your actual functions from joint_inventory.py (import build_actuator_index, joint_jsontable, actuator_jsontable, build_provenance_map, etc. — don't reimplement the logic in the test) against a freshly loaded model, and assert the function output matches your hardcoded ground truth.

### AI tools used

                Claude Code helped me solve the confusion while I was reading the contents for this milestone.
                ChatGPT helped me solve issues about WSL commands and bugs in my program.

### What I Learned

        in test file: tests/test_joint_inventory.py

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