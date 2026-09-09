### Milestone
    Milestone 1 — Build the Berkeley Joint/Actuator Inventory

### Current objective
     Improve the source file: src/joint_inventory.py for tasks #1, #2;
     finish the first pass of task #3
     
### Work Completed

    Revise the potential problems in the source file for tasks #1, #2
    finish the first version of python3 codes in the source file: src/joint_inventory.py

### Commands or tests run

    Tasks #1, #2, #3:
      '''
         unset PYTHONPATH
         source .venv/bin/activate
         python src/joint_inventory.py
         deactivate
      '''

### Blockers
    
     In task #3,
     I was not sure whether I should do this task by modifiying the existing file: src/joint_inventory.py; Assumption: modify the source file
     I was also not certain the name of the summary document; I supposed it is the "docs/joint_actuator_map.md".
     

### How I investigated

     I asked Claude Code with problems and my assumptions. Then I read the answers from Claude Code to make final decisions.

### AI tools used

    Claude Code helped me solve the confusion while I was reading the contents for this milestone.
    ChatGPT helped me solve issues about WSL commands and bugs in my program.

### What I Learned

        In order to write items of information for joints and actuators to a CSV file and a JSON file, different formats should be set via python codes: a list of lists, a list of directionary.

        To estimate whether the "forcerange" or "ctrlrange" is explicitly definied, inherited or absent for an actuator/motor, we should beginning at the actuator/motor node and checked its 'class'.