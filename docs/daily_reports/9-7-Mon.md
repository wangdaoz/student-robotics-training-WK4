### Milestone
    Milestone 1 — Build the Berkeley Joint/Actuator Inventory

### Current objective
    Generate the inventory from the compiled MuJoCo model
    Write it to CSV or JSON

### Work Completed
      1. create the main structure of the local repository
      2. load the Berkeley Humanoid assets submodule for the repo
      3. finish the engineering task #1, #2
      4. create a "milestone1_setup.md" file 

### Commands or tests run

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

      '''
         unset PYTHONPATH
         source .venv/bin/activate
         python src/joint_inventory.py
         deactivate
      '''

### Blockers
    
    After I first successfully ran the source file: "joint_inventory.py", I opened the target log file.
       The format was in chaos; Each character was separated by a comma.

### How I investigated

    I opened the source file and checked but didn't figure out the actual problem. Then, I uploaded my source file and log file to ChatGPT and asked it for help. It told me that in my list: "lines", each element is a string; A Python string is iterable one character at a time; But <csv.writer.writerow()> expects an iterable of fields (columns).

    It also pointed that my original program generates an Markdown table text not and suggested me create a real CSV file

### AI tools used

    Claude Code helped me solve the confusion while I was reading the contents for this milestone.
    ChatGPT helped me solve issues about WSL commands and bugs in my program.

### What I Learned

    To generate a CSV file via python, I should give csv writer an actual list of columns and then write corresponding items of information in it.

    Use "with open(...) as file: ..." to open the target file, the target file will be closed automatically after program exits this block.