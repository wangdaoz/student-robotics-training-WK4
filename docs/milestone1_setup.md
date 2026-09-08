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

      '''
         unset PYTHONPATH
         source .venv/bin/activate
         python src/joint_inventory.py
         deactivate
      '''