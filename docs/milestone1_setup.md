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
### Notes for for Future Students

    In src/joint_inventory.py:

    line 261: <json.dump(json_inventory, file, indent=4)>
      
    call module json's dump method:
                                   
               ● dump(xxx,xxx,xxx)

                    take an object(e.g. json_inventory), convert it into JSON format and write that
                    JSON into the target file, using 4 spaces for identification

               ● dumps(json_inventory indent=4)
                        
                     the object: json_inventory is taken and converted into JSON format and returns it as a string
   
   For general check whether ctrlrange/forcerange is explicitle definied or default/inherited: