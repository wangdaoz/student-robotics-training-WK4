Engineering Tasks

  Tasks #1, #2
         
      Error:
             [
                Traceback (most recent call last):
                  File "/home/kevin-lianhu/student-robotics-training-WK4/src/joint_inventory.py", line 266, in <module>
                    main()
                  File "/home/kevin-lianhu/student-robotics-training-WK4/src/joint_inventory.py", line 261, in main
                    json.dump(json_inventory, file, indent=4)
                  File "/usr/lib/python3.12/json/__init__.py", line 179, in dump
                    for chunk in iterable:
                  File "/usr/lib/python3.12/json/encoder.py", line 432, in _iterencode
                    yield from _iterencode_dict(o, _current_indent_level)
                  File "/usr/lib/python3.12/json/encoder.py", line 406, in _iterencode_dict
                    yield from chunks
                  File "/usr/lib/python3.12/json/encoder.py", line 326, in _iterencode_list
                    yield from chunks
                  File "/usr/lib/python3.12/json/encoder.py", line 406, in _iterencode_dict
                    yield from chunks
                  File "/usr/lib/python3.12/json/encoder.py", line 439, in _iterencode
                    o = _default(o)
                        ^^^^^^^^^^^
                  File "/usr/lib/python3.12/json/encoder.py", line 180, in default
                    raise TypeError(f'Object of type {o.__class__.__name__} '
                TypeError: Object of type int32 is not JSON serializable
             ]

      - Analysis
        In the source file:
           When calling json.dump(), it is trying convert your Python object json-inventory into JSON.
           The problem was that somewhere inside the "json-inventory", I had at least one value whose type is: int32 rather than ordinary python. Python's JSON encounder understands normal Python Types such as:
                  int, float, str, bool, None, list, dict
           but it doesn't automatically understand NumPy integer types such as: numpy.int32, numpy.int64
        
      - Fix
           In the source file: src/joint_inventory.py,

           in lines: 129, 131, 233, 236, convert all these values to 'int' by writing in such a form "int(xxx)", then use same commands re-run the file.