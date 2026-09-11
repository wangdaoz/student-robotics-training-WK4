Milestone 1
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

    Task #4
         After I inputted the commands:
                                       '''
                                          source .venv/bin/activate
                                          python -m pytest tests/test_joint_inventory.py -v
                                       '''
         Error Report:
[
Traceback (most recent call last):
  File "<frozen runpy>", line 198, in _run_module_as_main
  File "<frozen runpy>", line 88, in _run_code
  File "/home/kevin-lianhu/student-robotics-training-WK4/.venv/lib/python3.12/site-packages/pytest/__main__.py", line 9, in <module>
    raise SystemExit(_console_main())
                     ^^^^^^^^^^^^^^^
  File "/home/kevin-lianhu/student-robotics-training-WK4/.venv/lib/python3.12/site-packages/_pytest/config/__init__.py", line 253, in _console_main
    code = _main(prog=_get_prog_name(sys.argv))
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/home/kevin-lianhu/student-robotics-training-WK4/.venv/lib/python3.12/site-packages/_pytest/config/__init__.py", line 223, in _main
    config = _prepareconfig(new_args, plugins, prog=prog)
             ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/home/kevin-lianhu/student-robotics-training-WK4/.venv/lib/python3.12/site-packages/_pytest/config/__init__.py", line 410, in _prepareconfig
    config: Config = pluginmanager.hook.pytest_cmdline_parse(
                     ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/home/kevin-lianhu/student-robotics-training-WK4/.venv/lib/python3.12/site-packages/pluggy/_hooks.py", line 512, in __call__
    return self._hookexec(self.name, self._hookimpls.copy(), kwargs, firstresult)
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/home/kevin-lianhu/student-robotics-training-WK4/.venv/lib/python3.12/site-packages/pluggy/_manager.py", line 120, in _hookexec
    return self._inner_hookexec(hook_name, methods, kwargs, firstresult)
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/home/kevin-lianhu/student-robotics-training-WK4/.venv/lib/python3.12/site-packages/pluggy/_callers.py", line 167, in _multicall
    raise exception
  File "/home/kevin-lianhu/student-robotics-training-WK4/.venv/lib/python3.12/site-packages/pluggy/_callers.py", line 139, in _multicall
    teardown.throw(exception)
  File "/home/kevin-lianhu/student-robotics-training-WK4/.venv/lib/python3.12/site-packages/_pytest/helpconfig.py", line 124, in pytest_cmdline_parse
    config = yield
             ^^^^^
  File "/home/kevin-lianhu/student-robotics-training-WK4/.venv/lib/python3.12/site-packages/pluggy/_callers.py", line 121, in _multicall
    res = hook_impl.function(*args)
          ^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/home/kevin-lianhu/student-robotics-training-WK4/.venv/lib/python3.12/site-packages/_pytest/config/__init__.py", line 1232, in pytest_cmdline_parse
    self.parse(args)
  File "/home/kevin-lianhu/student-robotics-training-WK4/.venv/lib/python3.12/site-packages/_pytest/config/__init__.py", line 1583, in parse
    self.pluginmanager.load_setuptools_entrypoints("pytest11")
  File "/home/kevin-lianhu/student-robotics-training-WK4/.venv/lib/python3.12/site-packages/pluggy/_manager.py", line 416, in load_setuptools_entrypoints
    plugin = ep.load()
             ^^^^^^^^^
  File "/usr/lib/python3.12/importlib/metadata/__init__.py", line 205, in load
    module = import_module(match.group('module'))
             ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/usr/lib/python3.12/importlib/__init__.py", line 90, in import_module
    return _bootstrap._gcd_import(name[level:], package, level)
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "<frozen importlib._bootstrap>", line 1387, in _gcd_import
  File "<frozen importlib._bootstrap>", line 1360, in _find_and_load
  File "<frozen importlib._bootstrap>", line 1310, in _find_and_load_unlocked
  File "<frozen importlib._bootstrap>", line 488, in _call_with_frames_removed
  File "<frozen importlib._bootstrap>", line 1387, in _gcd_import
  File "<frozen importlib._bootstrap>", line 1360, in _find_and_load
  File "<frozen importlib._bootstrap>", line 1310, in _find_and_load_unlocked
  File "<frozen importlib._bootstrap>", line 488, in _call_with_frames_removed
  File "<frozen importlib._bootstrap>", line 1387, in _gcd_import
  File "<frozen importlib._bootstrap>", line 1360, in _find_and_load
  File "<frozen importlib._bootstrap>", line 1331, in _find_and_load_unlocked
  File "<frozen importlib._bootstrap>", line 935, in _load_unlocked
  File "<frozen importlib._bootstrap_external>", line 995, in exec_module
  File "<frozen importlib._bootstrap>", line 488, in _call_with_frames_removed
  File "/opt/ros/jazzy/lib/python3.12/site-packages/launch_testing/__init__.py", line 15, in <module>
    from . import tools
  File "/opt/ros/jazzy/lib/python3.12/site-packages/launch_testing/tools/__init__.py", line 18, in <module>
    from .process import launch_process
  File "/opt/ros/jazzy/lib/python3.12/site-packages/launch_testing/tools/process.py", line 17, in <module>
    import launch
  File "/opt/ros/jazzy/lib/python3.12/site-packages/launch/__init__.py", line 17, in <module>
    from . import actions
  File "/opt/ros/jazzy/lib/python3.12/site-packages/launch/actions/__init__.py", line 17, in <module>
    from .declare_launch_argument import DeclareLaunchArgument
  File "/opt/ros/jazzy/lib/python3.12/site-packages/launch/actions/declare_launch_argument.py", line 24, in <module>
    from ..frontend import Entity
  File "/opt/ros/jazzy/lib/python3.12/site-packages/launch/frontend/__init__.py", line 17, in <module>
    from . import type_utils
  File "/opt/ros/jazzy/lib/python3.12/site-packages/launch/frontend/type_utils.py", line 23, in <module>
    from ..utilities.type_utils import AllowedTypesType
  File "/opt/ros/jazzy/lib/python3.12/site-packages/launch/utilities/type_utils.py", line 31, in <module>
    import yaml  # type: ignore
    ^^^^^^^^^^^
ModuleNotFoundError: No module named 'yaml'
]

  Analysis:
           What's happening:
                            when pytest starts, it auto-discovers and loads every installed package that registers itself as a pytest plugin (via a pytest11 entry point) — regardless of whether your project actually uses it. Your traceback shows pytest found launch_testing under /opt/ros/jazzy/lib/python3.12/site-packages/, which is part of ROS 2 Jazzy, not your project. It tried to load it as a plugin, that import chain needs PyYAML, and yaml isn't installed inside your .venv.

                            The reason ROS's packages are visible at all from inside your activated .venv is almost certainly PYTHONPATH. Virtual environments don't normally see system packages, but they don't override PYTHONPATH either — if you've sourced ROS's setup script (source /opt/ros/jazzy/setup.bash) at some point in this same shell session, it prepends /opt/ros/jazzy/lib/python3.12/site-packages to PYTHONPATH, and that leaks through into your venv's Python, making pytest see (and try to load) ROS's plugins alongside your own.

                            fix:
                              inside the vritual .venv, input the command:
                              '''
                                 env -u PYTHONPATH python -m pytest tests/test_joint_inventory.py -v
                              '''