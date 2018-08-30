# Compare the `move_in_python` and `move_in_cpp` implementations

Both nodes print out the following log messages:

```
...
[INFO] [1535539689.251862]: yaw: 1570.0000, throttle: 0.0000
[INFO] [1535539689.303816]: yaw: 1570.0000, throttle: 0.0000
[INFO] [1535539689.350007]: yaw: 1570.0000, throttle: 0.0000
[INFO] [1535539689.399785]: yaw: 1570.0000, throttle: 0.0000
[INFO] [1535539689.451338]: yaw: 1570.0000, throttle: 0.0000
[INFO] [1535539689.500438]: yaw: 1570.0000, throttle: 0.0000
[INFO] [1535539689.504745]: yaw: 1570.0000, throttle: 0.0000
...

```

To compare them, run:

```bash
rosrun move_laser move_in_python &> ../compare_implementations/python_output
```

and then:

```bash
rosrun move_laser move_in_cpp &> ../compare_implementations/cpp_output
```

To analyze the outputs, check out the `../compare_implementations/analyze_log_outputs.ipynb` notebook.
