# How to Use 
As per the instructions from the original repository, after building and
sourcing, simply run ``` ros2 launch ee3305_bringup run.launch.py cpp:=False
headless:=True ```
# Current Stack
The current stack is made to run the A* global path planning algorithm, Bezier
path smoothing and DWA local planner/controller. In general, the robot should
have no problems with movement. We note that the mismatch between the costmap
and odometry of the robot can result in unintended collisions.
# Switching Things Up
To see what can be changed or used, there are two main files of interest:
`ee3305_bringup/params/run.yaml` and `ee3305_py/ee3305_py/planner.py`.

To try out different planners, with or without Path Smoothing, simply comment
in/out the relevant blocks in `planner.py`. For `FastRRT*` specifically, we recommend setting `enable_controls` in `run.yaml` to false and to purely visualize the path output without movement.This is due to the algorithm's inherent incompatibality with lookahead-based controllers as per our implementations and the production of paths that are dangerously close to obstacles. We acknowledge these limitations and address them within our report and presentation.

To try out different controllers, simply change the `boolean` parameters of `use_dwa` and `use_RPP` in `run.yaml`. To try out the basic pure pursuit controller, simply set both items to `false`. Note that the program will not work (assertion wil be thrown and controller will crash) if both items are set to `true`.