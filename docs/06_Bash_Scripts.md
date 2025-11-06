6&emsp;Bash Scripts
---

***EE3305/ME3243 Robotic System Design***

**&copy; Lai Yan Kai, National University of Singapore**

A bash script (file extension `.sh`) contains terminal commands, allowing users to conveniently run long terminal commands without re-typing them.
This chapter shows how a bash script can be created and run, and provides some examples of these scripts.

# Table of Contents

[1&emsp;Create, Allow, Run](#1create-allow-run)

[2&emsp;Example Scripts](#2example-scripts)

&emsp;[2.1&emsp;Build: `bd.sh`](#21-build-bdsh)

&emsp;[2.2&emsp;Clean Build: `clean_bd.sh`](#22clean-build-clean_bdsh)

&emsp;[2.3&emsp;Kill Ruby Scripts (Gazebo): `kill.sh`](#23kill-ruby-scripts-gazebo-killsh)

&emsp;[2.4&emsp;Run: `run.sh`](#24run-runsh)

&emsp;[2.5&emsp;Teleop: `teleop.sh`](#25teleop-teleopsh)

&emsp;[2.6&emsp;Simulate SLAM: `sim_slam.sh`](#26simulate-slam-sim_slamsh)

&emsp;[2.7&emsp;Save SLAM Map: `save_map.sh`](#27save-slam-map-save_mapsh)

&emsp;[2.8&emsp;Simulate Nav2 Localization: `sim_nav2.sh`](#28simulate-nav2-localization-sim_nav2sh)


# 1&emsp;Create, Allow, Run
1. Use VSCode to create a new file ending in `.sh` in the workspace directory. For example, `bd.sh`. 

2. In a terminal in the workspace directory, allow running the script with `chmod`. This should be **run once for every new `.sh` file**.

    ```bash
    cd ~/ee3305     # go to workspace.
    chmod +x *.sh   # allow all current .sh files in this directory.
    ```
    Again, the command only needs to be run once for each file in its lifetime.

3. To run a script called `bd.sh`, 

    ```bash
    cd ~/ee3305     # go to workspace.
    ./bd.sh         # run commands in bd.sh in a new subshell.
    ```
    The `./` copies the environment variables in the current shell to a new subshell without modifying the current shell's variables. This should be the preferred way to execute bash scripts in this course.

# 2&emsp;Example Scripts

The scripts below are used by the author to quickly run commands.

## 2.1&emsp;Build: `bd.sh`
For re-building the workspace.

```bash
#!/bin/bash
colcon build --symlink-install
```

## 2.2&emsp;Clean Build: `clean_bd.sh`
For re-building the workspace if the built executables do not seem to perform as intended (mostly due to careless changes to the package and executables). 
**Requires `bd.sh`**.

```bash
#!/bin/bash
rm -rf build install log
./bd.sh
```

## 2.3&emsp;Kill Ruby Scripts (Gazebo): `kill.sh`
Gazebo may occasionally fail to stop running when the user interrupts with `Ctrl+C`. 
Since Gazebo is run using ruby scripts, we can kill all ruby scripts with `pkill -9 ruby`. 
However, if there are other applications run by ruby, it will kill those applications as well.
As no applications are run by ruby in this course, we assume that the following is sufficient.

```bash
#!/bin/bash
# kills gz, which is run by ruby. This will kill other ruby processes.
pkill -9 ruby
```

## 2.4&emsp;Run: `run.sh`
This script runs the project files in simulation. The launch arguments below can be modified.
**Requires `kill.sh`**.

```bash
#!/bin/bash
source install/setup.bash
ros2 launch ee3305_bringup run.launch.py cpp:=False headless:=False libgl:=False
./kill.sh # run in case gz cannot be interrupted properly.
```

## 2.5&emsp;Teleop: `teleop.sh`
Teleoperates the turtlebot.

```bash
#!/bin/bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

## 2.6&emsp;Simulate SLAM: `sim_slam.sh`

Runs the simulation and SLAM together. 
The launch arguments below can be modified.
**Requires `kill.sh`**.

```bash
#!/bin/bash
source install/setup.bash
ros2 launch ee3305_bringup sim_slam.launch.py headless:=False libgl:=False
./kill.sh
```

## 2.7&emsp;Save SLAM Map: `save_map.sh`

Saves the map created by the running SLAM node and installs the map by building the workspace.

```bash
#!/bin/bash
# make sure the SLAM (cartographer) node is running.
ros2 run nav2_map_server map_saver_cli -f src/ee3305_bringup/maps/ee3305
./bd.sh
```

## 2.8&emsp;Simulate Nav2 Localization: `sim_nav2.sh`

Runs the simulation and Nav2 localization based on the previously saved SLAM map. 
The launch arguments below can be modified.
**Requires `kill.sh`**.

```bash
#!/bin/bash
source install/setup.bash
ros2 launch ee3305_bringup sim_nav2.launch.py headless:=False libgl:=False
./kill.sh
```