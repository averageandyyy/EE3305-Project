EE3305 Part 2 Project
=====
Controlling and Planning a Simulated Differential Drive

# Table of Contents

# Install Software
See the md

# Download Files

# Launching

## Launch file
The following shows the command to run Python file without the graphic user interface for Gazebo.
```bash
cd ~/ee3305
source install/setup.bash
ros2 launch ee3305_bringup run.launch.py cpp:=False headless:=True
```
The launch arguments appended to the command are described below:
| | Launch Argument | Description |
|-|-|-|
| 1. | `cpp` | `False` to run from Python code (default). `True` to run from C++ code. | 
| 2. | `headless` | `True` to run the Gazebo graphic user interface showing the 3D world (default). `False` to run without the user interface and run the simulation in the background, saving computational resources. |
| 3. | ... | Other non-essential launch arguments can be found in the `run.launch.py` launch file. |

## Kill Gazebo Harmonic
Gazebo may occasionally continue to run when a keyboard interrupt (`Ctrl+C`) is sent to stop the ROS nodes.
Subsequently running another launch may cause more than one Gazebo to run, and this will cause problems with the simulation.
A quick hack to close Gazebo is to kill all ruby scripts, since Gazebo is run using a ruby script:
```bash
pkill -9 ruby
```
Do this everytime after `Ctrl+C` is sent.

## Bash Scripting

# Planner Task
Dijkstra. To improve.

# Controller Task
Pure Pursuit controller. To improve.

# Submissions

## P1R: Report `.pdf`

## P1C: Code `.zip`
Code will be demonstrated during the session.

## P1P: Presentation: Poster `.pdf`


