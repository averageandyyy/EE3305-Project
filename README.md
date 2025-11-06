***EE3305/ME3243 Robotic System Design***
---

**&copy; Lai Yan Kai, National University of Singapore**

[01_About.md](docs/01_About.md)

[02_SLAM_and_Localization.md](docs/02_SLAM_and_Localization.md)

[03_ROS_System.md](docs/03_ROS_System.md)

[04_Controller.md](docs/04_Controller.md)

[05_Planner.md](docs/05_Planner.md)

[06_Bash_Scripts.md](docs/06_Bash_Scripts.md)

# Overall Items to Implement
- [ ] Behavior Node
  - [x] Be able to subscribe to odometry (from simulation)
  - [x] Be able to subscribe to a goal position (likely from RViz)
  - [x] Be able to publish a path request
    - [ ] Replanning is to be implemented with a timer
- [ ] Planner Node
  - [x] Be able to subscribe to path request
  - [x] Be able to subscribe to global costmap (from Map Server)
  - [x] Be able to publish a desired path (to Controller)
    - [ ] Implement Dijkstra
    - [ ] Publish interpolated path
    - [ ] Conversion functions
    - [ ] outOfMap
- [ ] Controller Node
  - [x] Be able to subscribe to odometry
  - [x] Be able to subscribe to a desired path
  - [x] Be able to publish a velocity command
    - [ ] Implement Pure Pursuit (also timer)
  - [ ] Be able to publish a lookahead

# Todos
- [x] Generate SLAM map (walk around with the bot, then save the map)

