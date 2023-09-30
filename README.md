# ENPM662_Project 0
UMD ENPM662 Project 0

# Student Information
Authors: Brendan Neal

Directory ID: bneal12

UID: 119471128

# Project Information
Goal: Software Installation, ROS2 Practice, OL Control of a 1 DOF System.

Package Name: tb_control

File Names: tb_openLoop.py, tb_grapher.py

Recommended IDE: Visual Studio Code

Python Version: 3

# Libraries Used
rclpy, Node, Twist, Odometry, matplotlib

# How to Run Code
1. Download the tb_control package to a workspace with all TurtleBot3 dependencies installed.
2. Open 3 separate terminals.
3. Change directories to said workspace (cd _____) on all 3 terminals.
4. Perform: 'colcon build' and 'source install/setup.bash' on all 3 terminals.
5. If these commands are not already in your ~/.bashrc file, perform them now in all 3 terminals:

---
export TURTLEBOT3_MODEL=burger
---

---
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg prefix turtlebot3_gazebo`/share/turtlebot3_gazebo/models/
---

6. In the first terminal, run the following command to launch the TurtleBot in Gazebo:

---
ros2 launch turtlebot3_gazebo empty_world.launch.py
---

7. In the second terminal, run the following command to launch the data logger (Robot Pose Subscriber):

---
ros2 run tb_control tb_grapher
---

8. In the third terminal, run the following command to begin open loop control of the TurtleBot:

---
ros2 run  tb_control  tb_openLoop
---

9. Observe the TurtleBot move 1m in 10s.
10. Once the TurtleBot completes its path, perform Ctl. C on the 2nd terminal to create a graph of the TurtleBot's pose over time.
11. Observe the graph, then Ctl. C on the other two terminals to end the program.
12. PROGRAM END
