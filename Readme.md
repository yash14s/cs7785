# CS 7785: Intro to Robotics Research

<img src="https://github.com/yash14s/cs7785/blob/main/media/turtlebot3.jpg" alt="Turtlebot" width="350">

This repository contains our solution for the course CS 7785 at Georgia Tech, taught by Dr Sean Wilson during Spring 2023. Contributors: [Pranay Mathur](https://github.com/Matnay) and [Yash Srivastava](https://github.com/yash14s).

## Pre-reqs:

A Linux Machine with ROS2 installed. Refer /assignment_pdfs/lab0.pdf for instructions. All additional dependencies are mentioned in the /assignment_pdfs directory with the corresponding labs.

![DemoGIF](https://github.com/yash14s/cs7785/blob/main/media/final_demo.gif)

## Execution
```python
# Setup environment variables
export TURTLEBOT3_MODEL=burger

# Launch the environment and turtlebot3_bringup
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 
ros2 launch turtlebot3_bringup robot.launch.py

# For keyboard tele-operation
ros2 run turtlebot3_teleop teleop_keyboard

# To excute the scripts found in the directory
ros2 run <package_name> <executable_name>
```
