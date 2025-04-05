# Comp0250 Coursework 2 Team 6

Authors: Lucas Young, Sonny Mo

Description: Coursework 2 - Pick and Place, Object Detection and Localization

## Pre-Requisites
```bash
sudo apt install ros-noetic-franka-ros ros-noetic-libfranka
```
Gazebo physics simluator is also needed (http://gazebosim.org/). This can be installed and then run with:
```bash
curl -sSL http://get.gazebosim.org | sh
gazebo
```

To run this task following packages are required: 
- Point Cloud Library
- MoveIt
- tf2
- Octomap

## Installation
Download the repository and place it in the `src` directory of your Catkin workspace.
Then, open a terminal and build the package using the following command:
```bash
catkin build
```

## Run Panda robot Gazebo and rviz
```bash
source devel/setup.bash
```
```bash
roslaunch cw2_team_6 run_solution.launch
```

## Run solutions run each task
The specific tasks should be launched from a separate terminal that has also been sourced

### Task 1 - (Lucas %, Sonny % , hours in total)
Given the position of the basket and the shape, a point cloud of the object is captured. The major axis of the object is extracted by PCA from the point cloud to determine its orientation. The goal is to pick up the shape and place it into the brown basket.

To run the task:
```bash
rosservice call /task 1
```
### Task 2 - (Lucas %, Sonny % hours in total)
Given two reference shapes and one mystery shape, the manipulator analyzes the scene and determines which reference shape matches the mystery shape.

To run the task:
```bash
rosservice call /task 2
```
The identified shapes are outputted in the ROS console as such:
```console
=================TASK 2 RESULT=================
[ INFO] [1743889837.305583744, 338.589000000]: The mystery object matches reference object 2
[ INFO] [1743889837.305597834, 338.589000000]: Reference object 1 shape: NOUGHT
[ INFO] [1743889837.305611571, 338.589000000]: Reference object 2 shape: CROSS
[ INFO] [1743889837.305624743, 338.589000000]: Mystery object shape: CROSS
```

### Task 3 - (Lucas %, Sonny % hours in total)
Task Objectives:
- Count the total number of objects, excluding the black obstacles.
- Identify which shape appears most frequently (or determine if there's a tie).
- Pick up and place one instance of the most common shape into the basket, while advoiding obstacles.

To run the task:
```bash
rosservice call /task 3
```

The identified total shapes are outputted in the ROS console as such:
```console
====== TASK 3 COMPLETED ======
[ INFO] [1743890238.553513906, 151.865000000]: Total shapes: 3, Most common shape count: 2
```

## License
Github Repo: [https://github.com/colinlaganier/COMP0129-CW3](https://github.com/coder2Young/comp0250_s25_labs.git)

This project is [MIT](LICENSE) licensed.

