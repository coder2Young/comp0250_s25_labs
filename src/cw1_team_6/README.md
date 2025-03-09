# COMP0250 Coursework 1 Team 6

Github Repo: https://github.com/coder2Young/comp0250_s25_labs

## Project Overview
This repository contains the solution for **COMP0250: Robot Sensing, Manipulation and Interaction Coursework 1: Pick and Place, Object Detection and Localization**, created by Lucas Yang, Sonny Mo. The project includes a single node handling all three tasks (Task 1, Task 2, and Task 3) through ROS services.

## How to Build and Run
### Prerequisites
- Ensure you have ROS (recommended version: Noetic) installed on your system.
- Install necessary dependencies: `sensor_msgs`, `geometry_msgs`, `moveit`, `opencv2`, `pcl`.
- Clone the repository and its submodules:
  ```bash
  git clone https://github.com/coder2Young/comp0250_s25_labs.git --recurse-submodules

### Building the project


Navigate to the workspace root and build the workspace:

    cd comp0250_s25_labs
    catkin build
    
Source the workspace:

    source devel/setup.bash

### Running the Solution
In the first terminal, launch the coursework environment with:

    roslaunch cw1_team_6 run_solution.launch
    
In the second terminal, launch the specific tasks with:

    rosservice call /task X

where X is the number of the tasks.

### Task 1
Lucas 50% Sonny 50% (2 hours)

Given the position of the basket and the cube, this task is to control the robot arm to pick up the cube and place it in the basket without collision. To run the task:

    rosservice call /task 1

The robot arm will move to the top of the box, then move down and grasp. Then move to the top of basket, move down and finaly open the gripper to place the box.

### Task 2
Lucas 50% Sonny 50% (15 hours)

Baskets will be spawned randomly. This task is to report the basket colours at each location or if any locations are empty. To run the task:

    rosservice call /task 2

The robot arm to move higher, make sure the camera is covering the whole scene. Then transform the base_frame point into camera_frame and check it's rgb value in the camera image coordinate.

The output of the basket colours and if any empty locations are printed in the ROS console as such:

    [ INFO] [1741556753.174384087, 13.898000000]: Basket 1 at 2D image coordinate: u=89, v=309
    [ INFO] [1741556753.188075967, 13.912000000]: Basket 1 r,g,b: r=0.094, g=0.094, b=0.863
    [ INFO] [1741556753.188165411, 13.912000000]: Basket 1/4: blue
    [ INFO] [1741556753.188237721, 13.912000000]: Basket 2 at 2D image coordinate: u=113, v=141
    [ INFO] [1741556753.189352340, 13.914000000]: Basket 2 r,g,b: r=0.478, g=0.718, b=0.478
    [ INFO] [1741556753.189425817, 13.914000000]: Basket 2/4: none
    [ INFO] [1741556753.189471477, 13.914000000]: Basket 3 at 2D image coordinate: u=558, v=121
    [ INFO] [1741556753.189680428, 13.914000000]: Basket 3 r,g,b: r=0.890, g=0.122, b=0.890
    [ INFO] [1741556753.189757107, 13.914000000]: Basket 3/4: purple
    [ INFO] [1741556753.189806407, 13.914000000]: Basket 4 at 2D image coordinate: u=563, v=317
    [ INFO] [1741556753.190105116, 13.915000000]: Basket 4 r,g,b: r=0.871, g=0.102, b=0.102
    [ INFO] [1741556753.190170451, 13.915000000]: Basket 4/4: red
    [ INFO] [1741556753.190203671, 13.915000000]: Task2 completed
    [ INFO] [1741556753.190306433, 13.915000000]: Response message:
    [ INFO] [1741556753.190369139, 13.915000000]: Response message: blue none purple red
    
### Task 3
Lucas 70% Sonny 30% (48 hours)

The task is to place each cube into a basket of the same colour. To run the task:

    rosservice call /task 3

This solution uses region-growing and point cloud filtering in the PCL libarary to determine the location of boxes and baskets. Then apply pick and place just like the task1 did.

## License
LICENSE: MIT.  See [LICENSE](LICENSE)
