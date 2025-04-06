Authors: Eddie Edwards (eddie.edwards@ucl.ac.uk), Kefeng Huang, Bowie (Heiyin) Wong, Dimitrios Kanoulas, Luke Beddow, Denis Hadjivelichkov

Description: This package forms the base ROS workspace for the module COMP0250 (formerly COMP0129): Robotic Sensing, Manipulation and Interaction.

## Pre-Requisites
```bash
sudo apt install ros-noetic-franka-ros ros-noetic-libfranka
```
Gazebo physics simluator is also needed (http://gazebosim.org/). This can be installed and then run with:
```bash
curl -sSL http://get.gazebosim.org | sh
gazebo
```

## Installation
```bash
git clone --recurse-submodules https://github.com/surgical-vision/comp0250_s25_labs.git
```
```bash
cd comp0250_s25_labs
```
```bash
git submodule update --init --recursive
```
```bash
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
```
```bash
catkin build
```

## Run Panda robot Gazebo and rviz
```bash
source devel/setup.bash
```
```bash
roslaunch panda_description description.launch
```

## Package Preparation (not to be ran by students):
```bash
mkdir src
git submodule add https://github.com/COMP0129-UCL/panda_moveit_config.git src/panda_moveit_config
git submodule add https://github.com/COMP0129-UCL/panda_description.git src/panda_description
git submodule add https://github.com/RPL-CS-UCL/realsense_gazebo_plugin.git src/realsense_gazebo_plugin 
git submodule add https://github.com/RPL-CS-UCL/rpl_panda_with_rs.git src/rpl_panda_with_rs
```

## License
LICENSE: MIT.  See [LICENSE.txt](LICENSE.txt)

DISCLAIMER:

THIS INFORMATION AND/OR SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS INFORMATION AND/OR
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Copyright (C) 2019-2024 Dimitrios Kanoulas and Eddie Edwards except where specified
