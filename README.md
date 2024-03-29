# Obstacle-Avoidance-System

[![Build Status](https://travis-ci.com/savnani5/Obstacle-Avoidance-System.svg?branch=main)](https://travis-ci.com/github/savnani5/Obstacle-Avoidance-System)
[![Coverage Status](https://coveralls.io/repos/github/savnani5/Obstacle-Avoidance-System/badge.svg?branch=main)](https://coveralls.io/github/savnani5/Obstacle-Avoidance-System?branch=main)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Overview
We develop a ROS package to move TurtleBot3 in a Gazebo world. It is capable of avoiding obstacles while navigating to the goal location. We use Lidar sensor data and divide the 360 degrees data into 8 sub-regions to detect and avoid obstacles. We can run the node with a location argument passed to the command line. Based on the argument passed, we move the robot from its current location to the goal location. For instance, the commands below should move the robot in this order: 0 → 1 and then 1 → 5. For detailed explanation of problem statement checkout [this link](https://github.com/savnani5/Obstacle-Avoidance-System/blob/main/ENPM809E_RWA2_Spring2021.pdf).

![world](git_images/world.png)

  ```
  rosrun rwa2_savnani my_bot_controller 1
  ```
  ```
  rosrun rwa2_savnani my_bot_controller 5
  ```

### **turtlebot3_world**
![turtleBot3_world](git_images/sim.gif)

---

## Dependencies
- Ubuntu 18.04 (Operating System)
- Python 3.6.9
- CMake (Build System)
- ROS Melodic
- Gazebo
- RVIZ

---
## ROS Dependencies
- Rospy
- Tf
- sensor_msgs
- nav_msgs
- geometry_msgs


---
## Build Instructions
Follow the build instructions to build on your local system. 

- Make and initialize the catkin workspace.
  ```
  mkdir -p ~/catkin_ws/src
  catkin config --init
  ```

- Clone this repo and copy the rwa2_savnani folder to the src directory in catkin_ws. 
  ```
  git clone https://github.com/savnani5/Obstacle-Avoidance-System.git
  cd ~/catkin_ws/src
  cp -R ~/Obstacle-Avoidance-System/rwa2_savnani ~/catkin_ws/src
  ```

- Build the workspace
  ```
  catkin build
  source devel/setup.bash
  ```

---
## Run Instructions
- The *my_bot_controller.launch* file has all the launch parameters. Run the following command to spawn the robot at (-2,0) position.
  
  ```
  roslaunch rwa2_savnani my_bot_controller.launch
  ```

- Run the following command to make the robot navigate to position "**x**" in the map.

  ```
  rosrun rwa2_savnani my_bot_controller x
  ```
---
## Licence
    ```
    MIT License

    Copyright (c) 2021 Paras Savnani

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.

    ```
    
   
