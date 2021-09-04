# Obstacle-Avoidance-System

## Overview


## Dependencies


## Build Instructions
Follow the build instructions to build on your local system. 
- Make and initialize the catkin workspace.
```
mkdir -p ~/catkin_ws/src
catkin config --init
```

- Clone this repo and copy 
git clone 


catkin build
```
---
## Run Instructions
 - The launch directory of the package `/rwa5_group_1` has a launch file `rwa5.launch`. All the necessary nodes will be launched within this node.

 ```
source ~/ariac_ws/devel/setup.bash

roslaunch rwa5_group_1 rwa5.launch load_moveit:=true
 ```

 - Run the rwa5 node 
 ```
source ~/ariac_ws/devel/setup.bash

rosrun rwa5_group_1 rwa5_node
 ``` 


---
 ## Output Video

An sample output video for the config file : [final_ariac_2021.yaml](./rwa5_group_1/config/final_ariac_2021.yaml) is shown below.

 [![alt text](./rwa5_group_1/docs/output_score/video_thumbnail.PNG?raw=true "Final Output Video")](https://www.youtube.com/watch?v=VHgZroqvAyw&ab_channel=RodrigoPerez)

An sample output video for the config file : [rwa5-sample.yaml](./rwa5_group_1/config/rwa5-sample.yaml) is shown below.

 [![alt text](./rwa5_group_1/docs/output_score/video_thumbnail.PNG?raw=true "Final Output Video")](https://www.youtube.com/watch?v=ij6FFzRd-K4&ab_channel=RodrigoPerez)





