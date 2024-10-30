##  	:notebook_with_decorative_cover: HOMEWORK 1
This repository has been created in order to fulfill the first homework of RoboticsLab 2024/2025 class. 

## :computer: Usage 
###  	:hammer: Build
First of all, clone this repo in your ros2_ws/src folder
```
git clone https://github.com/PasFar/homework1.git
```
Then, build the packages using the following command inside the ros2_ws folder and source the setup.bash file 
```
colcon build
. install/setup.bash
```
### :white_check_mark: How to run
Now you can run the nodes. You have different options:
 - Display the arm in Rviz
   ```
   ros2 launch arm_description display.launch.py
   ```
 - Load and spawn the arm in Gazebo
   ```
   ros2 launch arm_gazebo arm_world.launch.py
   ```
 - Spawn the arm in Gazebo with the position controller
   ```
   ros2 launch arm_gazebo arm_gazebo.launch.py
   ```
   After doing this, you can run the command publisher and the joint states subscriber node in a different terminal.
   ```
   ros2 run arm_controller arm_controller_node
   ```
   It is possible to change the default position command that will be published when running this node by changing the values in params.yaml, situated in the config folder of the arm_controller package.
   Moreover, it is possible to change the position command at runtime by running the following command in a new terminal:
   ```
   ros2 param set /arm_controller_node desired_param "[x.x, x.x, x.x, x.x]"
   ```
   Changing the values in the square brackets accordingly.
