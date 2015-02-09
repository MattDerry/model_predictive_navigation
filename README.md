# model_predictive_navigation
A sample ROS package that provides a model predictive equilibrium point control motion planner for the turtlebot simulator.

This work is an implementation of the model predictive equilibrium point control published by Jong Jin Park and Ben Kuipers of the University of Michigan. The relevant publications can be found in the literature folder of this repository.

## Dependencies
1) ROS Indigo (http://wiki.ros.org/indigo/Installation/Ubuntu)

2) ROS Turtlebot Simulator
```
sudo apt-get install ros-indigo-turtlebot-simulator
```
3) ROS Turtlebot Apps
```
sudo apt-get install ros-indigo-turtlebot-apps
```
4) FLANN - library for fast approximate nearest neighbors search
```
sudo apt-get install libflann-dev
```
5) NLopt - non-linear optimization library (http://ab-initio.mit.edu/wiki/index.php/NLopt)

## Install and build package
1) Clone model_predictive_navigation in to a ROS Indigo workspace

2) Build the workspace (planner works best if Release flag is used during build)
```
cd ~/<workspace_name>
catkin_make -DCMAKE_BUILD_TYPE=Release
```
3) Source the workspace
```
cd ~/<workspace_name>
source devel/setup.bash
```

## To Run
1) Run the included launch file:
```
roslaunch model_predictive_navigation mpepc_turtlebot_nav.launch
```
2) Rviz will automatically open, select "2D Nav Goal" button and place a goal pose in the scene

3) Turtlebot will begin navigating to desire goal pose

## Notes About Planner
The planner consists of two nodes `mpepc_trajectory` and `mpepc_plan`.  The goal request goes to `mpepc_trajectory` which then initiates the planner `mpepc_plan` to find subgoals online using a cost function optimization. The subgoals are published back to `mpepc_trajectory` which then uses a graceful control law to drive to the subgoal.  The planning time horizon is further than the replan rate, so the subgoal is not achieved until it is placed on the final goal.  
