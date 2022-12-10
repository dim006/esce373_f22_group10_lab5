# How to use
## Compile and Setup
```
> source /opt/ros/noetic/setup.bash
> cd ecse_733_ariac_ws (or whatever ws you use)
> catkin_make
> source devel/setup.bash
```
# Options
## Launch ARIAC Competition
```
Make sure Ariac competition is running:
> roslaunch ecse_373_ariac ecse_373_ariac.launch
```
## Launch Node
```
Launches the specific node worked on during the lab:
> roslaunch ecse_373_araic_ws competition.launch
```
## Close All
```
Kill both the node and the simulator:
> killall gzserver gzclient roslaunch
```
