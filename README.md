# How to use
## Compile and Setup
```
> source /opt/ros/noetic/setup.bash
> cd lab5_ws (or whatever ws you use)
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
> roslaunch ecse_373_ariac_ws competition.launch
```
## Close All
```
Kill both the node and the simulator:
> killall gzserver gzclient roslaunch
```

![image](file:///home/edninja/ecse_373_ariac_ws/src/cwru_ecse_373_submission/Screenshot_from_2022-12-09_18-41-03.png "Arm over Part")

## Theory of Operations
The goal of the project is to manipulate the arm in ARIAC by using utilizing trajectory matrices and waypoints. The ur kinematics package is additionally essential to moving the six joints. Using the transformation matrix, we also understand that two matrices exist: one that makes the vacuum gripper point sideways; the other raises the vacuum gripper's downward pointing. Additionally, let's assume that the code will reveal the arm and consequently ROS to pause before carrying out specific activities, and that we can since they are not as crucial theoretically, leave them out of the description. Defining our package dependencies comes first. We next proceed to define our rational cameras and start a ROS "trigger node" with an int and a char in our main function. Then, as we logical cameras and their vectors should be adjusted. Next, we initialize the transformation using the tf2. After setting up a while loop for if ROS = ok, we initialize the subscriber node. We possess subscription note watch out for a strong beginning and halt ROS IF it won't start, and if it does, we state that either the competition has begun successfully or it cannot. We then set up a spin up. The reason we do this is to call the callbacks and manage messages. Particularly, we are getting 8 solutions for the arm's joints will be established using inverse kinematics. Once ROS is OK, a second while loop is created and activated. As ROS continues to function, we utilize a single if statement to determine if a service order is present. Several if else statements are used to examine if the item is in the order or more. We are checking to see if our particular part is in any of the bins or the AGV trays. The part's posture is then determined using a for loop for future use. The best answer for q way indx and q sols indx must be found as part of finding solutions, but this feature might not have as much of an impact on the finished product. The final portion of our code addresses the issue of being able to go back and reverse our solutions. Last but not least, ROS waits for shutdown after finishing the tasks.

## Phases

If the arm exists, which it should, the arm must locate it, move the linear actuator, move the arm, place the vacuum gripper sufficiently close to the component, invert the waypoints to move the linear actuator to the AGV, move the waypoints to return to its initial position, move the to deposit the part and return to the original position, use a vacuum gripper to hold the part above the AGV tray position. The code should then determine if a shipment is finished. If so, the arm is free to determine if the order has been fulfilled or whether additional shipments are necessary. The function should end if the order is complete; if it is not, then, code should repeat the process for the following shipment. Incomplete shipments will result in the arm must carry out the aforementioned until the shipment is finished, at which point it must confirm that the order has been fulfilled, however another shipment still needs to be made. Last but not least, you must submit two orders, each with two shipments and various parts on each AGV. 
