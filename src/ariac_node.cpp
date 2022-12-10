#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/StorageUnit.h"
#include "osrf_gear/Order.h"
#include "ur_kinematics/ur_kin.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"

#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

#include "control_msgs/FollowJointTrajectoryAction.h"

//Global variables
//Begins competition
std_srvs::Trigger begin_comp; 
//Tracks Service call successes
int service_call_succeeded;
//Order vector
std::vector<osrf_gear::Order> order_vector;
//Material Locations
osrf_gear::GetMaterialLocations material_location;
//Tracks location call successes
int location_call_succeeded;
//Storage locations
osrf_gear::StorageUnit location;
//Camera image vectors
std::vector<osrf_gear::LogicalCameraImage> image_vector(10); 
//Stores the joint states of arm 1
sensor_msgs::JointState joint_states;
// Declare a variable for generating and publishing a trajectory.
trajectory_msgs::JointTrajectory joint_trajectory;
//Create variables for use with Kinematics
double T_pose[4][4], T_des[4][4];
double q_pose[6], q_des[8][6];
//logical camera fram to arm transform checker
geometry_msgs::TransformStamped tfStamped;
//Desired joint end position
geometry_msgs::Pose desired_position;
//Transformation Buffer
tf2_ros::Buffer tfBuffer;
//Stores transformed part location
geometry_msgs::PoseStamped arm_part_pose;
geometry_msgs::PoseStamped part_position_pose;
//AS?
control_msgs::FollowJointTrajectoryAction joint_trajectory_as;
//Product type of material
std::string product_type;
//Position vector of all part locations
std::vector<geometry_msgs::Pose> position_vector;
//Frame vector of part locations, corresponds with position_vector
std::vector<std::string> camera_frame_vector;
//For loop variables
int i;
int j;

//Order callback
void orderCallback(const osrf_gear::Order::ConstPtr &order)
{
	order_vector.push_back(*order);
}

//Camera callback
void cameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr &image, int index)
{
	image_vector[index] = *image;
}

//Bin1 Camera Callback
void bin1Callback(const osrf_gear::LogicalCameraImage::ConstPtr &image)
{
	cameraCallback(image,0);
}

//Bin2 Camera Callback
void bin2Callback(const osrf_gear::LogicalCameraImage::ConstPtr &image)
{
	cameraCallback(image,1);
}

//Bin3 Camera Callback
void bin3Callback(const osrf_gear::LogicalCameraImage::ConstPtr &image)
{
	cameraCallback(image,2);
}

//Bin4 Camera Callback
void bin4Callback(const osrf_gear::LogicalCameraImage::ConstPtr &image)
{
	cameraCallback(image,3);
}

//Bin5 Camera Callback
void bin5Callback(const osrf_gear::LogicalCameraImage::ConstPtr &image)
{
	cameraCallback(image,4);
}

//Bin6 Camera Callback
void bin6Callback(const osrf_gear::LogicalCameraImage::ConstPtr &image)
{
	cameraCallback(image,5);
}

//AGV1 Camera Callback
void agv1Callback(const osrf_gear::LogicalCameraImage::ConstPtr &image)
{
	cameraCallback(image,6);
}

//AGV2 Camera Callback
void agv2Callback(const osrf_gear::LogicalCameraImage::ConstPtr &image)
{
	cameraCallback(image,7);
}

//QC1 Camera Callback
void qc1Callback(const osrf_gear::LogicalCameraImage::ConstPtr &image)
{
	cameraCallback(image,8);
}

//QC2 Camera Callback
void qc2Callback(const osrf_gear::LogicalCameraImage::ConstPtr &image)
{
	cameraCallback(image,9);
}

//Joint state callback
void jointCB(const sensor_msgs::JointState::ConstPtr &joint_msgs)
{
	joint_states.header = joint_msgs->header;
	joint_states.name = joint_msgs->name;
	joint_states.position = joint_msgs->position;
	joint_states.velocity = joint_msgs->velocity;
	joint_states.effort = joint_msgs->effort;
}

//Gets all the positions of product_type and stores them in position_vector
std::vector<geometry_msgs::Pose> productPosition(std::string product_type)
{
	for(i = 0; i < image_vector.size(); i++)
			{
				for(j = 0; j < image_vector[i].models.size(); j++)
				{
					if(image_vector[i].models[j].type == product_type)
					{
						position_vector.push_back(image_vector[i].models[j].pose);
						std::string temp = "logical_camera_bin" + std::to_string(i+1) + "_frame";
						camera_frame_vector.push_back(temp);
						ROS_WARN("position_vector size is %s", std::to_string(position_vector.size()).c_str());
					}
				}
			}
	for(i = 0; i < camera_frame_vector.size(); i++)
	{
		ROS_WARN("camera_frame_vector value is %s", camera_frame_vector[i].c_str());
	}
	return position_vector;
}

//TF2 transform from Camera to Arm positions
geometry_msgs::PoseStamped cameraConvert(geometry_msgs::PoseStamped camera_part_pose)
{
	try {
		tfStamped = tfBuffer.lookupTransform("arm1_base_link", camera_frame_vector[0], ros::Time(0.0), ros::Duration(1.0));
		ROS_DEBUG("Transform from [%s] to [%s]", tfStamped.child_frame_id.c_str(), tfStamped.header.frame_id.c_str());
	}
	catch(tf2::TransformException &ex) {
		ROS_ERROR("%s", ex.what());
	}

	tf2::doTransform(camera_part_pose, arm_part_pose, tfStamped);

	arm_part_pose.pose.position.z += .1;
	arm_part_pose.pose.orientation.w = .707;
	arm_part_pose.pose.orientation.x = 0;
	arm_part_pose.pose.orientation.y = .707;
	arm_part_pose.pose.orientation.z = 0;

	return arm_part_pose;
}

//Creates a Joint Trajectory based on current and desired location
trajectory_msgs::JointTrajectory jointTrajectory()
{
	q_pose[0] = joint_states.position[1];
	q_pose[1] = joint_states.position[2];
	q_pose[2] = joint_states.position[3];
	q_pose[3] = joint_states.position[4];
	q_pose[4] = joint_states.position[5];
	q_pose[5] = joint_states.position[6];
		
	ur_kinematics::forward((double *)&q_pose, (double *)&T_pose);

	// Desired pose of the end effector wrt the base_link.
	T_des[0][3] = desired_position.position.x;
	T_des[1][3] = desired_position.position.y;
	T_des[2][3] = desired_position.position.z;
	T_des[3][3] = 1.0;
	// The orientation of the end effector so that the end effector is down.
	T_des[0][0] = 0.0; T_des[0][1] = -1.0; T_des[0][2] = 0.0;
	T_des[1][0] = 0.0; T_des[1][1] = 0.0; T_des[1][2] = 1.0;
	T_des[2][0] = -1.0; T_des[2][1] = 0.0; T_des[2][2] = 0.0;
	T_des[3][0] = 0.0; T_des[3][1] = 0.0; T_des[3][2] = 0.0;
		
	int num_sols = ur_kinematics::inverse((double *)&T_des, (double *)&q_des);

	// Fill out the joint trajectory header.
	// Each joint trajectory should have an non-monotonically increasing sequence number.
	int count = 0;
	joint_trajectory.header.seq = count++;
	joint_trajectory.header.stamp = ros::Time::now(); // When was this message created.
	joint_trajectory.header.frame_id = "/base_link"; // Frame in which this is specified.
	// Set the names of the joints being used. All must be present.
	joint_trajectory.joint_names.clear();
	joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
	joint_trajectory.joint_names.push_back("shoulder_pan_joint");
	joint_trajectory.joint_names.push_back("shoulder_lift_joint");
	joint_trajectory.joint_names.push_back("elbow_joint");
	joint_trajectory.joint_names.push_back("wrist_1_joint");
	joint_trajectory.joint_names.push_back("wrist_2_joint");
	joint_trajectory.joint_names.push_back("wrist_3_joint");

	// Set a start and end point.
	joint_trajectory.points.resize(2);
	// Set the start point to the current position of the joints from joint_states.	
	joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
	for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++) {
		for (int indz = 0; indz < joint_states.name.size(); indz++) {
			if (joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
				joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
				break;
			}
		}
	}
	// When to start (immediately upon receipt).
	joint_trajectory.points[0].time_from_start = ros::Duration(0.0);
	// Must select which of the num_sols solutions to use. Just start with the first.
	int q_des_indx = 0;
	for(i = 0; i < 8; i++)
	{
		if(q_des[i][1] > 3.141526 && q_des[i][3] > 3.141526 && q_des[i][2] < 3.141526)
		{
			q_des_indx = i;
			break;
		}
	}
	// Set the end point for the movement
	joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
	// Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution.
	joint_trajectory.points[1].positions[0] = joint_states.position[1];
	// The actuators are commanded in an odd order, enter the joint positions in the correct positions
	for (int indy = 0; indy < 6; indy++) {
		joint_trajectory.points[1].positions[indy + 1] = q_des[q_des_indx][indy];
	}
	// How long to take for the movement.
	joint_trajectory.points[1].time_from_start = ros::Duration(1.0);
	return joint_trajectory;
}

//Takes Pose data and stores it in a string
std::string pose2String(geometry_msgs::Pose pose)
{
	std::string posestring = "Position: x="+std::to_string(pose.position.x)+" y="+std::to_string(pose.position.y)+" z="+std::to_string(pose.position.z)+" Orientation: x="+std::to_string(pose.orientation.x)+" y="+std::to_string(pose.orientation.y)+" z="+std::to_string(pose.orientation.z)+" w="+std::to_string(pose.orientation.w);
	return posestring;
}

std::string joint2String(sensor_msgs::JointState joint, int a)
{
	std::string joint_string = "Position: "+std::to_string(joint.position[a]);
	return joint_string;
}

int main(int argc, char **argv)
{
	//Clears vectors
	order_vector.clear();
	// image_vector.clear();
	//Initalize
	ros::init(argc, argv, "lab_5_node");
	//Starts node
	ros::NodeHandle n;
	//Creates a listener for tfBuffer
	tf2_ros::TransformListener tfListener(tfBuffer);
	//All Subscriptions
	ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("ariac/start_competition");
	ros::Subscriber order_subscriber = n.subscribe("ariac/orders",1000,orderCallback);
	ros::ServiceClient material_client = n.serviceClient<osrf_gear::GetMaterialLocations>("ariac/material_locations");
	ros::Subscriber joint_states_sub = n.subscribe("ariac/arm1/joint_states", 10, jointCB);
	//All Camera Subscriptions
	ros::Subscriber bin1_sub = n.subscribe("ariac/logical_camera_bin1",10,bin1Callback);
	ros::Subscriber bin2_sub = n.subscribe("ariac/logical_camera_bin2",10,bin2Callback);
	ros::Subscriber bin3_sub = n.subscribe("ariac/logical_camera_bin3",10,bin3Callback);
	ros::Subscriber bin4_sub = n.subscribe("ariac/logical_camera_bin4",10,bin4Callback);
	ros::Subscriber bin5_sub = n.subscribe("ariac/logical_camera_bin5",10,bin5Callback);
	ros::Subscriber bin6_sub = n.subscribe("ariac/logical_camera_bin6",10,bin6Callback);
	ros::Subscriber agv1_sub = n.subscribe("ariac/logical_camera_agv1",10,agv1Callback);
	ros::Subscriber agv2_sub = n.subscribe("ariac/logical_camera_agv2",10,agv2Callback);
	ros::Subscriber qc1_sub = n.subscribe("ariac/quality_control_sensor_1",10,qc1Callback);
	ros::Subscriber qc2_sub = n.subscribe("ariac/quality_control_sensor_2",10,qc2Callback);
	//Publishes joint trajectory to the arm1 command
	ros::Publisher joint_trajectory_pub = n.advertise<trajectory_msgs::JointTrajectory>("ariac/arm1/arm/command",10);
	//Current default location, will be updated later to be automatic based on part location
	desired_position.position.x = 1;
	desired_position.position.y = 1;
	desired_position.position.z = 1;
	double currentTime;
	//Tracks if competition has begun
	service_call_succeeded = begin_client.call(begin_comp);

	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_as("ariac/arm1/arm/follow_joint_trajectory", true);
	trajectory_as.waitForServer();

	//If there is no competition
	if(!service_call_succeeded) {
		ROS_ERROR("Competition service call failed! Make sure you have started the ariac simulation!");
	}
	else {
		//If the competition is started
		if(begin_comp.response.success) {
			ROS_INFO("Competition service started successfully: %s", begin_comp.response.message.c_str());
		}
		//Competition error
		else {
			ROS_WARN("Competition service returned failure: %s", begin_comp.response.message.c_str());
		}
	}

	ros::AsyncSpinner spinner(4);
	spinner.start();
	ros::Duration(1.0).sleep();

  while(ros::ok())
	{

		//If we have orders
		if(order_vector.size() > 0)
		{
			//Current time in seconds from start
			currentTime = ros::Time::now().toSec();
			//Gives current joint positions and time
			ROS_INFO_THROTTLE(10,"Current Joint States: 0: %s, 1: %s, 2: %s, 3: %s, 4: %s, 5: %s, 6: %s; Time: %f", joint2String(joint_states, 0).c_str(), joint2String(joint_states, 1).c_str(), joint2String(joint_states, 2).c_str(), joint2String(joint_states, 3).c_str(), joint2String(joint_states, 4).c_str(), joint2String(joint_states, 5).c_str(), joint2String(joint_states, 6).c_str(), currentTime);
			//Gets current order object type (turn into order loop later)
			material_location.request.material_type = order_vector.front().shipments.front().products.front().type;
			//Checks if location call worked
			location_call_succeeded = material_client.call(material_location);
			//Gives current order object type (make into loop later)
			ROS_INFO("The object is type: %s", material_location.request.material_type.c_str());
			//Create loop here to check for all storage units w/ object
			ROS_INFO("The storage unit containing this object: %s", material_location.response.storage_units.front().unit_id.c_str());
			product_type = material_location.request.material_type;
			//ROS_INFO("Bin %i has %li", (3+1), image_vector[3].models.size());
			for(i = 0; i < image_vector.size(); i++)
			{
				//ROS_INFO_THROTTLE(0, "Image # %i", i);
				for(j = 0; j < image_vector[i].models.size(); j++)
				{
					//ROS_INFO_THROTTLE(0, "Model # %i", j);
					if(image_vector[i].models[j].type == product_type)
					{
						ROS_WARN("Product type: %s, Bin: %s, %s", product_type.c_str(), std::to_string(i+1).c_str(), pose2String(image_vector[i].models[j].pose).c_str());
					}
				}
			}
			//Calls joint trajectory to determine new position based on the set desired position. Currently set by default to 1,1,1
			//jointTrajectory();
			//Publishes the new location to gazebo and executes the move
			//joint_trajectory_pub.publish(joint_trajectory);
			productPosition(product_type);
			while(position_vector.size() > 0)
			{
				part_position_pose.pose = position_vector[0];
				cameraConvert(part_position_pose);
				desired_position = arm_part_pose.pose;
				jointTrajectory();
				//joint_trajectory_pub.publish(joint_trajectory);
				joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
				actionlib::SimpleClientGoalState state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
				ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
				position_vector.erase(position_vector.begin());
				camera_frame_vector.erase(camera_frame_vector.begin());
				ros::Duration(1.5).sleep();
			}
		}
		ros::Duration(0.1).sleep();
	}
  return 0;
}