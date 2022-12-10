#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/StorageUnit.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/AGVControl.h"
#include "ur_kinematics/ur_kin.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"

#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

#include "control_msgs/FollowJointTrajectoryAction.h"

#include "osrf_gear/VacuumGripperControl.h"
#include "osrf_gear/VacuumGripperState.h"

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
geometry_msgs::Pose position_vector;
//Frame vector of part locations, corresponds with position_vector
std::string camera_frame_vector;
//Stores the gripper state
osrf_gear::VacuumGripperState gripper_state;
//Stores gripper control
osrf_gear::VacuumGripperControl gripper_control;
//Stores agv control
osrf_gear::AGVControl agv_command;
bool agv_response;
//Stores if gripper should be on/off
bool gripper_success;
//For loop variables
int i;
int j;
float arm_base_location;
int count = 0;

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
	joint_states = *joint_msgs;
}

//Gripper callback
void gripperCallback(const osrf_gear::VacuumGripperState::ConstPtr &state)
{
	gripper_state = *state;
}

//Gets all the positions of product_type and stores them in position_vector
geometry_msgs::Pose productPosition()
{
	for(i = 0; i < image_vector.size(); i++)
			{
				for(j = 0; j < image_vector[i].models.size(); j++)
				{
					if(image_vector[i].models[j].type == product_type)
					{
						position_vector = image_vector[i].models[j].pose;
						std::string temp = "logical_camera_bin" + std::to_string(i+1) + "_frame";
						camera_frame_vector = temp;
						i = 1000;
						break;
					}
				}
			}
	return position_vector;
}

//TF2 transform from Camera to Arm positions
geometry_msgs::PoseStamped cameraConvert()
{
	//"arm1_base_link"
	try {
		tfStamped = tfBuffer.lookupTransform("arm1_base_link", camera_frame_vector, ros::Time(0.0), ros::Duration(7.5));
		ROS_WARN("Transform from [%s] to [%s]", tfStamped.child_frame_id.c_str(), tfStamped.header.frame_id.c_str());
	}
	catch(tf2::TransformException &ex) {
		ROS_ERROR("%s", ex.what());
	}


	tf2::doTransform(part_position_pose, arm_part_pose, tfStamped);


	arm_part_pose.pose.position.z += 0.02;
	//arm_part_pose.pose.orientation.w = .707;
	//arm_part_pose.pose.orientation.x = 0.0;
	//arm_part_pose.pose.orientation.y = .707;
	//arm_part_pose.pose.orientation.z = 0.0;

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
	joint_trajectory.header.seq = count++;
	joint_trajectory.header.stamp = ros::Time::now(); // When was this message created.
	joint_trajectory.header.frame_id = "/world"; // Frame in which this is specified.
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
	joint_trajectory.points[0].time_from_start = ros::Duration(0.1);
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
	joint_trajectory.points[1].positions[0] = arm_base_location;
	// The actuators are commanded in an odd order, enter the joint positions in the correct positions
	for (int indy = 0; indy < 6; indy++) {
		joint_trajectory.points[1].positions[indy + 1] = q_des[q_des_indx][indy];
	}
	// How long to take for the movement.
	joint_trajectory.points[1].time_from_start = ros::Duration(7.5);
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
	ros::Subscriber joint_states_sub = n.subscribe("ariac/arm1/joint_states", 10, jointCB);
	ros::ServiceClient gripper_client = n.serviceClient<osrf_gear::VacuumGripperControl>("ariac/arm1/gripper/control");
	ros::Subscriber gripper_state_sub = n.subscribe("ariac/arm1/gripper/state", 10, gripperCallback);
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
	ros::ServiceClient agv1_client = n.serviceClient<osrf_gear::AGVControlRequest>("ariac/agv1");
	ros::ServiceClient agv2_client = n.serviceClient<osrf_gear::AGVControlRequest>("ariac/agv2");
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
			//Full logic for determining orders, shipments, and parts; how we pick them up; and how we deliver them
			while(order_vector.size() > 0)
			{
				for(int idx = 0; idx < order_vector.size(); idx++)
				{
					ROS_WARN("Order Vector Size is %s", std::to_string(order_vector.size()).c_str());
					for(int jdx = 0; jdx < order_vector[idx].shipments.size(); jdx++)
					{
						ROS_WARN("Order Vector Shipment Size is %s", std::to_string(order_vector[idx].shipments.size()).c_str());
						for(int kdx = 0; kdx < order_vector[idx].shipments[jdx].products.size(); kdx++)
						{
							ROS_INFO_THROTTLE(10,"Current Joint States: 0: %s, 1: %s, 2: %s, 3: %s, 4: %s, 5: %s, 6: %s; Time: %f", joint2String(joint_states, 0).c_str(), joint2String(joint_states, 1).c_str(), joint2String(joint_states, 2).c_str(), joint2String(joint_states, 3).c_str(), joint2String(joint_states, 4).c_str(), joint2String(joint_states, 5).c_str(), joint2String(joint_states, 6).c_str(), currentTime);
							ROS_WARN("Order Vector Products Size is %s", std::to_string(order_vector[idx].shipments[jdx].products.size()).c_str());
							//Gets part type
							material_location.request.material_type = order_vector[idx].shipments[jdx].products[kdx].type;
							ROS_WARN("Shipment type: %s, AGV_ID %s", order_vector[idx].shipments[jdx].shipment_type.c_str(), order_vector[idx].shipments[jdx].agv_id.c_str());
							//Sets product_type equal to the requested material type
							product_type = material_location.request.material_type;
							//Gets one position of requested product type
							productPosition();
							desired_position.position.x = -.4;
							desired_position.position.y = .2;
							desired_position.position.z = .1;
							if(product_type == "piston_rod_part")
							{
								arm_base_location = -.1;
							}	
							if(product_type == "gear_part")
							{
								arm_base_location = 1.6;
							}
							jointTrajectory();
							joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
							//Publishes joint trajectory to action server
							actionlib::SimpleClientGoalState state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
							//Returns action server SUCCESS/ABORT
							ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
							ros::Duration(1.0).sleep();
							//Converts pose to pose stamped for part position
							part_position_pose.pose = position_vector;
							//TF2 convert part coords from logical camera to arm base, sets +.1 cm above desired part
							cameraConvert();
							//Sets new coords as the desired arm end point
							desired_position = arm_part_pose.pose;
							desired_position.position.y += .02;
							//Runs math to find arm pathing solution
							jointTrajectory();
							//Sets action server goal as the discovered joint trajectory
							joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
							//Publishes joint trajectory to action server
							state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
							//Returns action server SUCCESS/ABORT
							ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
							//Waits for a few seconds
							ros::Duration(1.0).sleep();
							//Turn on vacuum gripper
							while(gripper_state.attached != true)
							{
								gripper_control.request.enable = true;
								gripper_client.call(gripper_control);
								ros::Duration(.5).sleep();
							}
							//Go to home point
							desired_position.position.x = -.4;
							desired_position.position.y = .2;
							desired_position.position.z = .1;
							jointTrajectory();
							joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
							//Publishes joint trajectory to action server
							state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
							//Returns action server SUCCESS/ABORT
							ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
							ros::Duration(1.0).sleep();
							//Sets AGV as new desired drop off point
							if(order_vector[idx].shipments[jdx].agv_id == "agv1" || order_vector[idx].shipments[jdx].agv_id == "any")
							{
								arm_base_location = 1.99;
								jointTrajectory();
								joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
								//Publishes joint trajectory to action server
								state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
								//Returns action server SUCCESS/ABORT
								ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
								ros::Duration(1.0).sleep();
								part_position_pose.pose = order_vector[idx].shipments[jdx].products[kdx].pose;
								camera_frame_vector = "kit_tray_1";
								cameraConvert();
								desired_position = arm_part_pose.pose;
								desired_position.position.z += .1;
								jointTrajectory();
								joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
								//Publishes joint trajectory to action server
								actionlib::SimpleClientGoalState state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
								//Returns action server SUCCESS/ABORT
								ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
								ros::Duration(1.0).sleep();
								//Turn off vacuum gripper
								while(gripper_state.attached != false)
								{
									gripper_control.request.enable = false;
									gripper_client.call(gripper_control);
									ros::Duration(.5).sleep();
								}
							}
							if(order_vector[idx].shipments[jdx].agv_id == "agv2")
							{
								arm_base_location = -1.99;
								jointTrajectory();
								joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
								//Publishes joint trajectory to action server
								state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
								//Returns action server SUCCESS/ABORT
								ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
								ros::Duration(1.0).sleep();
								part_position_pose.pose = order_vector[idx].shipments[jdx].products[kdx].pose;
								camera_frame_vector = "kit_tray_2";
								cameraConvert();
								desired_position = arm_part_pose.pose;
								desired_position.position.z += .1;
								jointTrajectory();
								joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
								//Publishes joint trajectory to action server
								actionlib::SimpleClientGoalState state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
								//Returns action server SUCCESS/ABORT
								ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
								ros::Duration(1.0).sleep();
								while(gripper_state.attached != false)
								{
									gripper_control.request.enable = false;
									gripper_client.call(gripper_control);
									ros::Duration(.5).sleep();
								}
								desired_position.position.x = -.4;
								desired_position.position.y = .2;
								desired_position.position.z = .1;
								arm_base_location = -1.99;
								jointTrajectory();
								joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
								state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
								//Returns action server SUCCESS/ABORT
								ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
								ros::Duration(1.0).sleep();
							}
							//Check if shipment should be submitted
							if((kdx + 1) == order_vector[idx].shipments[jdx].products.size())
							{
								//Submit Order AGV1
								if(order_vector[idx].shipments[jdx].agv_id == "agv1" || order_vector[idx].shipments[jdx].agv_id == "any")
								{
									agv_command.request.shipment_type = order_vector[idx].shipments[jdx].shipment_type;
									agv_response = agv1_client.call(agv_command);
									ROS_WARN("AGV response is: %s", std::to_string(agv_response).c_str());
									ros::Duration(1.0).sleep();
								}
								//Submit Order AGV2
								if(order_vector[idx].shipments[jdx].agv_id == "agv2")
								{
									agv_command.request.shipment_type = order_vector[idx].shipments[jdx].shipment_type;
									agv_response = agv2_client.call(agv_command);
									ROS_WARN("AGV response is: %s", std::to_string(agv_response).c_str());
									ros::Duration(1.0).sleep();
								}
							}

						}
					}
				}
			}
		}
		ros::Duration(.5).sleep();
	}
  return 0;
}