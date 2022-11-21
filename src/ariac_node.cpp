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
//Desired joint trajectory
//trajectory_msgs::JointTrajectory desired;
//Desired joint end position
geometry_msgs::Pose desired_position;

//Order callback
void orderCallback(const osrf_gear::Order::ConstPtr &order)
{
	osrf_gear::Order o;
	o.order_id = order->order_id;
	o.shipments = order->shipments;
	order_vector.push_back(o);
}

//Camera callback
void cameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr &image, int index)
{
	osrf_gear::LogicalCameraImage i;
	i.models = image->models;
	i.pose = image->pose;
	image_vector[index] = i;
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
	T_des[2][3] = desired_position.position.z + 0.3;
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
	joint_trajectory.points[0].time_from_start = ros::Duration(0.0);
	// Must select which of the num_sols solutions to use. Just start with the first.
	int q_des_indx = 8;
	// Set the end point for the movement
	joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
	// Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution.
	joint_trajectory.points[1].positions[0] = joint_states.position[0];
	// The actuators are commanded in an odd order, enter the joint positions in the correct positions
	for (int indy = 0; indy < 6; indy++) {
		joint_trajectory.points[1].positions[indy + 1] = q_des[q_des_indx][indy];
	}
	// How long to take for the movement.
	joint_trajectory.points[1].time_from_start = ros::Duration(1.0);
	return joint_trajectory;
}

//Takes Pose data and stores it in a string
std::string pose2str(geometry_msgs::Pose pose)
{
	std::string posestring = "Position: x="+std::to_string(pose.position.x)+" y="+std::to_string(pose.position.y)+" z="+std::to_string(pose.position.z)+" Orientation: x="+std::to_string(pose.orientation.x)+" y="+std::to_string(pose.orientation.y)+" z="+std::to_string(pose.orientation.z)+" w="+std::to_string(pose.orientation.w);
	return posestring;
}

int main(int argc, char **argv)
{
	//Clears vectors
	order_vector.clear();
	image_vector.clear();
	//Initalize
	ros::init(argc, argv, "lab_5_node");
	//Starts node
	ros::NodeHandle n;
	//All Subscriptions
	ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("ariac/start_competition");
	ros::Subscriber order_subscriber = n.subscribe("ariac/orders",1000,orderCallback);
	ros::ServiceClient material_client = n.serviceClient<osrf_gear::GetMaterialLocations>("ariac/material_locations");
	ros::Subscriber joint_states_sub = n.subscribe("ariac/arm1/joint_states", 10, jointCB);
	//All Camera Subscriptions
	ros::Subscriber bin1_sub = n.subscribe("ariac/logical_camera_bin1",1,bin1Callback);
	ros::Subscriber bin2_sub = n.subscribe("ariac/logical_camera_bin2",1,bin2Callback);
	ros::Subscriber bin3_sub = n.subscribe("ariac/logical_camera_bin3",1,bin3Callback);
	ros::Subscriber bin4_sub = n.subscribe("ariac/logical_camera_bin4",1,bin4Callback);
	ros::Subscriber bin5_sub = n.subscribe("ariac/logical_camera_bin5",1,bin5Callback);
	ros::Subscriber bin6_sub = n.subscribe("ariac/logical_camera_bin6",1,bin6Callback);
	ros::Subscriber agv1_sub = n.subscribe("ariac/logical_camera_agv1",1,agv1Callback);
	ros::Subscriber agv2_sub = n.subscribe("ariac/logical_camera_agv2",1,agv2Callback);
	ros::Subscriber qc1_sub = n.subscribe("ariac/quality_control_sensor_1",1,qc1Callback);
	ros::Subscriber qc2_sub = n.subscribe("ariac/quality_control_sensor_2",1,qc2Callback);
	//Publishes joint trajectory to the arm1 command
	ros::Publisher joint_trajectory_pub = n.advertise<trajectory_msgs::JointTrajectory>("ariac/arm1/arm/command",10);

	//Current default location, will be updated later to be automatic based on part location
	desired_position.position.x = 1;
	desired_position.position.y = 1;
	desired_position.position.z = 1;

	//Tracks if competition has begun
	service_call_succeeded = begin_client.call(begin_comp);
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

	ros::Rate loop_rate(10);

  while(ros::ok())
	{
		loop_rate.sleep();

		//If we have orders
		if(order_vector.size() > 0)
		{
			material_location.request.material_type = order_vector.front().shipments.front().products.front().type;
			location_call_succeeded = material_client.call(material_location);
			ROS_INFO("The object is type: %s", material_location.request.material_type.c_str());
			ROS_INFO("The storage unit containing this object: %s", material_location.response.storage_units.front().unit_id.c_str());
			order_vector.clear();
			std::string product_type = material_location.request.material_type;
			for(int i = 0; i < 6; i++)
			{
				for(int j = 0; j < image_vector[i].models.size(); j++)
				{
					if(image_vector[i].models[j].type == product_type)
					{
						ROS_WARN("Product type: %s, Bin: %s, Pose: %s", product_type.c_str(), std::to_string(i+1).c_str(), pose2str(image_vector[i].models[j].pose).c_str());
					}
				}
			}
			//Calls joint trajectory to determine new position based on the set desired position. Currently set by default to 1,1,1
			jointTrajectory();
			//Publishes the new location to gazebo and executes the move
			joint_trajectory_pub.publish(joint_trajectory);
		}
		ros::spinOnce();
	}
  return 0;
}