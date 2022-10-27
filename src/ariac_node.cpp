#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/StorageUnit.h"
#include "osrf_gear/Order.h"

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

//Order callback
void orderCallback(const osrf_gear::Order::ConstPtr& order)
{
	osrf_gear::Order o;
	o.order_id = order->order_id;
	o.shipments = order->shipments;
	order_vector.push_back(o);
}

//Camera callback
void cameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& image, int index)
{
	osrf_gear::LogicalCameraImage i;
	i.models = image->models;
	i.pose = image->pose;
	image_vector[index] = i;
}

//Bin1 Camera Callback
void bin1Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image)
{
	cameraCallback(image,0);
}

//Bin2 Camera Callback
void bin2Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image)
{
	cameraCallback(image,1);
}

//Bin3 Camera Callback
void bin3Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image)
{
	cameraCallback(image,2);
}

//Bin4 Camera Callback
void bin4Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image)
{
	cameraCallback(image,3);
}

//Bin5 Camera Callback
void bin5Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image)
{
	cameraCallback(image,4);
}

//Bin6 Camera Callback
void bin6Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image)
{
	cameraCallback(image,5);
}

//AGV1 Camera Callback
void agv1Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image)
{
	cameraCallback(image,6);
}

//AGV2 Camera Callback
void agv2Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image)
{
	cameraCallback(image,7);
}

//QC1 Camera Callback
void qc1Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image)
{
	cameraCallback(image,8);
}

//QC2 Camera Callback
void qc2Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image)
{
	cameraCallback(image,9);
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
		}
		ros::spinOnce();
	}
  return 0;
}