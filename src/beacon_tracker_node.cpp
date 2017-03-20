#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include "BeaconLocator.hpp"

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "beacon_tracker_node");
	ros::NodeHandle nh;
	BeaconLocator locator
	(
			&nh,
			"/zed/rgb/image_raw_color",
			"/zed/point_cloud/cloud_registered",
			"/zed/depth/depth_registered",
			"/beacon1",
			"/beacon2"
	 );


	ros::spin() ;
}
