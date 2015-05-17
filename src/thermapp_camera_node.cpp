#include <ros/ros.h>
#include "ThermAppRos.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "h4r_thermapp_camera_node");

	thermapp_camera::ThermAppRos node;
	node.run();

	return 0;
}
