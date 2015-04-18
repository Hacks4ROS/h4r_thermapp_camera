#include <ros/ros.h>
#include "ThermAppRos.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "thermapp_camera_node");

	ThermApp::ThermAppRos ThermApp;
	ThermApp.run();

	return 0;
}
