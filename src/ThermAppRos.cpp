/*
 * ThermAppRos.cpp
 *
 *  Created on: Apr 18, 2015
 *      Author: cyborg-x1
 */

#include "ThermAppRos.h"
#include <libusb-1.0/libusb.h>
#include <sensor_msgs/image_encodings.h>
#include <iomanip>      // std::setw


namespace ThermApp {

ThermAppRos::ThermAppRos()
 :n(),
 nh("~"),
 it(n)
{
	int a;
	n.param("narf",a,1);
	pub_cam_thermal=it.advertiseCamera("thermapp_camera/image", 1);
	reconfCbType = boost::bind(&ThermApp::ThermAppRos::reconfigCb, this, _1, _2);
	reconfServer.setCallback(reconfCbType);

	pub_debug_image=it.advertise("debugimage",1,1);
}

ThermAppRos::~ThermAppRos() {
}


void ThermAppRos::reconfigCb(thermapp_camera::thermapp_camera_nodeConfig &config, uint32_t level)
{
	reconf_mutex.lock();
	int temp_1=config.temp_1;
	int temp_2=config.temp_2;
	reconf_mutex.unlock();
}

void ThermAppRos::run()
{

	while(ros::ok())
	{
		reconf_mutex.lock();
		//Variables from dynamic reconfigure
		reconf_mutex.unlock();


		ros::spinOnce();
	}

}//RUN




} /* namespace ThermApp */

