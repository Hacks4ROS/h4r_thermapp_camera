/*
 * ThermAppRos.cpp
 *
 *  Created on: Apr 18, 2015
 *      Author: cyborg-x1
 */

#include "ThermAppRos.h"

namespace ThermApp {

ThermAppRos::ThermAppRos()
 :n(),
 nh("~"),
 it(n)
{
	int a;
	n.param("narf",a,1);
	pub_image=it.advertiseCamera("thermapp_camera/image", 1);
	reconfCbType = boost::bind(&ThermApp::ThermAppRos::reconfigCb, this, _1, _2);
	reconfServer.setCallback(reconfCbType);
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
	ros::Rate loop_rate(10);


	while(ros::ok())
	{

		ros::spinOnce();
		loop_rate.sleep();
	}
}




} /* namespace ThermApp */

