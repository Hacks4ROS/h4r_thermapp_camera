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
	ros::Rate loop_rate(8.7);

	ThermApp::ThermAppCameraBulk usb_bulk;
	if(usb_bulk.openDevice())
	{
		ROS_ERROR("Could not open device!");
		return;
	}


	while(ros::ok())
	{
		cv_bridge::CvImagePtr imgPtrThermal, imgPtrRGB;
		cv::Mat img_rgb;
		cv::Mat img_thermal;



		usb_bulk.requestImage(img_rgb);

//		sensor_msgs::Image image;
//		sensor_msgs::CameraInfo info;

		//img_thermal;



		try
		{
			imgPtrRGB->image = img_rgb;//cv_bridge::toCvCopy(depth_msg, "16UC1");
			//imgPtrThermal->image = img_thermal;// cv_bridge::toCvCopy(rgb_msg, "bgr8");
		} catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		//sensor_msgs::CameraInfo info;

		pub_debug_image.publish(imgPtrRGB->toImageMsg());
		//pub_image.publish(img_rgb,,ros::now());
		ros::spinOnce();
		loop_rate.sleep();
	}
}




} /* namespace ThermApp */

