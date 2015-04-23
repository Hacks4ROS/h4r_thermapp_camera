/*
 * ThermAppRos.cpp
 *
 *  Created on: Apr 18, 2015
 *      Author: cyborg-x1
 */

#include "ThermAppRos.h"
#include <libusb-1.0/libusb.h>
#include <sensor_msgs/image_encodings.h>
#include <thermapp_camera/libthermapp.h>


namespace thermapp_camera {

ThermAppRos::ThermAppRos()
 :n(),
 nh("~"),
 it(n)
{
	//int a;
	//nh.param("narf",a,1);


	pub_cam=it.advertiseCamera("thermapp_camera/image", 1);
	reconfCbType = boost::bind(&thermapp_camera::ThermAppRos::reconfigCb, this, _1, _2);
	reconfServer.setCallback(reconfCbType);

	pub_image=it.advertise("flir_image",1,1);
}

ThermAppRos::~ThermAppRos() {}


void ThermAppRos::reconfigCb(thermapp_camera::thermapp_camera_nodeConfig &config, uint32_t level)
{
	reconf_mutex.lock();
	int temp_1=config.temp_1;
	int temp_2=config.temp_2;
	reconf_mutex.unlock();
}

void ThermAppRos::run()
{
	//Create cv::bridge
	cv_bridge::CvImage image_brdg;
	image_brdg.encoding = sensor_msgs::image_encodings::BGR8;
	image_brdg.header.frame_id="therm_app";

	//Create Mat for image
	cv::Mat img;


    therm = thermapp_initUSB();
    if(therm == NULL) {
    	ROS_INFO("thermapp_initUSB Error");
    }

    if(thermapp_USB_checkForDevice(therm, VENDOR, PRODUCT) == -1){
       ROS_ERROR("thermapp_USB_checkForDevice Error");
    }



    ROS_INFO("starting thermapp_FrameRequest_thread");
    //Run thread
    thermapp_FrameRequest_thread(therm);


	while(ros::ok())
	{
		reconf_mutex.lock();
		//Variables from dynamic reconfigure
		reconf_mutex.unlock();

		image_brdg.image    = img; // Your cv::Mat
		image_brdg.header.stamp=ros::Time::now();
		image_brdg.header.seq++;

		pub_image.publish(image_brdg.toImageMsg());

		ros::spinOnce();
	}

}//RUN




} /* namespace ThermApp */

