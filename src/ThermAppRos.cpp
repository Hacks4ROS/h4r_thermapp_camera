/*
 * ThermAppRos.cpp
 *
 *  Created on: Apr 18, 2015
 *      Author: cyborg-x1
 */

#include "ThermAppRos.h"


namespace thermapp_camera {

ThermAppRos::ThermAppRos()
 :n(),
 nh("~"),
 it(n)
{


	pub_cam=it.advertiseCamera("thermapp_camera/image", 1);
	reconfCbType = boost::bind(&thermapp_camera::ThermAppRos::reconfigCb, this, _1, _2);
	reconfServer.setCallback(reconfCbType);

	pub_image=it.advertise("flir_image",1,1);
}

ThermAppRos::~ThermAppRos() {}


void ThermAppRos::reconfigCb(h4r_thermapp_camera::h4r_thermapp_camera_nodeConfig &config, uint32_t level)
{
	mutex_reconf.lock();

		therm->cfg->VoutA=config.voltage_A/2048*2.5;
		therm->cfg->VoutC=config.voltage_C/2048*2.5;
		therm->cfg->VoutD=config.voltage_D/2048*2.5;
		therm->cfg->VoutE=config.voltage_E/2048*2.5;

	mutex_reconf.unlock();
}

void ThermAppRos::getFrames()
{
	while(ros::ok())
	{
		//Create cv::bridge
		cv_bridge::CvImage image_brdg;
		image_brdg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
		image_brdg.header.frame_id="therm_app";

		//Create Mat for image
		cv::Mat thermal;

	    short frame[PIXELS_DATA_SIZE];
	    mutex_reconf.lock();
	    bool frame_ok=thermapp_GetImage(therm, frame);
	    mutex_reconf.unlock();

	    if(frame_ok)
	    {
	    	thermal=cv::Mat(cv::Size(384,288), CV_16UC1, &frame);
			image_brdg.image = thermal;
			image_brdg.header.stamp=ros::Time::now();
			image_brdg.header.seq++;

			pub_image.publish(image_brdg.toImageMsg());
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	    }


	}
}

void ThermAppRos::run()
{

    therm = thermapp_initUSB();
    if(therm == NULL) {
    	ROS_INFO("thermapp_initUSB Error");
    }

    if(thermapp_USB_checkForDevice(therm, VENDOR, PRODUCT) == -1)
    {
       ROS_ERROR("thermapp_USB_checkForDevice Error");
       exit(1);
    }else
    {
    	ROS_INFO("thermapp_FrameRequest_thread");
        thermapp_FrameRequest_thread(therm);
    }


    ROS_INFO("starting thermapp_FrameRequest_thread");
    //Run thread
    //thermapp_FrameRequest_thread(therm);

	 boost::thread* thread_frames = new boost::thread(boost::bind(&ThermAppRos::getFrames,this));


	while(ros::ok())
	{


		ros::spinOnce();
	}

	thread_frames->join();
	delete thread_frames;

}//RUN




} /* namespace ThermApp */

