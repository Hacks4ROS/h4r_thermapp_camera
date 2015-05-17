/*
 * ThermAppRos.h
 *
 *  Created on: Apr 18, 2015
 *      Author: cyborg-x1
 */

#ifndef THERMAPPROS_H_
#define THERMAPPROS_H_

#include <ros/ros.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <dynamic_reconfigure/server.h>

#include <h4r_thermapp_camera/h4r_thermapp_camera_nodeConfig.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <sensor_msgs/image_encodings.h>

#include "h4r_thermapp_camera/libthermapp.h"


namespace thermapp_camera {

class ThermAppRos {
private:

	boost::mutex mutex_reconf;

	//Configure Options

	//


	ros::NodeHandle n;
	ros::NodeHandle nh;

    image_transport::ImageTransport it;
    image_transport::CameraPublisher pub_cam;

    image_transport::Publisher pub_image;

    dynamic_reconfigure::Server<h4r_thermapp_camera::h4r_thermapp_camera_nodeConfig> reconfServer;
    dynamic_reconfigure::Server<h4r_thermapp_camera::h4r_thermapp_camera_nodeConfig>::CallbackType reconfCbType;

    //ThermAppCam Stuff
    ThermApp *therm;


    void getFrames();

public:
    void reconfigCb(h4r_thermapp_camera::h4r_thermapp_camera_nodeConfig &config, uint32_t level);
	ThermAppRos();
	virtual ~ThermAppRos();
	void run();
};

} /* namespace ThermApp */

#endif /* THERMAPPROS_H_ */
