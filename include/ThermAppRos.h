/*
 * ThermAppRos.h
 *
 *  Created on: Apr 18, 2015
 *      Author: cyborg-x1
 */

#ifndef THERMAPP_CAMERA_SRC_THERMAPPROS_H_
#define THERMAPP_CAMERA_SRC_THERMAPPROS_H_

#include <ros/ros.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <thermapp_camera/thermapp_camera_nodeConfig.h>
#include <boost/bind.hpp>
#include <thermapp_camera/ThermAppCameraBulk.h>

namespace ThermApp {

class ThermAppRos {
private:

	boost::mutex reconf_mutex;

	ros::NodeHandle n;
	ros::NodeHandle nh;

    image_transport::ImageTransport it;
    image_transport::CameraPublisher pub_image;

    dynamic_reconfigure::Server<thermapp_camera::thermapp_camera_nodeConfig> reconfServer;
    dynamic_reconfigure::Server<thermapp_camera::thermapp_camera_nodeConfig>::CallbackType reconfCbType;

public:
    void reconfigCb(thermapp_camera::thermapp_camera_nodeConfig &config, uint32_t level);
	ThermAppRos();
	virtual ~ThermAppRos();
	void run();
};

} /* namespace ThermApp */

#endif /* THERMAPP_CAMERA_SRC_THERMAPPROS_H_ */
