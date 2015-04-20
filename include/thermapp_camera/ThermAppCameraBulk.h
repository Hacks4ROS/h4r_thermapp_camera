/*
 * ThermAppCameraBulk.h
 *
 *  Created on: Apr 18, 2015
 *      Author: cyborg-x1
 */

#ifndef THERMAPP_CAMERA_SRC_THERMAPPCAMERABULK_H_
#define THERMAPP_CAMERA_SRC_THERMAPPCAMERABULK_H_

#include <cv.h>
#include <boost/thread.hpp>
#include <libusb-1.0/libusb.h>
#include <iostream>
#include <inttypes.h>


namespace ThermApp {
using namespace std;

class ThermAppCameraBulk
{

	enum
	{
	   ENDPOINT_DOWN = 0x81,
	   ENDPOINT_UP = 0x02,
	};

	bool dev_open;

	//Libusb variables
	libusb_context *context;
    libusb_device **list;
    libusb_device_handle *handle;

public:
	ThermAppCameraBulk();
	virtual ~ThermAppCameraBulk();

	bool openDevice();
	void closeDevice();
	void requestImage(cv::Mat& image);
};

} /* namespace ThermApp */

#endif /* THERMAPP_CAMERA_SRC_THERMAPPCAMERABULK_H_ */
