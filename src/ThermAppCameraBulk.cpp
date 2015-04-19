/*
 * ThermAppCameraBulk.cpp
 *
 *  Created on: Apr 18, 2015
 *      Author: cyborg-x1
 */

#include "thermapp_camera/ThermAppCameraBulk.h"

namespace ThermApp {

ThermAppCameraBulk::ThermAppCameraBulk()
:context(NULL),
 list(NULL),
 handle(NULL)
{
    libusb_init(&context);
    libusb_set_debug(context,3);
}

ThermAppCameraBulk::~ThermAppCameraBulk() {
	// TODO Auto-generated destructor stub
	this->closeDevice();
}

bool ThermAppCameraBulk::openDevice()
{
	if(dev_open)
	{
		cerr<<__PRETTY_FUNCTION__<<"There is already an open device!"<<endl;
		return false;
	}

	ssize_t count = libusb_get_device_list(context, &list);
    for (size_t idx = 0; idx < count; ++idx)
    {
        libusb_device *device = list[idx];
        struct libusb_device_descriptor desc = {0};

        struct libusb_device_descriptor  devDesc;


        libusb_get_device_descriptor(device, &desc);

        if(desc.idVendor==0x1772 && desc.idProduct==0x0002)
        {
        	int retVal;
            retVal=libusb_open(device,&handle);
            if(retVal<0)
            {
            	cerr<<__PRETTY_FUNCTION__<<"Found thermapp camera but was not able to open it."<<
            	endl<<"Device could be already in use,"<<
            	endl<<"or rights are missing to open it."<<endl;
            	handle=NULL;
            	continue;
            }

            retVal=libusb_claim_interface(handle,0);
            if(retVal<0)
            {
            	cerr<<__PRETTY_FUNCTION__<<"Found thermapp camera, opened it."<<
            	endl<<"But could not claim interface!"<<endl;
            	libusb_close(handle);
            	handle=NULL;
            	continue;
            }

            dev_open=true;
            return true;
        }
        dev_open=false;
        return false;
    }
}

void ThermAppCameraBulk::closeDevice()
{
	if(dev_open)
	{
		libusb_close(handle);
	}
}

void ThermAppCameraBulk::requestImage(cv::Mat& image)
{
	if(!dev_open)
	{
		cerr<<__PRETTY_FUNCTION__<<
		"Device not opened!"<<endl;
		return;
	}

    unsigned char ENDPOINT_DOWN = 0x81;
	unsigned char ENDPOINT_UP = 0x02;
	int actual_length;
	unsigned char dataUp[64];
	unsigned char dataDown[61440];


	memset(dataDown,0, sizeof(dataDown));
	memset(dataUp,0, sizeof(dataUp));

	libusb_bulk_transfer(handle, ENDPOINT_DOWN, dataDown,
								0, &actual_length, 10);

}



} /* namespace ThermApp */
