/*
 * ThermAppCameraBulk.cpp
 *
 *  Created on: Apr 18, 2015
 *      Author: cyborg-x1
 */

#include "thermapp_camera/ThermAppCameraBulk.h"


namespace ThermApp {

ThermAppCameraBulk::ThermAppCameraBulk()
:dev_open(false),
 context(NULL),
 list(NULL),
 handle(NULL)
{

}

ThermAppCameraBulk::~ThermAppCameraBulk() {
	// TODO Auto-generated destructor stub
	this->closeDevice();
}

bool ThermAppCameraBulk::openDevice()
{

    libusb_init(&context);
    libusb_set_debug(context,3);
	if(dev_open)
	{
		cerr<<__PRETTY_FUNCTION__<<"There is already an open device!"<<endl;
		return false;
	}

	ssize_t count = libusb_get_device_list(context, &list);
    for (size_t idx = 0; idx < count; ++idx)
    {
    	cout<<"..."<<endl;
        libusb_device *device = list[idx];
        struct libusb_device_descriptor desc = {0};

        struct libusb_device_descriptor  devDesc;


        libusb_get_device_descriptor(device, &desc);

        if(desc.idVendor==0x1772 && desc.idProduct==0x0002)
        {
        	cout<<"Found device..."<<endl;

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
        	this->thread_stream=new boost::thread(&ThermAppCameraBulk::streamer,this);
            return false;
        }

    }
    dev_open=false;
    return true;
}

void ThermAppCameraBulk::closeDevice()
{
	if(dev_open)
	{
		this->dev_open=false;
		this->thread_stream->interrupt();
		this->thread_stream->join();
		libusb_close(handle);
	}
}







void ThermAppCameraBulk::streamer()
{
	  try
	  {
	    for (int i = 0; i < 5; ++i)
	    {
            libusb_claim_interface(handle,0);

            unsigned char ENDPOINT_DOWN = 0x81;
			unsigned char ENDPOINT_UP = 0x02;
			int actual_length;
			unsigned char dataUp[64];
			unsigned char dataDown[61440];

			memset(dataDown,0, sizeof(dataDown));
			memset(dataUp,0, sizeof(dataUp));

			unsigned char config2[64] =
			{
				0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xd5, 0xa5,	0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x20, 0x01, 0x80, 0x01, 0x20, 0x01,	0x80, 0x01, 0x19, 0x00,	0x00, 0x00, 0x00, 0x00,
				0xae, 0x07, 0x00, 0x00, 0x8f, 0x05, 0xa2, 0x08,	0x6d, 0x0b, 0x85, 0x0b,	0x00, 0x00, 0x00, 0x00,
				0x98, 0x09, 0x40, 0x00,	0x00, 0x00, 0x00, 0x00,	0x00, 0x00, 0x00, 0x00,	0x00, 0x00, 0xff, 0x0f
			};

			dataDown[0]=1;
			libusb_bulk_transfer(handle, ENDPOINT_DOWN, dataDown,
										0, &actual_length, 10);

			bool headerwait=true;
			bool imagestarted=false;
			unsigned byteno=0;
			std::vector <unsigned char> imvec;

			typedef
			enum
			{
				HEADER_COMPARE,
				SKIP,
				IMAGE,
			}state_t;


			state_t state=HEADER_COMPARE;
			std::vector<unsigned char> header;
			unsigned int compare=9;
			unsigned int compared=0;
			unsigned int skip=64;
			unsigned int skipped;


			std::vector<uint8_t> im_data;
			while(1)
			{

				unsigned char config[64] =
				{
					0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xd5, 0xa5,	0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					0x00, 0x00,

					/*Resolution twice ->*/0x20, 0x01, 0x80, 0x01,/**/ 0x20, 0x01,	0x80, 0x01,/**/



					0x19, 0x00,	0x00, 0x00, 0x00, 0x00,
					0xae, 0x07, 0x00, 0x00,
					/*Range ->*/0x7f, 0x05, 0xa2, 0x08,	0x6d, 0x0b, 0x85, 0x0b,	0x00, 0x00, 0x00, 0x00,
					0x98, 0x09, 0x40, 0x00,	0x00, 0x00, 0x00, 0x00,	0x00, 0x00, 0x00, 0x00,	0x00, 0x00, 0xff, 0x0f
				};


				actual_length=0;

					libusb_bulk_transfer(handle, ENDPOINT_UP, config,
							64, &actual_length, 30);
					actual_length=0;
					int a = libusb_bulk_transfer(handle, ENDPOINT_DOWN, dataDown,
							sizeof(dataDown), &actual_length, 120);


					std::vector<uint8_t> usb_data(dataDown, dataDown+actual_length);

					for(std::vector<uint8_t>::iterator it=usb_data.begin(); it!=usb_data.end();it++)
					{
						if(*it==config[0])
						{
							int h=0;
							for(std::vector<uint8_t>::iterator it2=it; it2!=usb_data.end();it2++)
							{

								if(*it2==config[h])
								{
									h++;
								}
								else
								{
									break;
								}

								if(h==9)
								{
									std::cout<<"CLEAR after "<<std::dec<<im_data.size()<<std::endl;
									h=0;
									im_data.clear();

									std::cout<<"                      HEADER: ";
									for(std::vector<uint8_t>::iterator it3=it; it3!=it+64;it3++)
									{
										std::cout<<std::hex<<(unsigned)*it3;
									}

									unsigned numberX=*(it+30); //Package length???
									unsigned numberY=*(it+52); //Package Number

									std::cout<<std::endl<<" ----------------- > "<<std::dec<<numberX<<" "<<numberY<<std::endl;
									it=it+64;
								}

							}
						}


						im_data.push_back(*it);


						if(im_data.size()==384*288*2)
						{

							cv::Mat myuv(288,384, CV_16UC1, im_data.data());
							cv::Mat color;
							color.create(288,384, CV_8UC3);

							myuv*=20;


							for (int row = 0; row < 288; ++row)
							{
								for (int col = 0; col < 384; ++col)
								{

									color.at<cv::Vec3b>(row,col)[0]=myuv.at<unsigned short>(row,col)/100;
									color.at<cv::Vec3b>(row,col)[1]=255;
									color.at<cv::Vec3b>(row,col)[2]=255;

									if(row==288*3/4/2 && col==384/2)
									{
										unsigned short value=myuv.at<unsigned short>(row,col);
										//value=((value&0xFF)<<8) | ((value&0xFF00)>>8);


										printf("%0.2X -> %i\n",value, value);
									}

								}


							}

							cv::cvtColor(color,color,CV_HSV2RGB);

							color.at<cv::Vec3b>(288/2,384/2)[0]=0;
							color.at<cv::Vec3b>(288/2,384/2)[1]=0;
							color.at<cv::Vec3b>(288/2,384/2)[2]=0;
							cv::circle(color,cv::Point(288/4,384/2),10,cv::Scalar(255,0,0));


							//imshow("Test",color);
							//cv::waitKey(1);

							std::cout<<im_data.size()<<std::endl;
							im_data.clear();

						}


					}//FOR usb_data

					printf("\nEND-PACKAGE  %i\n",actual_length);
					boost::this_thread::sleep(boost::posix_time::microseconds(114942));



			}//WHILE
			//libusb_release_interface(handle,0);
	    }
	  }
	  catch (boost::thread_interrupted&) {}
}


} /* namespace ThermApp */
