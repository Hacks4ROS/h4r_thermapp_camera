/***************************************************************************
* Copyright (C) 2015 by Alexander G  <pidbip@gmail.com> *
* *
* This program is free software: you can redistribute it and/or modify *
* it under the terms of the GNU General Public License as published by *
* the Free Software Foundation, either version 3 of the License, or *
* (at your option) any later version. *
* *
* This program is distributed in the hope that it will be useful, *
* but WITHOUT ANY WARRANTY; without even the implied warranty of *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the *
* GNU General Public License for more details. *
* *
* You should have received a copy of the GNU General Public License *
* along with this program. If not, see <http://www.gnu.org/licenses/>. *
* *
***************************************************************************/

#ifndef THERMAPP_H_
#define THERMAPP_H_

#include <pthread.h>

#include "libusb-1.0/libusb.h"

#if defined(_WIN32) || defined(__CYGWIN__) || defined(_WIN32_WCE)
#define THERMAPP_CALL WINAPI
#else
#define THERMAPP_CALL
#endif


#ifdef __cplusplus
extern "C" {
#endif


#define VENDOR	(0x1772)
#define PRODUCT	(0x0002)

#define PACKET_SIZE	(221688)
#define PIXELS_DATA_SIZE	(384*288)
#define FRAME_START_HEADER  (0xa5d5a5a5)
#define FRAME_STOP_HEADER   (0xa5a5a5a5)

//My Banana Pi works fine only at these settings
#define DEFAULT_BUF_LENGTH (16384)
#define DEFAULT_BUF_NUMBER (PACKET_SIZE / (DEFAULT_BUF_LENGTH))
#define DEFAULT_BUF_REMAIN (DEFAULT_BUF_NUMBER * DEFAULT_BUF_LENGTH)

#if ((DEFAULT_BUF_REMAIN % 512) != 0) /* len must be multiple of 512 */
#error "DEFAULT_BUF_LENGTH"
#endif

#define BULK_TIMEOUT 0

#ifndef FALSE
	#define FALSE 0
	#define TRUE 1
#endif


#pragma pack(push, 1)
// AD5628 DAC in Therm App is for generating control voltage
// VREF = 2.5 volts 11 Bit
struct cfg_packet{
	unsigned int none_volatile_data0;
	unsigned int none_volatile_data1;
    unsigned short modes;// 0xXXXM  Modes set last nibble
	unsigned short none_volatile_dbyte0;
	unsigned int none_volatile_data2;
	unsigned int none_volatile_data3;
	unsigned int none_volatile_data4;
	unsigned int none_volatile_data5;
	unsigned int none_volatile_data6;
    unsigned short VoutA; //DCoffset;// AD5628 VoutA, Range: 0V - 2.45V, max 2048
    unsigned short none_volatile_data7;
    unsigned short VoutC;//gain;// AD5628 VoutC, Range: 0V - 3.59V, max 2984 ??????
    unsigned short VoutD;//none_volatile_dbyte1;// AD5628 VoutD, Range: 0V - 2.895V, max 2394 ??????
    unsigned short VoutE;//volatile_data;// AD5628 VoutE, Range: 0V - 3.63V, max 2997, FPA VBUS
    unsigned short none_volatile_data8;
	unsigned int none_volatile_data9;
	unsigned int none_volatile_data10;
	unsigned int none_volatile_data11;
	unsigned int none_volatile_data12;
	unsigned int none_volatile_data13;
};


typedef struct _thermapp_packet{
	//int FrameHeaderStart;
	short some_data0;
	int id;
	unsigned char some_data1[16];
	short temperature;
	unsigned char some_data2[20];
    unsigned short frame_count;
	unsigned char some_data3[10];
	short pixels_data[PIXELS_DATA_SIZE];
	unsigned char some_data4[448];
	//int FrameHeaderStop;
}thermapp_packet;

#pragma pack(pop)


enum thermapp_async_status{
	THERMAPP_INACTIVE = 0,
	THERMAPP_CANCELING,
	THERMAPP_RUNNING
};

typedef void(*thermapp_read_async_cb_t)(unsigned char *buf, uint32_t len, void *ctx);

typedef struct thermapp{
    pthread_t pthreadReadAsync;
    pthread_t pthreadReadPipe;

	pthread_mutex_t mutex_thermapp;

	pthread_cond_t  cond_pipe,
					cond_getimage;

	int id;
	short temperature;
    unsigned short frame_count;

    libusb_device_handle *dev;
	libusb_context *ctx;
	struct libusb_transfer *transfer_in;
	struct libusb_transfer *transfer_out;

	char pipe_name[128];
	int fd_pipe_wr;
	int fd_pipe_rd;
    int pipe_create;

    unsigned int xfer_buf_num;
    unsigned int xfer_buf_len;
	struct libusb_transfer **xfer;
	unsigned char **xfer_buf;
	thermapp_read_async_cb_t cb;
	void *cb_ctx;
	unsigned int xfer_errors;
	enum thermapp_async_status async_status;
	int async_cancel;
	int dev_lost;

    struct cfg_packet *cfg;
    thermapp_packet *therm_packet;
    int lost_packet;
	int is_NewFrame;
    //short **calibrate_pixels;
}ThermApp;


ThermApp *thermapp_initUSB(void);

int thermapp_USB_checkForDevice(ThermApp *thermapp, int vendor, int product);

int thermapp_SendConfigurationHeader(ThermApp *thermapp, unsigned char *data, int lengh);

thermapp_packet *thermapp_FrameCapture(ThermApp *thermapp);

int thermapp_FrameRequest_thread(ThermApp *thermapp);

int thermapp_GetImage(ThermApp *thermapp, short *ImgData);

int thermapp_ParsingUsbPacket(ThermApp *thermapp, short *ImgData);

void thermapp_setGain(ThermApp *thermapp, unsigned short gain);

unsigned short thermapp_getGain(ThermApp *thermapp);

int thermapp_getId(ThermApp *thermapp);

float thermapp_getTemperature(ThermApp *thermapp);

unsigned short thermapp_getFrameCount(ThermApp *thermapp);

unsigned short thermapp_getDCoffset(ThermApp *thermapp);

void thermapp_setDCoffset(ThermApp *thermapp, unsigned short offset);

int thermapp_CalibrateFile(ThermApp *thermapp, FILE *file);

int thermapp_read_async(ThermApp *thermapp, thermapp_read_async_cb_t cb, void *ctx
			  /*uint32_t buf_num, uint32_t buf_len*/);

int thermapp_cancel_async(ThermApp *thermapp);

int thermapp_Close(ThermApp *thermapp);

#ifdef __cplusplus
}
#endif


#endif /* THERMAPP_H_ */
