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

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>

#include <time.h>
#include <string.h>
#include <errno.h>


#include "thermapp.h"


ThermApp *thermapp_initUSB(void)
{

    //fprintf(stderr, "malloc thermapp_initUSB\n");

    ThermApp *thermapp = (ThermApp*)malloc(sizeof(ThermApp));
    if(thermapp == NULL){
        fprintf(stderr, "Can't allocate thermapp\n");
        return NULL;
    }

    memset(thermapp, 0, sizeof(ThermApp));

    thermapp->cfg = (struct cfg_packet*)malloc(sizeof(struct cfg_packet));
    if(thermapp->cfg == NULL){
        free(thermapp);
        fprintf(stderr, "Can't allocate cfg_packet\n");
        return NULL;
    }

    thermapp->therm_packet = malloc(sizeof(thermapp_packet));
    if(thermapp->therm_packet == NULL){
        free(thermapp->cfg);
        free(thermapp);
        fprintf(stderr, "Can't allocate thermapp_packet\n");
        return NULL;
    }

    if(sizeof(*thermapp->therm_packet) != PACKET_SIZE){
        free(thermapp->cfg);
        free(thermapp);
        fprintf(stderr, "thermapp_packet not equal PACKET_SIZE\n");
        return NULL;
    }

    //Initialize data struct
    // this init data was recieved from usbmonitor
    thermapp->cfg->none_volatile_data0 = 0xa5a5a5a5;//I think it's preamble of cfg_packet
    thermapp->cfg->none_volatile_data1 = 0xa5d5a5a5;
    thermapp->cfg->modes = 0x0002; //test pattern low
    thermapp->cfg->none_volatile_dbyte0 = 0x0000;//
    thermapp->cfg->none_volatile_data2 = 0x00000000;//
    thermapp->cfg->none_volatile_data3 = 0x01200000;//
    thermapp->cfg->none_volatile_data4 = 0x01200180;//
    thermapp->cfg->none_volatile_data5 = 0x00190180;// high  low
    thermapp->cfg->none_volatile_data6 = 0x00000000;//
    thermapp->cfg->VoutA = 0x0795;
    thermapp->cfg->none_volatile_data7 = 0x0000;
    thermapp->cfg->VoutC = 0x058f;
    thermapp->cfg->VoutD = 0x08a2;
    thermapp->cfg->VoutE = 0x0b6d;
    thermapp->cfg->none_volatile_data8 = 0x0b85;//
    thermapp->cfg->none_volatile_data9 =  0x00000000;//
    thermapp->cfg->none_volatile_data10 = 0x00400998;//
    thermapp->cfg->none_volatile_data11 = 0x00000000;//
    thermapp->cfg->none_volatile_data12 = 0x00000000;//low
    thermapp->cfg->none_volatile_data13 = 0x0fff0000;//

    thermapp->async_status = THERMAPP_INACTIVE;

    //Init libusb
    if(libusb_init(&thermapp->ctx) < 0)
    {
        free(thermapp->cfg);
        free(thermapp);
        fprintf(stderr, "failed to initialize libusb\n");
        return NULL;
    }

    //fprintf(stderr, "PTHREAD INITIALIZER\n");

    //init thread condition
	thermapp->cond_pipe = (pthread_cond_t) PTHREAD_COND_INITIALIZER;
	thermapp->cond_getimage = (pthread_cond_t) PTHREAD_COND_INITIALIZER;
	thermapp->mutex_thermapp = (pthread_mutex_t) PTHREAD_MUTEX_INITIALIZER;

	srand(time(NULL));

    //Get random name for fifo pipe
	sprintf(thermapp->pipe_name, "/tmp/therm_%d", (int)rand());
    //fprintf(stderr, "mkfifo(thermapp->pipe_name, 0777)\n");

    //Make fifo pipe
	if ( (mkfifo(thermapp->pipe_name, 0777) == -1) )// && (errno != EEXIST))
	{
        free(thermapp->cfg);
        free(thermapp);
        //FIXME: need to compare thermapp->pipe_name on NULL
        thermapp->pipe_create = FALSE;
		perror("mkfifo");
		return NULL;
    }else{
        thermapp->pipe_create = TRUE;
		printf("\nfifo pipe %s is created\n", thermapp->pipe_name);
    }
    thermapp->fd_pipe_rd = 0;
    thermapp->fd_pipe_wr = 0;
    //thermapp->is_NewFrame = FALSE;
    //thermapp->lost_packet = 0;

    return thermapp;
}

int thermapp_USB_checkForDevice(ThermApp *thermapp, int vendor, int product)
{

	int status;
	unsigned char buffer[255];

    //puts("libusb_open_device_with_vid_pid\n");

    ///FIXME: For Debug use libusb_open_device_with_vid_pid
    /// need to add search device
    thermapp->dev = libusb_open_device_with_vid_pid(thermapp->ctx, vendor, product);
    if (thermapp->dev == NULL){
        free(thermapp->cfg);
        free(thermapp);
        fprintf(stderr, "Open device with vid pid failed\n");
		return -1;
	}

    //if (libusb_kernel_driver_active(thermapp->dev, 0))
    //      libusb_detach_kernel_driver(thermapp->dev, 0);

    if(libusb_claim_interface(thermapp->dev, 0))
	{
        free(thermapp->cfg);
        free(thermapp);
        fprintf(stderr, "claim interface failed\n");
		return 0;
	}


    // I don't know what is this but this is needed to make ThermApp work. I received it from usbmonitor
    status = libusb_control_transfer(thermapp->dev, LIBUSB_ENDPOINT_IN, 0x06, 0x0100,0x0000,buffer, 0x12,0);
	fprintf(stdout, "status: %d, ",status);

    status = libusb_control_transfer(thermapp->dev, LIBUSB_ENDPOINT_IN, 0x06, 0x0200,0x0000,buffer, 0x09,0);
	fprintf(stdout, "status: %d, ",status);

    status = libusb_control_transfer(thermapp->dev, LIBUSB_ENDPOINT_IN, 0x06, 0x0200,0x0000,buffer, 0x20,0);
	fprintf(stdout, "status: %d, ",status);

    status = libusb_control_transfer(thermapp->dev, LIBUSB_ENDPOINT_IN, 0x06, 0x0300,0x0000,buffer, 0xff,0);
	fprintf(stdout, "status: %d, ",status);

    status = libusb_control_transfer(thermapp->dev, LIBUSB_ENDPOINT_IN, 0x06, 0x0302,0x0409,buffer, 0xff,0);
	fprintf(stdout, "status: %d, ",status);

    status = libusb_control_transfer(thermapp->dev, LIBUSB_ENDPOINT_IN, 0x06, 0x0301,0x0409,buffer, 0xff,0);
	fprintf(stdout, "status: %d, ",status);

    status = libusb_control_transfer(thermapp->dev, LIBUSB_ENDPOINT_IN, 0x06, 0x0303,0x0409,buffer, 0xff,0);
	fprintf(stdout, "status: %d, ",status);

    status = libusb_control_transfer(thermapp->dev, LIBUSB_TRANSFER_TYPE_CONTROL, 0x09, 0x0001,0x0000,buffer, 0x00,0);
	fprintf(stdout, "status: %d, ",status);

    status = libusb_control_transfer(thermapp->dev, LIBUSB_ENDPOINT_IN, 0x06, 0x0304,0x0409,buffer, 0xff,0);
	fprintf(stdout, "status: %d, ",status);
    //print_status(buffer, 0xff);

    status = libusb_control_transfer(thermapp->dev, LIBUSB_ENDPOINT_IN, 0x06, 0x0305,0x0409,buffer, 0xff,0);
    fprintf(stdout, "status: %d, \n",status);

	return 0;
}

static
void THERMAPP_CALL thermapp_PipeWrite(unsigned char *buf, uint32_t len, void *ctx)
{
    //fprintf(stderr, "therm_callback\n");
    unsigned int w_len;

	if (ctx) {
		if((w_len = write(*(int*)ctx, buf, len)) <= 0)
		{
			perror("fifo write");

		}
        //FIXME: w_len return can be smaller len
		if(w_len < len){
            printf("pipe write len smaller. len: %d w_len: %d\n", len, w_len);
		}	
	}
}

void* thermapp_ThreadReadAsync(void* ctx){

	ThermApp *thermapp = ctx;

    // Open fifo pipe for write
    if ((thermapp->fd_pipe_wr = open(thermapp->pipe_name, O_WRONLY)) <= 0 ) {
		perror("fifo open");
		return NULL;
	}

    puts("thermapp_ThreadReadAsync run\n");
    thermapp_read_async(thermapp, thermapp_PipeWrite, (void*)&thermapp->fd_pipe_wr);

    puts("close(thermapp->fd_pipe_wr)\n");

	close(thermapp->fd_pipe_wr);
    thermapp->fd_pipe_wr = 0;

	return NULL;
}

void* thermapp_ThreadPipeRead(void* ctx){

	ThermApp *thermapp = (ThermApp*)ctx;
    unsigned int len, FrameHeaderStart = 0, FrameHeaderStop = 0;
    int actual_length = 0;

    // Open fifo pipe for read
	if ( (thermapp->fd_pipe_rd = open(thermapp->pipe_name, O_RDONLY)) <= 0 ) {
		perror("fifo pipe open");
		return NULL;
	}

    enum thermapp_async_status current_status = THERMAPP_RUNNING;

    puts("thermapp_ThreadPipeRead run\n");
    while(current_status == THERMAPP_RUNNING){

        if (( len =read(thermapp->fd_pipe_rd, &FrameHeaderStart, sizeof(FrameHeaderStart))) <= 0 ) {
            fprintf(stderr, "read thermapp_ThreadPipeRead()\n");
            perror("fifo pipe read");
            break;
		}

        //fprintf(stderr,"thermapp_ThreadPipeRead(): thermapp->async_status =  %d\n",thermapp->async_status);

        //Catch Start preamble FRAME_START_HEADER
		if(FrameHeaderStart == FRAME_START_HEADER){
			//fprintf(stderr,"FrameHeaderStart == FRAME_START_HEADER\n");
			thermapp->is_NewFrame = FALSE;
			pthread_mutex_lock(&thermapp->mutex_thermapp);
            for(actual_length = 0; actual_length < (PACKET_SIZE);){
                len = read(thermapp->fd_pipe_rd, (void*)(thermapp->therm_packet)
                        + actual_length, (PACKET_SIZE) - actual_length);
				if ( len <= 0 ) {
                    fprintf(stderr, "read thermapp_ThreadPipeRead()\n");
                    perror("fifo pipe read");
                    pthread_mutex_unlock(&thermapp->mutex_thermapp);
                    break;
				}
                actual_length += len;
                //fprintf(stderr,"len: %d, actual_length: %d\n",len, actual_length);
			}

           // fprintf(stderr,"len: %d, actual_length: %d\n",len, actual_length);
			//fprintf(stderr,"FRAME------------->");
			if (read(thermapp->fd_pipe_rd, &FrameHeaderStop, sizeof(FrameHeaderStop)) <= 0 ) {
                fprintf(stderr, "read thermapp_ThreadPipeRead()\n");
                perror("fifo pipe read");
                pthread_mutex_unlock(&thermapp->mutex_thermapp);
                break;;
			}

            //Catch Stop preamble FRAME_STOP_HEADER
            // On FRAME_STOP_HEADER we can check the damaged packet
			if(FrameHeaderStop == FRAME_STOP_HEADER){
               // fprintf(stderr,"FRAME_OK\n");
				thermapp->is_NewFrame = TRUE;
                //pthread_cond_signal(&thermapp->cond_getimage);
                pthread_cond_wait(&thermapp->cond_pipe, &thermapp->mutex_thermapp);
			}
			else{
                fprintf(stderr,"lost frame\n");
                thermapp->lost_packet++; //increment damaged frames counter
			}

            current_status = thermapp->async_status;

			pthread_mutex_unlock(&thermapp->mutex_thermapp);
		}
	}

    fprintf(stderr, "close(thermapp->fd_pipe_rd);\n");

    close(thermapp->fd_pipe_rd);
    thermapp->fd_pipe_rd = 0;

	return NULL;
}

int thermapp_ParsingUsbPacket(ThermApp *thermapp, short *ImgData){


    thermapp->id = thermapp->therm_packet->id;
    thermapp->temperature = thermapp->therm_packet->temperature;
    thermapp->frame_count = thermapp->therm_packet->frame_count;

#if 0
	int i;
	for(i = 0; i < PIXELS_DATA_SIZE; i++)
	{
        ImgData[i] = thermapp->therm_packet->pixels_data[i];
				//- thermapp->calibrate_pixels[0][i];
	}
#else

#if 1
    memcpy(ImgData, thermapp->therm_packet->pixels_data, PIXELS_DATA_SIZE*2);
#else
// Debug some test
    int i, i_src = 0;
    for (i = 0; i < PIXELS_DATA_SIZE; i++){

       // if((i == 8567))
        //else
        ImgData[i] = thermapp->therm_packet->pixels_data[i_src];

        i_src++;
    }
    ImgData[8567] = (thermapp->therm_packet->pixels_data[8566]
                    + thermapp->therm_packet->pixels_data[8568])/2;

    i_src = 63360-384;
    for (i=63360; i<63360+384; i++){
        ImgData[i] = thermapp->therm_packet->pixels_data[i_src];
        i_src++;
    }

    i_src = 79488-384;
    for (i=79488; i<79488+384; i++){
        ImgData[i] = thermapp->therm_packet->pixels_data[i_src];
        i_src++;
    }
#endif
#endif

	return 1;
}

// This function for getting frame pixel data
int thermapp_GetImage(ThermApp *thermapp, short *ImgData){

    if(!thermapp->is_NewFrame) return 0;

	pthread_mutex_lock(&thermapp->mutex_thermapp);

    thermapp_ParsingUsbPacket(thermapp, ImgData);
    thermapp->is_NewFrame = FALSE;
    pthread_cond_signal(&thermapp->cond_pipe);

	pthread_mutex_unlock(&thermapp->mutex_thermapp);

	return 1;
}


// Create read and write thread
int thermapp_FrameRequest_thread(ThermApp *thermapp){
	int  ret;

    ret = pthread_create( &thermapp->pthreadReadAsync, NULL, thermapp_ThreadReadAsync, (void*)thermapp);
	if(ret)
	{
		fprintf(stderr,"Error - pthread_create() return code: %d\n",ret);
		return -1;
	}

    ret = pthread_create( &thermapp->pthreadReadPipe, NULL, thermapp_ThreadPipeRead, (void*)thermapp);
	if(ret)
	{
		fprintf(stderr,"Error - pthread_create() return code: %d\n",ret);
		return -1;
	}

	return 1;
}


void thermapp_setGain(ThermApp *thermapp, unsigned short gain){
    //thermapp->cfg->gain = gain;
}

unsigned short thermapp_getGain(ThermApp *thermapp){
    //return thermapp->cfg->gain;
}

int thermapp_getId(ThermApp *thermapp){
	return thermapp->id;
}

//I don't know offset and quant value for temperature.
//I use experimental value.
float thermapp_getTemperature(ThermApp *thermapp){
	short t = thermapp->temperature;
    return (t - 14336) * 0.00652;
}

unsigned short thermapp_getFrameCount(ThermApp *thermapp){
	return thermapp->frame_count;
}

unsigned short thermapp_getDCoffset(ThermApp *thermapp){
   // return thermapp->cfg->DCoffset;
}

void thermapp_setDCoffset(ThermApp *thermapp, unsigned short offset){
   // thermapp->cfg->DCoffset = offset;
}

/*
int thermapp_LoadCalibrate(ThermApp *thermapp, int id){

	return 0;
}
*/

//Transfer function for put date to ThermApp
static void LIBUSB_CALL transfer_cb_out(struct libusb_transfer *transfer)
{
	ThermApp *thermapp = transfer->user_data;

	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
        fprintf(stderr, "transfer_cb_out() : in transfer status %d\n", transfer->status);
		libusb_free_transfer(transfer);
		thermapp->transfer_out = NULL;
		return;
	}

	if (libusb_submit_transfer(thermapp->transfer_out) < 0){
		fprintf(stderr, "in transfer libusb_submit_transfer\n");
	}

}

static int thermapp_alloc_transfer_out(ThermApp *thermapp){
	int r;

	printf("libusb_alloc_transfer out \n");
	thermapp->transfer_out = libusb_alloc_transfer(0);
	if(thermapp->transfer_out == NULL){
		fprintf(stderr, "Can't allocate transfer_out\n");
		return -1;
	}


	printf("libusb_fill_bulk_transfer out \n");
    libusb_fill_bulk_transfer(thermapp->transfer_out, thermapp->dev,LIBUSB_ENDPOINT_OUT | 2,
			(unsigned char*)thermapp->cfg, sizeof(*thermapp->cfg), transfer_cb_out, thermapp, -1);


	printf("libusb_submit_transfer out \n");
	r = libusb_submit_transfer(thermapp->transfer_out);
	printf("libusb submit transfer_out %d\n", r);

	return 0;
}

static int thermapp_free_transfer_out(ThermApp *thermapp){

    fprintf(stderr, "thermapp_free_transfer_out\n");

    if (!thermapp)
        return -1;

    if(!thermapp->transfer_out)
        return -2;

    libusb_free_transfer(thermapp->transfer_out);

    return 0;
}

static
void LIBUSB_CALL _libusb_callback(struct libusb_transfer *xfer)
{
    //fprintf(stderr, "LIBUSB_CALL _libusb_callback\n");

	ThermApp *thermapp = (ThermApp *)xfer->user_data;

	if (LIBUSB_TRANSFER_COMPLETED == xfer->status) {
        //fprintf(stderr, "LIBUSB_TRANSFER_COMPLETED\n");
		if (thermapp->cb)
			thermapp->cb(xfer->buffer, xfer->actual_length, thermapp->cb_ctx);
                //write(thermapp->fd_pipe_wr, xfer->buffer, xfer->actual_length);// Write to fifo pipe

		libusb_submit_transfer(xfer); /* resubmit transfer */
		thermapp->xfer_errors = 0;
	} else if (LIBUSB_TRANSFER_CANCELLED != xfer->status) {

		fprintf(stderr, "LIBUSB_TRANSFER_CANCELLED != xfer->status\n");

		if (LIBUSB_TRANSFER_ERROR == xfer->status)
			thermapp->xfer_errors++;

		if (thermapp->xfer_errors >= thermapp->xfer_buf_num ||
		    LIBUSB_TRANSFER_NO_DEVICE == xfer->status) {

            thermapp->dev_lost = 1;
            thermapp_cancel_async(thermapp);

			fprintf(stderr, "LIBUSB_TRANSFER_NO_DEVICE\n");

		}

	}
}

static int _thermapp_alloc_async_buffers(ThermApp *thermapp)
{
	unsigned int i;

	fprintf(stderr, "_thermapp_alloc_async_buffers\n");

	if (!thermapp)
		return -1;


	if (!thermapp->xfer) {
        thermapp->xfer = calloc(DEFAULT_BUF_NUMBER + (DEFAULT_BUF_REMAIN != 0),
				   sizeof(struct libusb_transfer *));

        for(i = 0; i < DEFAULT_BUF_NUMBER + (DEFAULT_BUF_REMAIN != 0) ; i++)
			thermapp->xfer[i] = libusb_alloc_transfer(0);
	}


	if (!thermapp->xfer_buf) {
        thermapp->xfer_buf = calloc(DEFAULT_BUF_NUMBER + (DEFAULT_BUF_REMAIN != 0),
					   sizeof(unsigned char *));


    for(i = 0; i < DEFAULT_BUF_NUMBER; i++)
        thermapp->xfer_buf[i] = malloc(DEFAULT_BUF_LENGTH);
	}



#if (DEFAULT_BUF_REMAIN != 0)
    thermapp->xfer_buf[i] = malloc(DEFAULT_BUF_REMAIN);
#endif


	return 0;
}

static int _thermapp_free_async_buffers(ThermApp *thermapp)
{
	unsigned int i;

	fprintf(stderr, "_thermapp_free_async_buffers\n");

	if (!thermapp)
		return -1;

	if (thermapp->xfer) {
        for(i = 0; i < DEFAULT_BUF_NUMBER + (DEFAULT_BUF_REMAIN != 0); i++) {
			if (thermapp->xfer[i]) {
				libusb_free_transfer(thermapp->xfer[i]);
			}
		}

		free(thermapp->xfer);
		thermapp->xfer = NULL;
	}

	if (thermapp->xfer_buf) {
        for(i = 0; i < DEFAULT_BUF_NUMBER + (DEFAULT_BUF_REMAIN != 0); i++) {
			if (thermapp->xfer_buf[i])
				free(thermapp->xfer_buf[i]);
		}

		free(thermapp->xfer_buf);
		thermapp->xfer_buf = NULL;
	}

	return 0;
}

int thermapp_read_async(ThermApp *thermapp, thermapp_read_async_cb_t cb, void *ctx)
{

    fprintf(stderr, "thermapp_read_async\n");

	unsigned int i;
	int r = 0;
	struct timeval tv = { 1, 0 };
	struct timeval zerotv = { 0, 0 };
	enum thermapp_async_status next_status = THERMAPP_INACTIVE;

	thermapp->xfer = NULL;
	thermapp->xfer_buf = NULL;

    if (!thermapp){
        fprintf(stderr, "!thermapp\n");
		return -1;
    }

    if (THERMAPP_INACTIVE != thermapp->async_status){
        fprintf(stderr, "THERMAPP_INACTIVE != thermapp->async_status\n");
		return -2;
    }

	thermapp->async_status = THERMAPP_RUNNING;
	thermapp->async_cancel = 0;

	thermapp->cb = cb;
	thermapp->cb_ctx = ctx;

	thermapp_alloc_transfer_out(thermapp);

	_thermapp_alloc_async_buffers(thermapp);

	for(i = 0; i < DEFAULT_BUF_NUMBER; i++) {
		fprintf(stderr, "libusb_fill: %d\n", i);
		libusb_fill_bulk_transfer(thermapp->xfer[i],
                thermapp->dev,
				LIBUSB_ENDPOINT_IN | 1,
					  thermapp->xfer_buf[i],
                      DEFAULT_BUF_LENGTH,
					  _libusb_callback,
					  (void *)thermapp,
					  BULK_TIMEOUT);

		r = libusb_submit_transfer(thermapp->xfer[i]);
		if (r < 0) {
			fprintf(stderr, "Failed to submit transfer %i!\n", i);
			thermapp->async_status = THERMAPP_CANCELING;
			break;
		}
	}

#if (DEFAULT_BUF_REMAIN != 0)
	libusb_fill_bulk_transfer(thermapp->xfer[i],
            thermapp->dev,
			LIBUSB_ENDPOINT_IN | 1,
			thermapp->xfer_buf[i],
            DEFAULT_BUF_REMAIN,
			_libusb_callback,
			(void *)thermapp,
			BULK_TIMEOUT);

	r = libusb_submit_transfer(thermapp->xfer[i]);
	if (r < 0) {
		fprintf(stderr, "Failed to submit transfer %i!\n", i);
		thermapp->async_status = THERMAPP_CANCELING;
	}
#endif

	while (THERMAPP_INACTIVE != thermapp->async_status) {
        r = libusb_handle_events_timeout_completed(thermapp->ctx, &tv,
							   &thermapp->async_cancel);
        //r = libusb_handle_events_completed(thermapp->ctx, &thermapp->async_cancel);
		if (r < 0) {
            fprintf(stderr, "handle_events returned: %d\n", r);
			if (r == LIBUSB_ERROR_INTERRUPTED) /* stray signal */{
                fprintf(stderr, "LIBUSB_ERROR_INTERRUPTED\n");
				continue;
			}
			break;
		}

		if (THERMAPP_CANCELING == thermapp->async_status) {
            fprintf(stderr, "THERMAPP_CANCELING == thermapp->async_status\n");

			next_status = THERMAPP_INACTIVE;

			if (!thermapp->xfer)
				break;

			for(i = 0; i < thermapp->xfer_buf_num; ++i) {
				if (!thermapp->xfer[i])
					continue;

				if (LIBUSB_TRANSFER_CANCELLED !=
						thermapp->xfer[i]->status) {
					r = libusb_cancel_transfer(thermapp->xfer[i]);
                    // handle events after cancelling
                    // to allow transfer status to
                    // propagate
                    libusb_handle_events_timeout_completed(thermapp->ctx,
									       &zerotv, NULL);
					if (r < 0)
						continue;

					next_status = THERMAPP_CANCELING;
				}
            }

			if (thermapp->dev_lost || THERMAPP_INACTIVE == next_status) {
                // handle any events that still need to
                // be handled before exiting after we
                // just cancelled all transfers
                libusb_handle_events_timeout_completed(thermapp->ctx,
								       &zerotv, NULL);
				break;
			}
		}
	}
	if(THERMAPP_INACTIVE == thermapp->async_status)
		fprintf(stderr, "THERMAPP_INACTIVE == thermapp->async_status\n");

    thermapp_free_transfer_out(thermapp);

	_thermapp_free_async_buffers(thermapp);
	fprintf(stderr, "_thermapp_free_async_buffers complate\n");

	thermapp->async_status = next_status;

	return r;
}

int thermapp_cancel_async(ThermApp *thermapp)
{
	if (!thermapp)
		return -1;

    // if streaming, try to cancel gracefully
	if (THERMAPP_RUNNING == thermapp->async_status) {
		thermapp->async_status = THERMAPP_CANCELING;
		thermapp->async_cancel = 1;
		return 0;
	}

    // if called while in pending state, change the state forcefully
#if 0
    if (THERMAPP_INACTIVE != thermapp->async_status) {
		thermapp->async_status = THERMAPP_INACTIVE;
		return 0;
	}
#endif
	return -2;
}


int thermapp_Close(ThermApp *thermapp)
{
    if(!thermapp)
        return -1;

    thermapp_cancel_async(thermapp);

    sleep(1);

    if(thermapp->fd_pipe_rd)
       close(thermapp->fd_pipe_rd);
    thermapp->fd_pipe_rd = 0;

    if(thermapp->fd_pipe_wr)
       close(thermapp->fd_pipe_wr);
    thermapp->fd_pipe_wr = 0;

    if(thermapp->pipe_create)
        remove(thermapp->pipe_name);

    thermapp->pipe_name[0] = '\0';

    if(thermapp->cfg)
        free(thermapp->cfg);
    thermapp->cfg = NULL;

    libusb_release_interface(thermapp->dev, 0);

    //libusb_close(thermapp->dev);
    //printf("libusb_close\n");
    libusb_exit(thermapp->ctx);

    free(thermapp);
    thermapp = NULL;

    return 0;
}
