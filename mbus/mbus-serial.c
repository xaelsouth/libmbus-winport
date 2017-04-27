//------------------------------------------------------------------------------
// Copyright (C) 2011, Robert Johansson, Raditex AB
// All rights reserved.
//
// rSCADA
// http://www.rSCADA.se
// info@rscada.se
//
//------------------------------------------------------------------------------

#include <limits.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#ifndef _WIN32
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <termios.h>
#include <strings.h>
#endif

#include "mbus-serial.h"
#include "mbus-protocol-aux.h"
#include "mbus-protocol.h"

#define PACKET_BUFF_SIZE 2048

#ifdef _WIN32
static MBUS_HANDLE open_serial_port(const char *device)
{
	return CreateFile(device,
					    GENERIC_READ | GENERIC_WRITE,
						0,                          /* no share  */
						NULL,                       /* no security */
						OPEN_EXISTING,
						0,
						NULL);                      /* no templates */

}

static int close_serial_port(mbus_handle *handle)
{
	if (handle == NULL)
		return -1;

	CloseHandle(handle->hndl);

	return 0;
}

static int set_serial_port_baudrate_2400(mbus_handle *handle)
{
	DCB port_settings;
	COMMTIMEOUTS timeouts;

	if (handle == NULL)
		return -1;

	memset(&port_settings, 0, sizeof(port_settings));
	memset(&timeouts, 0, sizeof(timeouts));

	port_settings.DCBlength = sizeof(port_settings);
	if ( !BuildCommDCBA("baud=2400 parity=N data=8 stop=1", &port_settings) )
	{
		fprintf(stderr, "Unable to set comport dcb settings.\n");
		return -1;
	}

	port_settings.fBinary = 1;
	if ( !SetCommState(handle->hndl, &port_settings) )
	{
		fprintf(stderr, "Unable to set comport cfg settings.\n");
		return -1;
	}

	timeouts.ReadTotalTimeoutConstant = 30;
	if (!SetCommTimeouts(handle->hndl, &timeouts))
	{
		fprintf(stderr, "Unable to set comport timeouts.\n");
		return -1;
	}

	return 0;
}

static ssize_t read_serial_port(mbus_handle *handle, char *buf, int remaining)
{
	DWORD read_bytes;

	if (handle == NULL)
		return -1;

	if (!ReadFile(handle->hndl, buf, remaining, &read_bytes, NULL))
		return -1;
	
	return read_bytes;
}

static ssize_t write_serial_port(mbus_handle *handle, char *buf, int remaining)
{
	DWORD written_bytes;

	if (handle == NULL)
		return -1;

	if (!WriteFile(handle->hndl, buf, remaining, &written_bytes, NULL))
		return -1;

	return written_bytes;
}
#else
static MBUS_HANDLE open_serial_port(const char *device)
{
	return open(device, O_RDWR | O_NOCTTY);
}

static int close_serial_port(mbus_handle *handle)
{
	if (handle == NULL)
		return -1;

	close(handle->hndl);
}

static int set_serial_port_baudrate_2400(mbus_handle *handle)
{
	mbus_serial_data *serial_data;
	struct termios *term;

	if (handle == NULL)
		return -1;

	serial_data = (mbus_serial_data *) handle->auxdata;

	if (serial_data == NULL)
		return -1;

	term = serial_data->t;

	if (term == NULL)
		return -1;

	memset(term, 0, sizeof(*term));
	term->c_cflag |= (CS8|CREAD|CLOCAL);
	term->c_cflag |= PARENB;

	// No received data still OK
	term->c_cc[VMIN] = (cc_t)0;

	// Wait at most 0.2 sec.Note that it starts after first received byte!!
	// I.e. if CMIN>0 and there are no data we would still wait forever...
	//
	// The specification mentions link layer response timeout this way:
	// The time structure of various link layer communication types is described in EN60870-5-1. The answer time
	// between the end of a master send telegram and the beginning of the response telegram of the slave shall be
	// between 11 bit times and (330 bit times + 50ms).
	//
	// Nowadays the usage of USB to serial adapter is very common, which could
	// result in additional delay of 100 ms in worst case.
	//
	// For 2400Bd this means (330 + 11) / 2400 + 0.15 = 292 ms (added 11 bit periods to receive first byte).
	// I.e. timeout of 0.3s seems appropriate for 2400Bd.

	term->c_cc[VTIME] = (cc_t)3; // Timeout in 1/10 sec

	cfsetispeed(term, B2400);
	cfsetospeed(term, B2400);

#ifdef MBUS_SERIAL_DEBUG
	printf("%s: t->c_cflag = %x\n", __PRETTY_FUNCTION__, term->c_cflag);
	printf("%s: t->c_oflag = %x\n", __PRETTY_FUNCTION__, term->c_oflag);
	printf("%s: t->c_iflag = %x\n", __PRETTY_FUNCTION__, term->c_iflag);
	printf("%s: t->c_lflag = %x\n", __PRETTY_FUNCTION__, term->c_lflag);
#endif

	tcsetattr(handle->hndl, TCSANOW, term);

	return 0;
}

static ssize_t read_serial_port(mbus_handle *handle, char *buf, int remaining)
{
	if (handle == NULL)
		return -1;

	return read(handle->hndl, buf, remaining);
}

static ssize_t write_serial_port(mbus_handle *handle, char *buf, int remaining)
{
	if (handle == NULL)
		return -1;

	return write(handle->hndl, buf, remaining);
}
#endif

//------------------------------------------------------------------------------
/// Set up a serial connection handle.
//------------------------------------------------------------------------------
int
mbus_serial_connect(mbus_handle *handle)
{
    mbus_serial_data *serial_data;
    const char *device;

    if (handle == NULL)
        return -1;

    serial_data = (mbus_serial_data *) handle->auxdata;
    if (serial_data == NULL || serial_data->device == NULL)
        return -1;

    device = serial_data->device;

    //
    // create the SERIAL connection
    //

    // Use blocking read and handle it by serial port setting
    if ((handle->hndl = open_serial_port(device)) != MBUS_INVALID_HANDLE)
    {
        fprintf(stderr, "%s: failed to open tty.", __PRETTY_FUNCTION__);
        return -1;
    }

	set_serial_port_baudrate_2400(handle);

    return 0;
}

//------------------------------------------------------------------------------
// Set baud rate for serial connection
//------------------------------------------------------------------------------
#ifdef _WIN32
int
mbus_serial_set_baudrate(mbus_handle *handle, long baudrate)
{
	char mode_str[128];
	DCB port_settings;
	COMMTIMEOUTS timeouts;
	mbus_serial_data *serial_data;

	memset(&port_settings, 0, sizeof(port_settings));
	memset(&timeouts, 0, sizeof(timeouts));

	if (handle == NULL)
		return -1;

	serial_data = (mbus_serial_data *) handle->auxdata;

	if (serial_data == NULL)
		return -1;

	switch (baudrate)
	{
	case     300:
		strncpy(mode_str, "baud=300", sizeof(mode_str));
		timeouts.ReadTotalTimeoutConstant = 130;
		break;
	case     600:
		strncpy(mode_str, "baud=600", sizeof(mode_str));
		timeouts.ReadTotalTimeoutConstant = 80;
		break;
	case    1200:
		strncpy(mode_str, "baud=1200", sizeof(mode_str));
		timeouts.ReadTotalTimeoutConstant = 50;
		break;
	case    2400:
		strncpy(mode_str, "baud=2400", sizeof(mode_str));
		timeouts.ReadTotalTimeoutConstant = 30;
		break;
	case    4800:
		strncpy(mode_str, "baud=4800", sizeof(mode_str));
		timeouts.ReadTotalTimeoutConstant = 30;
		break;
	case    9600:
		strncpy(mode_str, "baud=9600", sizeof(mode_str));
		timeouts.ReadTotalTimeoutConstant = 20;
		break;
	case   19200:
		strncpy(mode_str, "baud=19200", sizeof(mode_str));
		timeouts.ReadTotalTimeoutConstant = 20;
		break;
	case   38400:
		strncpy(mode_str, "baud=38400", sizeof(mode_str));
		timeouts.ReadTotalTimeoutConstant = 20;
		break;
	default:
		return -1; // unsupported baudrate
	}

	strncat(mode_str, " parity=N data=8 stop=1", sizeof(mode_str));

	port_settings.DCBlength = sizeof(port_settings);
	if (!BuildCommDCBA(mode_str, &port_settings))
	{
		fprintf(stderr, "Unable to set comport dcb settings.\n");
		return -1;
	}

	port_settings.fBinary = 1;
	if ( !SetCommState(handle->hndl, &port_settings) )
	{
		fprintf(stderr, "Unable to set comport cfg settings.\n");
		return -1;
	}
	
	if (!SetCommTimeouts(handle->hndl, &timeouts))
	{
		fprintf(stderr, "Unable to set comport timeouts.\n");
		return -1;
	}

	return 0;
}
#else
int
mbus_serial_set_baudrate(mbus_handle *handle, long baudrate)
{
    speed_t speed;
    mbus_serial_data *serial_data;
	struct termios *term;

    if (handle == NULL)
        return -1;

    serial_data = (mbus_serial_data *) handle->auxdata;

    if (serial_data == NULL)
        return -1;

	term = serial_data->t;
	
	if (term == NULL)
	    return -1;

    switch (baudrate)
    {
        case 300:
            speed = B300;
            term->c_cc[VTIME] = (cc_t) 13; // Timeout in 1/10 sec
            break;

        case 600:
            speed = B600;
			term->c_cc[VTIME] = (cc_t)8;  // Timeout in 1/10 sec
            break;

        case 1200:
            speed = B1200;
			term->c_cc[VTIME] = (cc_t)5;  // Timeout in 1/10 sec
            break;

        case 2400:
            speed = B2400;
			term->c_cc[VTIME] = (cc_t)3;  // Timeout in 1/10 sec
            break;

        case 4800:
            speed = B4800;
			term->c_cc[VTIME] = (cc_t)3;  // Timeout in 1/10 sec
            break;

        case 9600:
            speed = B9600;
			term->c_cc[VTIME] = (cc_t)2;  // Timeout in 1/10 sec
            break;

        case 19200:
            speed = B19200;
			term->c_cc[VTIME] = (cc_t)2;  // Timeout in 1/10 sec
            break;

        case 38400:
            speed = B38400;
			term->c_cc[VTIME] = (cc_t)2;  // Timeout in 1/10 sec
            break;

       default:
            return -1; // unsupported baudrate
    }

    // Set input baud rate
    if (cfsetispeed(term, speed) != 0)
    {
        return -1;
    }

    // Set output baud rate
	if (cfsetospeed(term, speed) != 0)
    {
        return -1;
    }

    // Change baud rate immediately
	if (tcsetattr(handle->hndl, TCSANOW, term) != 0)
    {
        return -1;
    }

    return 0;
}
#endif

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
int
mbus_serial_disconnect(mbus_handle *handle)
{
    if (handle == NULL)
    {
        return -1;
    }

	close_serial_port(handle);

    return 0;
}

void
mbus_serial_data_free(mbus_handle *handle)
{
    mbus_serial_data *serial_data;

    if (handle)
    {
        serial_data = (mbus_serial_data *) handle->auxdata;

        if (serial_data == NULL)
        {
            return;
        }

		free(serial_data->t);
        free(serial_data->device);
        free(serial_data);
        handle->auxdata = NULL;
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
int
mbus_serial_send_frame(mbus_handle *handle, mbus_frame *frame)
{
    unsigned char buff[PACKET_BUFF_SIZE];
    int len, ret;

    if (handle == NULL || frame == NULL)
    {
        return -1;
    }

    // Make sure serial connection is open
	if (isatty(handle->hndl) == 0)
    {
        return -1;
    }

    if ((len = mbus_frame_pack(frame, buff, sizeof(buff))) == -1)
    {
        fprintf(stderr, "%s: mbus_frame_pack failed\n", __PRETTY_FUNCTION__);
        return -1;
    }

#ifdef MBUS_SERIAL_DEBUG
    // if debug, dump in HEX form to stdout what we write to the serial port
    printf("%s: Dumping M-Bus frame [%d bytes]: ", __PRETTY_FUNCTION__, len);
    int i;
    for (i = 0; i < len; i++)
    {
       printf("%.2X ", buff[i]);
    }
    printf("\n");
#endif

	if ((ret = write_serial_port(handle, buff, len)) == len)
    {
        //
        // call the send event function, if the callback function is registered
        //
        if (handle->send_event)
                handle->send_event(MBUS_HANDLE_TYPE_SERIAL, buff, len);
    }
    else
    {
        fprintf(stderr, "%s: Failed to write frame to socket (ret = %d: %s)\n", __PRETTY_FUNCTION__, ret, strerror(errno));
        return -1;
    }

    //
    // wait until complete frame has been transmitted
    //
	tcdrain(handle->hndl);

    return 0;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
int
mbus_serial_recv_frame(mbus_handle *handle, mbus_frame *frame)
{
    char buff[PACKET_BUFF_SIZE];
    int remaining, timeouts;
    ssize_t len, nread;

    if (handle == NULL || frame == NULL)
    {
        fprintf(stderr, "%s: Invalid parameter.\n", __PRETTY_FUNCTION__);
        return MBUS_RECV_RESULT_ERROR;
    }

    // Make sure serial connection is open
	if (isatty(handle->hndl) == 0)
    {
        fprintf(stderr, "%s: Serial connection is not available.\n", __PRETTY_FUNCTION__);
        return MBUS_RECV_RESULT_ERROR;
    }

    memset((void *)buff, 0, sizeof(buff));

    //
    // read data until a packet is received
    //
    remaining = 1; // start by reading 1 byte
    len = 0;
    timeouts = 0;

    do {
        if (len + remaining > PACKET_BUFF_SIZE)
        {
            // avoid out of bounds access
            return MBUS_RECV_RESULT_ERROR;
        }

        //printf("%s: Attempt to read %d bytes [len = %d]\n", __PRETTY_FUNCTION__, remaining, len);

		if ((nread = read_serial_port(handle, &buff[len], remaining)) == -1)
        {
       //     fprintf(stderr, "%s: aborting recv frame (remaining = %d, len = %d, nread = %d)\n",
         //          __PRETTY_FUNCTION__, remaining, len, nread);
            return MBUS_RECV_RESULT_ERROR;
        }

//   printf("%s: Got %d byte [remaining %d, len %d]\n", __PRETTY_FUNCTION__, nread, remaining, len);

        if (nread == 0)
        {
            timeouts++;

            if (timeouts >= 3)
            {
                // abort to avoid endless loop
                fprintf(stderr, "%s: Timeout\n", __PRETTY_FUNCTION__);
                break;
            }
        }

        if (len > (SSIZE_MAX-nread))
        {
            // avoid overflow
            return MBUS_RECV_RESULT_ERROR;
        }

        len += nread;

    } while ((remaining = mbus_parse(frame, buff, len)) > 0);

    if (len == 0)
    {
        // No data received
        return MBUS_RECV_RESULT_TIMEOUT;
    }

    //
    // call the receive event function, if the callback function is registered
    //
    if (handle->recv_event)
        handle->recv_event(MBUS_HANDLE_TYPE_SERIAL, buff, len);

    if (remaining != 0)
    {
        // Would be OK when e.g. scanning the bus, otherwise it is a failure.
        // printf("%s: M-Bus layer failed to receive complete data.\n", __PRETTY_FUNCTION__);
        return MBUS_RECV_RESULT_INVALID;
    }

    if (len == -1)
    {
        fprintf(stderr, "%s: M-Bus layer failed to parse data.\n", __PRETTY_FUNCTION__);
        return MBUS_RECV_RESULT_ERROR;
    }

    return MBUS_RECV_RESULT_OK;
}

