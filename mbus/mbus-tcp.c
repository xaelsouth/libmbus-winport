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
#include <fcntl.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>

#ifndef _WIN32
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>

#include <strings.h>
#endif

#include "mbus-tcp.h"

#define PACKET_BUFF_SIZE 2048

//------------------------------------------------------------------------------
/// Setup a TCP/IP handle.
//------------------------------------------------------------------------------
int
mbus_tcp_connect(mbus_handle *handle)
{
    char error_str[128], *host;    
	struct addrinfo hints, *result, *rp;
    struct sockaddr_in s;
    struct timeval time_out;
    mbus_tcp_data *tcp_data;
    uint16_t port;

    if (handle == NULL)
        return -1;

    tcp_data = (mbus_tcp_data *) handle->auxdata;
    if (tcp_data == NULL || tcp_data->host == NULL)
        return -1;

    host = tcp_data->host;
    port = tcp_data->port;

    /* resolve hostname */
	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;     /* Allow IPv4 or IPv6 */
	hints.ai_socktype = SOCK_STREAM; /* Stream socket */
	hints.ai_flags = AI_PASSIVE;     /* For wildcard IP address */
	hints.ai_protocol = 0;           /* Any protocol */
	hints.ai_canonname = NULL;
	hints.ai_addr = NULL;
	hints.ai_next = NULL;

	if (getaddrinfo(NULL, host, &hints, &result) != 0)
	{
		snprintf(error_str, sizeof(error_str), "%s: unknown host: %s", __PRETTY_FUNCTION__, host);
		mbus_error_str_set(error_str);
		return -1;
	}

	//
	// create the TCP connection
	//
	for (rp = result; rp != NULL; rp = rp->ai_next)
	{
		handle->sock = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);

		if (handle->sock == MBUS_INVALID_SOCKET)
			continue;

		if (connect(handle->sock, (struct sockaddr *)&s, sizeof(s)) == 0)
			break;
		else
			closesocket(handle->sock);
	}

	if (rp == NULL)
	{
		snprintf(error_str, sizeof(error_str), "%s: Failed to establish connection to %s:%d", __PRETTY_FUNCTION__, host, port);
		mbus_error_str_set(error_str);
		return -1;
	}

    // Set a timeout
	time_out.tv_sec = handle->tcp_timeout_sec;   // seconds
	time_out.tv_usec = handle->tcp_timeout_usec;  // microseconds
    setsockopt(handle->sock, SOL_SOCKET, SO_SNDTIMEO, (void *)&time_out, sizeof(time_out));
	setsockopt(handle->sock, SOL_SOCKET, SO_RCVTIMEO, (void *)&time_out, sizeof(time_out));

    return 0;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void
mbus_tcp_data_free(mbus_handle *handle)
{
    mbus_tcp_data *tcp_data;

    if (handle)
    {
        tcp_data = (mbus_tcp_data *) handle->auxdata;

        if (tcp_data == NULL)
        {
            return;
        }

        free(tcp_data->host);
        free(tcp_data);
        handle->auxdata = NULL;
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
int
mbus_tcp_disconnect(mbus_handle *handle)
{
    if (handle == NULL)
    {
        return -1;
    }

	closesocket(handle->sock);

    return 0;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
int
mbus_tcp_send_frame(mbus_handle *handle, mbus_frame *frame)
{
    unsigned char buff[PACKET_BUFF_SIZE];
    int len, ret;
    char error_str[128];

    if (handle == NULL || frame == NULL)
    {
        return -1;
    }

    if ((len = mbus_frame_pack(frame, buff, sizeof(buff))) == -1)
    {
        snprintf(error_str, sizeof(error_str), "%s: mbus_frame_pack failed\n", __PRETTY_FUNCTION__);
        mbus_error_str_set(error_str);
        return -1;
    }

	if ((ret = send(handle->sock, buff, len, 0)) == len)
    {
        //
        // call the send event function, if the callback function is registered
        //
        if (handle->send_event)
            handle->send_event(MBUS_HANDLE_TYPE_TCP, buff, len);
    }
    else
    {
        snprintf(error_str, sizeof(error_str), "%s: Failed to write frame to socket (ret = %d)\n", __PRETTY_FUNCTION__, ret);
        mbus_error_str_set(error_str);
        return -1;
    }

    return 0;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
int mbus_tcp_recv_frame(mbus_handle *handle, mbus_frame *frame)
{
    char buff[PACKET_BUFF_SIZE];
    int remaining;
    ssize_t len, nread;

    if (handle == NULL || frame == NULL) {
        fprintf(stderr, "%s: Invalid parameter.\n", __PRETTY_FUNCTION__);
        return MBUS_RECV_RESULT_ERROR;
    }

    memset((void *) buff, 0, sizeof(buff));

    //
    // read data until a packet is received
    //
    remaining = 1; // start by reading 1 byte
    len = 0;

    do {
retry:
        if (len + remaining > PACKET_BUFF_SIZE)
        {
            // avoid out of bounds access
            return MBUS_RECV_RESULT_ERROR;
        }

		nread = recv(handle->sock, &buff[len], remaining, 0);
        switch (nread) {
        case -1:
			if (mbus_socket_errno == MBUS_EINTR)
                goto retry;

			if (mbus_socket_errno == MBUS_EAGAIN || mbus_socket_errno == MBUS_EWOULDBLOCK) {
                mbus_error_str_set("M-Bus tcp transport layer response timeout has been reached.");
                return MBUS_RECV_RESULT_TIMEOUT;
            }

            mbus_error_str_set("M-Bus tcp transport layer failed to read data.");
            return MBUS_RECV_RESULT_ERROR;
        case 0:
            mbus_error_str_set("M-Bus tcp transport layer connection closed by remote host.");
            return MBUS_RECV_RESULT_RESET;
        default:
            if (len > (SSIZE_MAX-nread))
            {
                // avoid overflow
                return MBUS_RECV_RESULT_ERROR;
            }

            len += nread;
        }
    } while ((remaining = mbus_parse(frame, buff, len)) > 0);

    //
    // call the receive event function, if the callback function is registered
    //
    if (handle->recv_event)
        handle->recv_event(MBUS_HANDLE_TYPE_TCP, buff, len);

    if (remaining < 0) {
        mbus_error_str_set("M-Bus layer failed to parse data.");
        return MBUS_RECV_RESULT_INVALID;
    }

    return MBUS_RECV_RESULT_OK;
}

//------------------------------------------------------------------------------
/// The the timeout in seconds that will be used as the amount of time the
/// a read operation will wait before giving up. Note: This configuration has
/// to be made before calling mbus_tcp_connect.
//------------------------------------------------------------------------------
int
mbus_tcp_set_timeout_set(mbus_handle *handle, double seconds)
{
	if (handle == NULL)
		return -1;

    if (seconds < 0.0)
    {
        mbus_error_str_set("Invalid timeout (must be positive).");
        return -1;
    }

	handle->tcp_timeout_sec = (long)seconds;
	handle->tcp_timeout_usec = (long)(seconds - handle->tcp_timeout_sec) * 1000000ll;

    return 0;
}

