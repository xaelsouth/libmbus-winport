#ifndef MBUS_PLATFORM_H
#define MBUS_PLATFORM_H

#ifdef _WIN32

#ifdef LIBMBUS_EXPORTS
#define ADDAPI __declspec(dllexport)
#else
#define ADDAPI __declspec(dllimport)
#endif

#define ADDCALL __cdecl

#else
#define ADDAPI
#define ADDCALL
#endif

#endif /* MBUS_PLATFORM_H */

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>

#pragma comment(lib, "ws2_32.lib")

#define MBUS_EINTR WSAEINTR
#define MBUS_EAGAIN WSAEWOULDBLOCK
#define MBUS_EWOULDBLOCK WSAEWOULDBLOCK
#define MBUS_EINPROGRESS WSAEINPROGRESS

#define MBUS_INVALID_SOCKET INVALID_SOCKET
#define MBUS_INVALID_HANDLE	INVALID_HANDLE_VALUE

#define mbus_errno GetLastError()
#define mbus_socket_errno WSAGetLastError()

#define gmtime_r(time,tm) gmtime_s(tm,time)	

#define tcdrain(hndl)
#define isatty(hndl)	(~0)

typedef SOCKET MBUS_SOCKET;
typedef HANDLE MBUS_HANDLE;

#else

#define MBUS_EINTR EINTR
#define MBUS_EAGAIN EWOULDBLOCK
#define MBUS_EWOULDBLOCK EWOULDBLOCK
#define MBUS_EINPROGRESS EINPROGRESS

#define MBUS_INVALID_SOCKET (-1)
#define MBUS_INVALID_HANDLE	(-1)

#define closesocket(fd) close(fd)

#define mbus_errno errno
#define mbus_socket_errno errno

typedef int MBUS_SOCKET;
typedef int MBUS_HANDLE;

#endif

#if defined(_MSC_VER)
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#define SSIZE_MAX MAXSIZE_T
#define __PRETTY_FUNCTION__ __FUNCTION__
#define snprintf(buf,len,fmt,...) _snprintf_s(buf,len,_TRUNCATE,fmt,__VA_ARGS__)
#define strdup(s) _strdup(s)
#endif
