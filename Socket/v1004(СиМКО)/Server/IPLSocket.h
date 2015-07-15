//IPLSocket.h - Inter Platform Library for Socket by izvolsky@dc.ru, February 2004

#ifndef _IPLSOCKET__H_
#define _IPLSOCKET__H_

//#define _WIN32 //Target OS (MS Windows or not)

//#ifndef _WIN32
//#define TARGET_QNX
//#endif

#ifndef _WIN32

#include <sys/socket.h>
#include <sys/time.h>
//#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <netdb.h>
#include <errno.h>
#include "windows.h"

typedef	int 					SOCKET;
typedef hostent					*PHOSTENT,HOSTENT;
typedef sockaddr				*PSOCKADDR,SOCKADDR;
typedef	sockaddr_in				SOCKADDR_IN;
#define INVALID_SOCKET			-1
#define SOCKET_ERROR			-1
#define closesocket				close

#else //it's _WIN32

#include <process.h>
#include <windows.h>
#include <conio.h>
#include <winsock.h>

#endif //#ifndef _WIN32

//Common includes
#include <stdio.h>

#define SOCKET_TIMEOUT_SEND		5000 //Default socket waiting timeout for send in ms
#define SOCKET_TIMEOUT_RECEIVE	5000 //Default socket waiting timeout for receive in ms

//defs for error log
#define UNKNOWN_FUNCTION				0
#define WSASTARTUP_FUNCTION				1
#define SOCKET_FUNCTION					2
#define BIND_FUNCTION					3
#define LISTEN_FUNCTION					4
#define ACCEPT_FUNCTION					5
#define SETRECVTO_FUNCTION				6
#define SETSENDTO_FUNCTION				7
#define RECV_FUNCTION					8
#define SEND_FUNCTION					9
#define _BEGINTHREADEX_FUNCTION			10
#define GETHOSTBYNAME_FUNCTION			11
#define CONNECT_FUNCTION				12
#define CLIENTC_OPEN_FUNCTION			13
#define CLIENTC_CLOSEHANDLE_FUNCTON		14
#define CLIENTC_SENDMESS_FUNCTON		15
#define SERVER_SETSERVER_FUNCTON		16

//Custom error definitions
#define CREATE_EVENT_ERROR				0x80000001
#define ALLOC_MEMORY_ERROR				0x80000002
#define THREAD_START_ERROR				0x80000003
#define INVALID_CONNECT_STRING_ERROR	0x80000004
#define INVALID_APPLICATION_NAME_ERROR	0x80000005
#define NO_FREE_SLOT_ERROR				0x80000006
#define INVALID_HANDLE_ERROR			0x80000007
#define INVALID_PARAMETER_ERROR			0x80000008
#define ALREADY_INITED_ERROR			0x80000009

//Format of data contains info about error
typedef struct
{
	int ErrorCode; //Code of error
	int ErrorSource; //Function id which made error

} ErrorStorage_t, *pErrorStorage_t;

//Get code of last socket error
int GetLastSocketError(); 

//Get name of the function by it's code
char *GetFunctionName(int EventSource,char *ErrorMessage);

//Make valid error string
char *FormatErrorString(pErrorStorage_t hErrStruct);

//Get description of socket error
char *GetDescSocketError(int errcode);

//Set timeout for receive operations
int SetRecvTO(SOCKET oSocket,int Timeout);

//Set timeout for send operations
int SetSendTO(SOCKET oSocket,int Timeout);

#endif
