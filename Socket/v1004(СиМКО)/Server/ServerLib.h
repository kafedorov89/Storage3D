#ifndef _SERVERLIB__H_
#define _SERVERLIB__H_

//#include "CircleBuffer.h"
#include "IPLSocket.h"
#include "ClSrvDef.h"
#include "CServer.h"
#ifdef _WIN32
#define WaitForEventObject WaitForSingleObject
typedef int socklen_t;
#endif

#define MAX_PENDING_CONNECTS	4     // Maximum length of the queue pending connections
#define SERVER_THREAD_MIN_COUNT		2 //Min nount of server socket threads
#define SERVER_THREAD_MAX_COUNT		20 //Max count of server threads
/*
#define CIRCLE_BUFFER_ACCEPTED_SOCKET_SIZE 10 //Accepted sockets buffer size
#define CIRCLE_BUFFER_SOCKET_ERROR_SIZE 1000 //Socket errors buffer size
#define CIRCLE_BUFFER_SOCKET_STATUS_SIZE 1000 //Socket status buffer size
*/
//Format of data contains status info
typedef struct
{

	int FuncNumb; //Function id which make event
	int Argue; //Code

} StatusStorage_t, *pStatusStorage_t;

//This helps operate with ip address
typedef struct
{ 

	u_char s_b1,s_b2,s_b3,s_b4;

} IPADDRESS_t;

//Server description
typedef struct {
	bool				bInited;			//Флаг, был ли уже SetServer
	unsigned			FreeThreadsCount;	//кол-во свободных потоков

	BarsCallBack		*hCallBack;			//callback serve requests
	BarsCallBackEx		*hCallBackEx;		//extended callback serve requests
	BarsErrorCallBack	*hErrorCallBack;	//server error events
	BarsStatusCallBack	*hStatusCallBack;	//server status events

	void *hCallBackExParam;					//handle to extended callback parameter
	void *hErrorCallBackParam;				//handle to error callback parameter
	void *hStatusCallBackParam;				//handle to status event callback parameter

	int SendTimeOut;			//Timeout in ms for send()
	int RecvTimeOut;			//Timeout in ms for recv()

	HANDLE DispathThread;					//client requests dispather thread
//	HANDLE ErrorThread;						//error thread handler
//	HANDLE StatusThread;						//status event thread handler
	HANDLE arThreads[SERVER_THREAD_MAX_COUNT]; //handles for serving threads

	int ServerThreadsCount;		//Count of server threads now.

	char ServerName[1000];		//name of the Server

	HANDLE hSockAcceptEvent;	//Handle to event which will be set when connection is accepted
	HANDLE hSockAccepted;	//Если сокет схвачен, то этот Event взводится
	/*
	HANDLE hErrOccuredEvent;	//Handle to event which will be set when error is occured
	HANDLE hStatusEvent;		//Handle to event which will be set when anything happens
	*/

	SOCKET SrvSocket;			// server socket

	SOCKADDR_IN local_sin;      // Local socket address
	SOCKADDR_IN accept_sin;             
	socklen_t accept_sin_len;         // Length of accept_sin
	/*
	CircleBuffer_t oCBAccSock;		//handle to the Accepted Sockets buffer

	CircleBuffer_t oCBSockErr;		//handle to the Socket Errors buffer
	CircleBuffer_t oCBSockStatus;	//handle to the Socket Events buffer
	*/

	CRITICAL_SECTION	csError;	//Для CALLBACK при ошибке
	CRITICAL_SECTION	csStatus;	//Для CALLBACK при собитии

	ErrorStorage_t	LastError;		//сюда сохраняем код последней ошибки
	StatusStorage_t	LastStatus;		//сюда сохраняем последнее действие

	SOCKET	ClientSocket;			//Место для передачи сокета потоку обслуживания
} Server_Ion_t, *pServer_Ion_t;

//Save error code and source into the cycle buffer
void Server_SaveSocketError(int socketerrorcode,int errorsource);

//Save socket status code into the cycle buffer
void Server_SaveSocketStatus(int statussource,int argument);

//Make valid status info string
char * FormatStatusString(pStatusStorage_t hStatusStruct);

//Make valid status info string
char *GetDescSocketStatus(pStatusStorage_t hStatusStruct);

//--------------------------------------------------------
//server's threads
//--------------------------------------------------------

//Thread which serve client's requests
unsigned int __stdcall server_listener_thread (void *arg);

//Thread which dispatch client's requests
unsigned int __stdcall server_dispather_thread (void *arg);
/*
//Thread which handles errors
unsigned int __stdcall server_error_thread (void *arg);

//Thread which handles status event
unsigned int __stdcall server_status_thread (void *arg);
*/
#endif //_SERVERLIB__H_
