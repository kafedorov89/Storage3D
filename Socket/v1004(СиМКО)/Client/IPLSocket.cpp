#include "IPLSocket.h"

//-----------------------------------------------------------------
//Get code of last socket error
//-----------------------------------------------------------------
int GetLastSocketError()
{
	int socketerrorcode;

#ifdef _WIN32 //In Windows
	socketerrorcode=WSAGetLastError();
#else //Collect errors in QNX
	socketerrorcode=errno;
#endif

	return(socketerrorcode);
}

//-----------------------------------------------------------------
//Get function name
//-----------------------------------------------------------------
char *GetFunctionName(int EventSource,char *ErrorMessage)
{
	switch (EventSource)
	{

	case WSASTARTUP_FUNCTION: 
		sprintf(ErrorMessage,"%s","WSAStartup()");
		break;

	case SOCKET_FUNCTION: 
		sprintf(ErrorMessage,"%s","socket()");
		break;

	case BIND_FUNCTION: 
		sprintf(ErrorMessage,"%s","bind()");
		break;

	case LISTEN_FUNCTION: 
		sprintf(ErrorMessage,"%s","listen()");
		break;

	case ACCEPT_FUNCTION: 
		sprintf(ErrorMessage,"%s","accept()");
		break;

	case SETRECVTO_FUNCTION: 
		sprintf(ErrorMessage,"%s","SetRecvTO()");
		break;

	case SETSENDTO_FUNCTION: 
		sprintf(ErrorMessage,"%s","SetSendTO()");
		break;

	case RECV_FUNCTION: 
		sprintf(ErrorMessage,"%s","recv()");
		break;

	case SEND_FUNCTION: 
		sprintf(ErrorMessage,"%s","send()");
		break;

	case _BEGINTHREADEX_FUNCTION:
		sprintf(ErrorMessage,"%s","_beginthreadex()");
		break;

	case GETHOSTBYNAME_FUNCTION:
		sprintf(ErrorMessage,"%s","gethostbyname()");
		break;

	case CONNECT_FUNCTION:
		sprintf(ErrorMessage,"%s","connect()");
		break;

	case CLIENTC_OPEN_FUNCTION:
		sprintf(ErrorMessage,"%s","ClientC_Open()");
		break;

	case CLIENTC_CLOSEHANDLE_FUNCTON:
		sprintf(ErrorMessage,"%s","ClientC_CloseHandle()");
		break;

	case CLIENTC_SENDMESS_FUNCTON:
		sprintf(ErrorMessage,"%s","ClientC_SendMess()");
		break;

	case SERVER_SETSERVER_FUNCTON:
		sprintf(ErrorMessage,"%s","Server_SetServer()");
		break;

	default:
		sprintf(ErrorMessage,"%s%d","Unknown function. Code ",EventSource);
		break;
	}
	return(ErrorMessage);
}

//-----------------------------------------------------------------
//Make valid error string
//-----------------------------------------------------------------
char * FormatErrorString(pErrorStorage_t hErrStruct)
{
	static char ErrorMessage[1000];

	memset(ErrorMessage,0,1000);

	//retrieve function name
	GetFunctionName(hErrStruct->ErrorSource,ErrorMessage);

	sprintf(ErrorMessage+strlen(ErrorMessage),
		" failed. Error code: %d. Error description: %s",
		hErrStruct->ErrorCode,
		GetDescSocketError(hErrStruct->ErrorCode)
		);

	return(ErrorMessage);
}

//-----------------------------------------------------------------
//Get description of socket error
//-----------------------------------------------------------------
char *GetDescSocketError(int errcode)
{

	static char ErrorDescription[1000];
	memset(ErrorDescription,0,1000);

	//See if it's an our custom error
	switch (errcode)
	{
	case CREATE_EVENT_ERROR:	
		sprintf(ErrorDescription,"Can't create event object.");
		break;

	case ALLOC_MEMORY_ERROR:	
		sprintf(ErrorDescription,"Can't allocate memory.");
		break;

	case THREAD_START_ERROR:	
		sprintf(ErrorDescription,"Can't start servicing thread.");
		break;

	case INVALID_CONNECT_STRING_ERROR:	
		sprintf(ErrorDescription,"Connect string syntax invalid.");
		break;

	case INVALID_APPLICATION_NAME_ERROR:	
		sprintf(ErrorDescription,"Target application name invalid.");
		break;

	case NO_FREE_SLOT_ERROR:	
		sprintf(ErrorDescription,"Too much server handles opened. Close unused handles to free memory.");
		break;

	case INVALID_HANDLE_ERROR:	
		sprintf(ErrorDescription,"Handle points to free server slot.");
		break;

	case INVALID_PARAMETER_ERROR:
		sprintf(ErrorDescription,"Invalid input parameter in function.");
		break;

	case ALREADY_INITED_ERROR:
		sprintf(ErrorDescription,"Server already inited.");
		break;

	};

	if (strlen(ErrorDescription))
	{
		return(ErrorDescription);
	}

	char *lpMsgBuf;
#ifdef _WIN32 //In Windows

	if (!FormatMessage( 
		FORMAT_MESSAGE_ALLOCATE_BUFFER | 
		FORMAT_MESSAGE_FROM_SYSTEM | 
		FORMAT_MESSAGE_IGNORE_INSERTS,
		NULL,
		errcode,
		MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), // Default language
		(LPTSTR) &lpMsgBuf,
		0,
		NULL ))
	{
		// Handle the error.
		return(NULL);
	}

#else //Collect errors in QNX

	lpMsgBuf = strerror(errcode);

#endif
	return (lpMsgBuf);
}

//----------------------------------------------------------------------------------
//Set timeout for receive operations
//----------------------------------------------------------------------------------
int SetRecvTO(SOCKET oSocket,int Timeout)
{
	int socketerrorcode;

#ifdef _WIN32 //Set recv() timeout in Windows

	socketerrorcode=setsockopt(oSocket,SOL_SOCKET,SO_RCVTIMEO,(char *)&Timeout,sizeof(int));

#else //Set recv() timeout in QNX

	struct timeval hwmch;	

	hwmch.tv_sec=(long)(Timeout/1000);
	hwmch.tv_usec=(Timeout-hwmch.tv_sec*1000)*1000;

	socketerrorcode=setsockopt(oSocket,SOL_SOCKET,SO_RCVTIMEO,(void *)&hwmch,sizeof(hwmch));

#endif

	return socketerrorcode;
}

//----------------------------------------------------------------------------------
//Set timeout for send operations
//----------------------------------------------------------------------------------
int SetSendTO(SOCKET oSocket,int Timeout)
{
	int socketerrorcode;

#ifdef _WIN32 //Set recv() timeout in Windows

	socketerrorcode=setsockopt(oSocket,SOL_SOCKET,SO_SNDTIMEO,(char *)&Timeout,sizeof(int));

#else //Set recv() timeout in QNX

	struct timeval hwmch;	

	hwmch.tv_sec=(long)(Timeout/1000);
	hwmch.tv_usec=(Timeout-hwmch.tv_sec*1000)*1000;

	socketerrorcode=setsockopt(oSocket,SOL_SOCKET,SO_SNDTIMEO,(void *)&hwmch,sizeof(hwmch));

#endif

	return socketerrorcode;
}

