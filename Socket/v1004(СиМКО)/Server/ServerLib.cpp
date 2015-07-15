#include "ServerLib.h"

//storage for server features
Server_Ion_t oServerDescription=
{
	false,	//bInited
	0,		//FreeThreadsCount
	NULL,	//hCallBack
	NULL,	//hCallBackEx
	NULL,	//hErrorCallBack
	NULL,	//hStatusCallBack
	NULL,	//hCallBackExParam
	NULL,	//hErrorCallBackParam
	NULL,	//hStatusCallBackParam
	SOCKET_TIMEOUT_SEND,	//SendTimeOut
	SOCKET_TIMEOUT_RECEIVE	//RecvTimeOut
};

//-----------------------------------------------------------------
//Save error code and source
//-----------------------------------------------------------------
void Server_SaveSocketError(int socketerrorcode,int errorsource)
{
	EnterCriticalSection(&oServerDescription.csError);
	oServerDescription.LastError.ErrorCode = socketerrorcode;
	oServerDescription.LastError.ErrorSource = errorsource;
	if (oServerDescription.hErrorCallBack)
		oServerDescription.hErrorCallBack(oServerDescription.hErrorCallBackParam);
	LeaveCriticalSection(&oServerDescription.csError);
}

//-----------------------------------------------------------------
//Save status source
//-----------------------------------------------------------------
void Server_SaveSocketStatus(int statussource,int argument)
{
	EnterCriticalSection(&oServerDescription.csStatus);
	oServerDescription.LastStatus.Argue = argument;
	oServerDescription.LastStatus.FuncNumb = statussource;
	if (oServerDescription.hStatusCallBack)
		oServerDescription.hStatusCallBack(oServerDescription.hStatusCallBackParam);
	LeaveCriticalSection(&oServerDescription.csStatus);
}

//-------------------------------------------------------------------------
//Startup server
//-------------------------------------------------------------------------
int Server_SetServer(int inPort,char *Name)
{
	int iReturn;
	
	if (oServerDescription.bInited) 
	{
		Server_SaveSocketError(ALREADY_INITED_ERROR,SERVER_SETSERVER_FUNCTON);		
		return(-1);
	}

	InitializeCriticalSection(&oServerDescription.csError);
	InitializeCriticalSection(&oServerDescription.csStatus);

	if (!Name)
	{
		Server_SaveSocketError(INVALID_PARAMETER_ERROR,SERVER_SETSERVER_FUNCTON);
		return(-1);
	}

	//Set Server name
	strcpy(oServerDescription.ServerName,Name);

	//Create event object to notify serving threads
	oServerDescription.hSockAcceptEvent=(HANDLE)CreateEvent(NULL,false,false,NULL); //Alloc memory for one event object
	oServerDescription.hSockAccepted=(HANDLE)CreateEvent(NULL,false,false,NULL); //Alloc memory for one event object

	if ((!oServerDescription.hSockAcceptEvent) || (!oServerDescription.hSockAccepted))
	{
		Server_SaveSocketError(CREATE_EVENT_ERROR,SERVER_SETSERVER_FUNCTON);
	}

#ifdef _WIN32	
	WSADATA WSAData;                    // Contains details of the Winsock
	// Initialize Winsock.
	if (WSAStartup (MAKEWORD(2,2), &WSAData) != 0) 
	{
		Server_SaveSocketError(GetLastSocketError(),WSASTARTUP_FUNCTION);
		return(-1);
	}

#endif //_WIN32

	//create socket object
	oServerDescription.SrvSocket=socket(AF_INET, SOCK_STREAM, 0);
	if (oServerDescription.SrvSocket==INVALID_SOCKET)
	{
		Server_SaveSocketError(GetLastSocketError(),SOCKET_FUNCTION);
		closesocket (oServerDescription.SrvSocket);
		return(-1);
	}

	// Fill out the local socket's address information.
	oServerDescription.local_sin.sin_family = AF_INET;
	oServerDescription.local_sin.sin_port = htons (inPort);  
	oServerDescription.local_sin.sin_addr.s_addr = htonl (INADDR_ANY);

	iReturn = bind (oServerDescription.SrvSocket, (struct sockaddr *) &oServerDescription.local_sin,sizeof (oServerDescription.local_sin));
	if (iReturn == SOCKET_ERROR)
	{
		Server_SaveSocketError(GetLastSocketError(),BIND_FUNCTION);
		closesocket (oServerDescription.SrvSocket);
		return(-1);
	}

	iReturn = listen (oServerDescription.SrvSocket, MAX_PENDING_CONNECTS);
	if (iReturn == SOCKET_ERROR)
	{
		Server_SaveSocketError(GetLastSocketError(),LISTEN_FUNCTION);
		closesocket (oServerDescription.SrvSocket);
		return(-1);
	}

	for (oServerDescription.ServerThreadsCount=0;oServerDescription.ServerThreadsCount<SERVER_THREAD_MIN_COUNT;oServerDescription.ServerThreadsCount++)
	{
		oServerDescription.arThreads[oServerDescription.ServerThreadsCount]=(HANDLE)_beginthreadex(NULL,0,server_listener_thread,NULL,0,NULL);
		if (oServerDescription.arThreads[oServerDescription.ServerThreadsCount]==(HANDLE)(-1L))
		{
			Server_SaveSocketError(THREAD_START_ERROR,_BEGINTHREADEX_FUNCTION);
			return(-1);
		}
	}

	oServerDescription.accept_sin_len = sizeof (oServerDescription.accept_sin);	

	oServerDescription.DispathThread=(HANDLE)_beginthreadex(NULL,0,server_dispather_thread,NULL,0,NULL);	

	if (oServerDescription.DispathThread==(HANDLE)(-1L))
	{
		Server_SaveSocketError(THREAD_START_ERROR,_BEGINTHREADEX_FUNCTION);
		return(-1);
	}

	oServerDescription.bInited = true;
	return(0);
}

//-------------------------------------------------------------------------
// Close Socket-server
//-------------------------------------------------------------------------
int Server_Close()
{
 if (oServerDescription.SrvSocket != INVALID_SOCKET)
  {
   closesocket (oServerDescription.SrvSocket);
   oServerDescription.SrvSocket = INVALID_SOCKET;
  }

 return(oServerDescription.SrvSocket == INVALID_SOCKET ? 0 : -1);
}

//-----------------------------------------------------------------
//Make valid status info string
//-----------------------------------------------------------------
char *GetDescSocketStatus(pStatusStorage_t hStatusStruct)
{
	static char DescStatus[1000];
	memset(DescStatus,0,1000);

	IPADDRESS_t *hRmtAddr;

	switch (hStatusStruct->FuncNumb)
	{

	case ACCEPT_FUNCTION: 
		hRmtAddr=(IPADDRESS_t *)&hStatusStruct->Argue;
		sprintf(DescStatus,"Accepted socket ip %d.%d.%d.%d",
			hRmtAddr->s_b1,
			hRmtAddr->s_b2,
			hRmtAddr->s_b3,
			hRmtAddr->s_b4);
		break;

	case SETRECVTO_FUNCTION: 
		sprintf(DescStatus,"New receive timeout is %d ms",hStatusStruct->Argue);
		break;

	case SETSENDTO_FUNCTION: 
		sprintf(DescStatus,"New send timeout is %d ms",hStatusStruct->Argue);
		break;

	case RECV_FUNCTION: 
		hRmtAddr=(IPADDRESS_t *)&hStatusStruct->Argue;
		sprintf(DescStatus,"Received data on socket ip %d.%d.%d.%d",
			hRmtAddr->s_b1,
			hRmtAddr->s_b2,
			hRmtAddr->s_b3,
			hRmtAddr->s_b4);
		break;

	case SEND_FUNCTION: 
		hRmtAddr=(IPADDRESS_t *)&hStatusStruct->Argue;
		sprintf(DescStatus,"Sent data on socket ip %d.%d.%d.%d",
			hRmtAddr->s_b1,
			hRmtAddr->s_b2,
			hRmtAddr->s_b3,
			hRmtAddr->s_b4);
		break;

	case _BEGINTHREADEX_FUNCTION:
		sprintf(DescStatus,"Now there is %d service threads",hStatusStruct->Argue);
		break;

	default:
		sprintf(DescStatus,"Argument is %d",hStatusStruct->Argue);
		break;
	}

	return(DescStatus);
}

//-----------------------------------------------------------------
//Make valid status info string
//-----------------------------------------------------------------
char * FormatStatusString(pStatusStorage_t hStatusStruct)
{
	static char StatusMessage[1000];

	memset(StatusMessage,0,1000);

	//retrieve function name
	GetFunctionName(hStatusStruct->FuncNumb,StatusMessage);

	sprintf(StatusMessage+strlen(StatusMessage)," complete successfully. %s.",GetDescSocketStatus(hStatusStruct));

	return(StatusMessage);
}

//---------------------------------------------------------------------------
//Retrieve description for last server error
//---------------------------------------------------------------------------
char *Server_GetLastServerError(void)
{
	char *ErrorDescr;

	//Get and format error message
	ErrorDescr=FormatErrorString((pErrorStorage_t)&oServerDescription.LastError);

	return(ErrorDescr);
}

//---------------------------------------------------------------------------
//Retrieve description for last server status event
//---------------------------------------------------------------------------
char *Server_GetLastServerStatusEvent(void)
{
	char *EventDescr;

	//Get and format status event message
	EventDescr=FormatStatusString(&oServerDescription.LastStatus);

	return(EventDescr);
}

//---------------------------------------------------------------------------
//Needed for community
//---------------------------------------------------------------------------
int Server_SetNoRegisterChannelFlag(int flag)
{
	return(0);
}

//-------------------------------------------------------------------------
//Set callback for serving client's threads
//-------------------------------------------------------------------------
int Server_SetCallBack( BarsCallBack *C )
{
	//Initialize callback
	oServerDescription.hCallBack=C;
	return(0);
}

//----------------------------------------------------------------------------------
//Set extended callback for serving client's threads
//----------------------------------------------------------------------------------
int Server_SetCallBackEx( BarsCallBackEx *C, void *param )
{
	//Initialize extended callback
	oServerDescription.hCallBackEx=C;
	oServerDescription.hCallBackExParam=param;
	return(0);
}

//----------------------------------------------------------------------------------
//Set error callback
//----------------------------------------------------------------------------------
int Server_SetErrorCallBack( BarsErrorCallBack *C, void *param )
{
	//Initialize error callback
	oServerDescription.hErrorCallBack=C;
	oServerDescription.hErrorCallBackParam=param;
	return(0);
}

//----------------------------------------------------------------------------------
//Set status callback
//----------------------------------------------------------------------------------
int Server_SetStatusCallBack( BarsStatusCallBack *C, void *param )
{
	//Initialize status callback
	oServerDescription.hStatusCallBack=C;
	oServerDescription.hStatusCallBackParam=param;
	return(0);
}

//---------------------------------------------------------------------
// Thread which serve client's requests
//---------------------------------------------------------------------
// Nigmatullin+
char RequestStr [CLSRVDEF_COMMUNICATION_DATA_SIZE]; //String for client's request
char ResponseStr[CLSRVDEF_COMMUNICATION_DATA_SIZE]; //String for response to client
// Nigmatullin-
unsigned int __stdcall server_listener_thread (void *arg)
{ 
 int   idError = 0;
 int   iReturn = 0;
 int   RequestId       = 0;
 ULONG ResponseStrSize = 0;

 sockaddr_in oSockAddress;
 socklen_t   iSizeOfSockAddress;

 SOCKET ClientSock; // Socket for communicating 

 for (;;)
  {
   // Clear previous error
   idError=0;

   // Clear string for transferring data
   memset(RequestStr, 0, CLSRVDEF_COMMUNICATION_DATA_SIZE);
   memset(ResponseStr,0, CLSRVDEF_COMMUNICATION_DATA_SIZE);

   // Wait if no sockets accepted
   oServerDescription.FreeThreadsCount++;
   WaitForEventObject(oServerDescription.hSockAcceptEvent,INFINITE);
   oServerDescription.FreeThreadsCount--;

   // Get handle to the accepted socket
   memcpy(&ClientSock,&oServerDescription.ClientSocket,sizeof(SOCKET));

   SetEvent(oServerDescription.hSockAccepted);
  // Set timeout for recv()
  iReturn = SetRecvTO(ClientSock, oServerDescription.RecvTimeOut);

  if (iReturn == SOCKET_ERROR)
   {
    idError = GetLastSocketError();
    Server_SaveSocketError(idError,SETRECVTO_FUNCTION);
    closesocket (ClientSock);
   }
  else
   {
    if (oServerDescription.hStatusCallBack)
     {
      Server_SaveSocketStatus(SETRECVTO_FUNCTION,oServerDescription.RecvTimeOut);
     }
   }

  // Set timeout for send()
  if (!idError)
   {
    iReturn = SetSendTO(ClientSock,oServerDescription.SendTimeOut);

    if (iReturn == SOCKET_ERROR)
     {
      idError = GetLastSocketError();
      Server_SaveSocketError(idError,SETSENDTO_FUNCTION);
      closesocket (ClientSock);
     }
    else
     {
      if (oServerDescription.hStatusCallBack)
       {
        Server_SaveSocketStatus(SETSENDTO_FUNCTION, oServerDescription.SendTimeOut);
       }
     }
   }

  if (!idError)
   {
    RequestId       = 0;
    ResponseStrSize = 0;

    // Receive data from the client.
    iReturn = recv(ClientSock, RequestStr, CLSRVDEF_COMMUNICATION_DATA_SIZE, 0);

    // Check if there is any data received. If there is, display it.
    if (iReturn == SOCKET_ERROR)
     {
      idError = GetLastSocketError();
      Server_SaveSocketError(idError,RECV_FUNCTION);
      closesocket (ClientSock);
     }
    else
     {//received Ok
      if (oServerDescription.hStatusCallBack)
       {
        memset(&oSockAddress,0,sizeof(oSockAddress));
        iSizeOfSockAddress=sizeof(oSockAddress);
        getpeername(ClientSock,(sockaddr *)&oSockAddress,&iSizeOfSockAddress);
        Server_SaveSocketStatus(RECV_FUNCTION,*(int *)&oSockAddress.sin_addr);
       }

      // Nigmatullin - здесь было жестко - пустая строка - это запрос имени сервера 
      memcpy(&RequestId, RequestStr, sizeof(int));

      if (RequestId == 0)
       {
        // Server Name is requested
        ResponseStrSize = strlen(oServerDescription.ServerName) + 1;
        memcpy(ResponseStr, oServerDescription.ServerName, ResponseStrSize);
       }
      else
       {
        if (oServerDescription.hCallBackEx)
         {
          // If extended callback is registered
          iReturn = (*oServerDescription.hCallBackEx)(RequestStr, ResponseStr, &ResponseStrSize, oServerDescription.hCallBackExParam);
         }
        else if (oServerDescription.hCallBack)
              {
               // If standard callback is registered
               iReturn = (*oServerDescription.hCallBack)(RequestStr, ResponseStr, &ResponseStrSize);
              }

        // Nigmatullin+
        /*
        if (strlen(ResponseStr)==0)
         {
          // Nothing returned.
          if (oServerDescription.hCallBackEx || oServerDescription.hCallBack)
           {
            // return result code
            sprintf(ResponseStr,"%d",iReturn);					
           }
         }
        else// if (strlen(ResponseStr)>=CLSRVDEF_COMMUNICATION_STRING_LEN)
         {
          //overload output buffer
          ResponseStr[CLSRVDEF_COMMUNICATION_STRING_LEN-1]=0;
         }
        */				
        // Nigmatullin-
        
        if (ResponseStrSize > CLSRVDEF_COMMUNICATION_DATA_SIZE) ResponseStrSize = CLSRVDEF_COMMUNICATION_DATA_SIZE;
        ResponseStr[CLSRVDEF_COMMUNICATION_DATA_SIZE - 1] = 0;
       }//(RequestStr[0]==0)

     }//if (iReturn == SOCKET_ERROR)

    }//if (!idError)

   if (!idError)
    {
     // Send a string from the server to the client.
     if (send(ClientSock, ResponseStr, ResponseStrSize, 0) == SOCKET_ERROR) 
      {
       idError = GetLastSocketError();
       Server_SaveSocketError(idError,SEND_FUNCTION);
       closesocket (ClientSock);
      }
     else
      {
       if (oServerDescription.hStatusCallBack)
        {
         memset(&oSockAddress,0,sizeof(oSockAddress));
         iSizeOfSockAddress=sizeof(oSockAddress);
         getpeername(ClientSock,(sockaddr *)&oSockAddress,&iSizeOfSockAddress);

         Server_SaveSocketStatus(SEND_FUNCTION,*(int *)&oSockAddress.sin_addr);
        }
      }
    }

   closesocket (ClientSock);
  }//for

 return (true);
}

//---------------------------------------------------------------------
//Thread which dispatch client's requests
//---------------------------------------------------------------------
unsigned int __stdcall server_dispather_thread (void *arg)
{ 
	int idError;
	SOCKET ClientSock; // Socket for communicating 

	for (;;)
	{
		idError = 0;

		// Accept an incoming connection attempt on WinSocket.
		ClientSock = accept (oServerDescription.SrvSocket, 
			(struct sockaddr *) &oServerDescription.accept_sin, 
			&oServerDescription.accept_sin_len);

		if (ClientSock == INVALID_SOCKET) 
		{
			idError = GetLastSocketError();
			Server_SaveSocketError(idError,ACCEPT_FUNCTION);
			closesocket (oServerDescription.SrvSocket);
		}
		else
		{
			if (oServerDescription.hStatusCallBack)
			{
				Server_SaveSocketStatus(ACCEPT_FUNCTION,*(int *)&oServerDescription.accept_sin.sin_addr);
			}
		}


		if (
			(oServerDescription.FreeThreadsCount == 0) && 
			(oServerDescription.ServerThreadsCount < SERVER_THREAD_MAX_COUNT)
			)
		{//Если все потоки заняты, и их кол-во меньше максимума
		//то надо создать ещё один поток
			oServerDescription.arThreads[oServerDescription.ServerThreadsCount]=(HANDLE)_beginthreadex(NULL,0,server_listener_thread,NULL,0,NULL);
			if (oServerDescription.arThreads[oServerDescription.ServerThreadsCount]==(HANDLE)(-1L))
			{
				Server_SaveSocketError(THREAD_START_ERROR,_BEGINTHREADEX_FUNCTION);
//				return(0);
			}
			else
			{
				oServerDescription.ServerThreadsCount++;
				Server_SaveSocketStatus(_BEGINTHREADEX_FUNCTION,oServerDescription.ServerThreadsCount);
			}
		}

		memcpy(&oServerDescription.ClientSocket,&ClientSock, sizeof(SOCKET));
		//Push accepted socket into the buffer
//		Push_Buffer(&oServerDescription.oCBAccSock,(char *)&ClientSock);

		//Notify thread to begin serve
		SetEvent(oServerDescription.hSockAcceptEvent);

		WaitForEventObject(oServerDescription.hSockAccepted,INFINITE);
	}//for 
	return(0);
}

//---------------------------------------------------------------------
//Thread which handles errors
//---------------------------------------------------------------------
/*
unsigned int __stdcall server_error_thread (void *arg)
{ 

	for (;;)
	{

		//Wait for error occuring
		WaitForEventObject(oServerDescription.hErrOccuredEvent,INFINITE);

		if (oServerDescription.hErrorCallBack)
		{
			//Call callback, if exists
			(*oServerDescription.hErrorCallBack)(oServerDescription.hErrorCallBackParam);
		}

	}

	return(0);
}
*/
//---------------------------------------------------------------------
//Thread which handles status event
//---------------------------------------------------------------------
/*
unsigned int __stdcall server_status_thread (void *arg)
{ 

	for (;;)
	{

		if (IsBufferFree(&oServerDescription.oCBSockStatus))
		{
			//Wait for status change occuring
			WaitForEventObject(oServerDescription.hStatusEvent,INFINITE);
		}

		if (oServerDescription.hStatusCallBack)
		{
			//Call callback, if exists
			(*oServerDescription.hStatusCallBack)(oServerDescription.hStatusCallBackParam);
		}

	}

	return(0);
}
*/
//----------------------------------------------------------------------------------------
//Set send timeout
//----------------------------------------------------------------------------------------
void Server_SetSendTimeout(int millisec)
{
	oServerDescription.SendTimeOut=millisec;
}

//----------------------------------------------------------------------------------------
//Set receive timeout
//----------------------------------------------------------------------------------------
void Server_SetRecvTimeout(int millisec)
{
	oServerDescription.RecvTimeOut=millisec;
}
