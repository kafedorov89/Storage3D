#include "ClientLib.h"

//storage for client features
Client_Ion_t oClientDescription =
{
	SOCKET_TIMEOUT_SEND,	//SendTimeOut
	SOCKET_TIMEOUT_RECEIVE,	//RecvTimeOut
	false					//bInited
};

//-----------------------------------------------------------------
//Save error code and source
//-----------------------------------------------------------------
void Client_SaveSocketError(int socketerrorcode,int errorsource)
{

	EnterCriticalSection(&oClientDescription.oLastError_Protect);
	
	//Fill the structure
	oClientDescription.oLastError.ErrorCode=socketerrorcode;
	oClientDescription.oLastError.ErrorSource=errorsource;

	LeaveCriticalSection(&oClientDescription.oLastError_Protect);

}

//----------------------------------------------------------------------------------------
//Register server
//----------------------------------------------------------------------------------------
int ClientC_Open(char *ServerAddress)
{
	int iCounter,iTargCounter;
	char strProtocol[]="tcp://";	//protocol header	

	char strHostname[256];			//string for storing hostname
	char strAppname[256];			//string for storing appname
	char strAnsAppname[256];		//string for storing answered appname
	int iPort;						//port holder
	char strPort[16];				//string for storing port

	//clear string for receiving data
	memset(strHostname,0,256);
	memset(strAppname,0,256);
	memset(strPort,0,16);

	//Is critical section initialized?
	if (!oClientDescription.bInited)
	{
		InitializeCriticalSection(&oClientDescription.oLastError_Protect);
		oClientDescription.bInited = true;
	}

	//Analyse ServerAddress string

	//validate header
	for (iCounter=0;iCounter<(int)strlen(strProtocol);iCounter++)
	{
		if (strProtocol[iCounter]!=ServerAddress[iCounter])
		{
			Client_SaveSocketError(INVALID_CONNECT_STRING_ERROR,CLIENTC_OPEN_FUNCTION);
			return(0);
		}
	}
	
	iTargCounter=0;

	//extract server name
	while (true)
	{
		//if unexpected end of string
		if (ServerAddress[iCounter]==0)
		{
			Client_SaveSocketError(INVALID_CONNECT_STRING_ERROR,CLIENTC_OPEN_FUNCTION);
			return(0);		
		}

		//if end of host name
		if (ServerAddress[iCounter]==':')
		{
			iCounter++;
			break;
		}
		
		strHostname[iTargCounter]=ServerAddress[iCounter];
		iCounter++;
		iTargCounter++;

	}

	iTargCounter=0;

	//extract port number
	while (true)
	{
		//if unexpected end of string
		if (ServerAddress[iCounter]==0)
		{
			Client_SaveSocketError(INVALID_CONNECT_STRING_ERROR,CLIENTC_OPEN_FUNCTION);
			return(0);
		}

		//if end of port name
		if (ServerAddress[iCounter]=='/')
		{
			iCounter++;
			iPort=atoi(strPort);
			break;
		}
		
		strPort[iTargCounter]=ServerAddress[iCounter];
		iCounter++;
		iTargCounter++;

	}

	iTargCounter=0;

	//extract app name
	while (true)
	{
		//if end of string
		if (ServerAddress[iCounter]==0)
		{
			break;
		}
		
		strAppname[iTargCounter]=ServerAddress[iCounter];
		iCounter++;
		iTargCounter++;		
	}

	//verify existing records in the table
	for (iCounter=1/*it's not a mistake*/;iCounter<SERVER_STORAGE_SIZE;iCounter++)
	{
		if (oClientDescription.ServerInfo[iCounter].iPort==iPort)
		{//slot is used

			if ((strcmp(oClientDescription.ServerInfo[iCounter].strHostname,strHostname)==0) &&
				(strcmp(oClientDescription.ServerInfo[iCounter].strAppname,strAppname)==0))
			{//connection exists
				//let's return it's number
				return(iCounter);
			}
		}
		
	}

	//needed to request server appname
	char fourzero[4];
	memset(fourzero,0,4);


#ifdef _WIN32	
	WSADATA WSAData;                    // Contains details of the Winsock
	// Initialize Winsock.
	if (WSAStartup (MAKEWORD(1,1), &WSAData) != 0) 
	{
		Client_SaveSocketError(GetLastSocketError(),WSASTARTUP_FUNCTION);
		return(-1);
	}
#endif //_WIN32


	if (Client_TalkToServer(fourzero,sizeof(fourzero),strAnsAppname,256,
							strHostname,iPort)!=0)
	{//NO answer from server received
		return(0);
	}

	if (strcmp(strAppname,strAnsAppname)!=0)
	{//everything is bad, appnames are different
		Client_SaveSocketError(INVALID_APPLICATION_NAME_ERROR,CLIENTC_OPEN_FUNCTION);
		return(0);
	}

	//register server
	for(iCounter=1;iCounter<SERVER_STORAGE_SIZE;iCounter++)
	{//search for free slot

		if(oClientDescription.ServerInfo[iCounter].iPort==0)
		{//it's a free server slot!!!
			oClientDescription.ServerInfo[iCounter].iPort=iPort;
			strcpy(oClientDescription.ServerInfo[iCounter].strAppname,strAppname);
			strcpy(oClientDescription.ServerInfo[iCounter].strHostname,strHostname);
			return(iCounter);
		}
		
	}

	//we did not find free slot
	Client_SaveSocketError(NO_FREE_SLOT_ERROR,CLIENTC_OPEN_FUNCTION);
	return(0);

}

//-------------------------------------------------------------------------------
//Send and receive data to/from server
//-------------------------------------------------------------------------------
// Nigmatullin+
int Client_TalkToServer(void *Data, int DataSize, char *out, int outlen,char *server, int port)
//int Client_TalkToServer(char *in,int inlen,char *out,int outlen,char *server,int port)
// Nigmatullin-
{

	SOCKET ClientSock;				//Socket for communication
	SOCKADDR_IN destination_sin;	// Destination socket address
	PHOSTENT phostent;		// Points to the HOSTENT structure
	int iReturn;



	ClientSock=socket(AF_INET, SOCK_STREAM, 0);
	if (ClientSock==INVALID_SOCKET)
	{
		Client_SaveSocketError(GetLastSocketError(),SOCKET_FUNCTION);
		return(-1);
	}

	// Fill out the local socket's address information.
	destination_sin.sin_family = AF_INET;

  	// Retrieve the host information corresponding to the host name.
	if ((phostent = gethostbyname (server)) == NULL)
	{
		Client_SaveSocketError(GetLastSocketError(),GETHOSTBYNAME_FUNCTION);
    	closesocket (ClientSock);
		return(-1);
	}

	// Assign the socket IP address.
	memcpy ((char *)&(destination_sin.sin_addr), phostent->h_addr, phostent->h_length);

	// Convert to network ordering.
	destination_sin.sin_port = htons (port);

	// Establish a connection to the server socket.
	if (connect (ClientSock, (PSOCKADDR) &destination_sin, sizeof (destination_sin)) == SOCKET_ERROR)
	{
		Client_SaveSocketError(GetLastSocketError(),CONNECT_FUNCTION);
    	closesocket (ClientSock);
    	return(-1);
	}

	//Set timeout for recv()
	if (SetRecvTO(ClientSock,oClientDescription.RecvTimeOut) == SOCKET_ERROR)
	{
		Client_SaveSocketError(GetLastSocketError(),SETRECVTO_FUNCTION);
		closesocket (ClientSock);
		return(-1);
	}

	//Set timeout for send()
	if (SetSendTO(ClientSock,oClientDescription.SendTimeOut) == SOCKET_ERROR)
	{
		Client_SaveSocketError(GetLastSocketError(),SETSENDTO_FUNCTION);
		closesocket (ClientSock);
		return(-1);
	}

	// Send a string to the server.
	if (send (ClientSock, (char *)Data, DataSize, 0) == SOCKET_ERROR)
	{
		Client_SaveSocketError(GetLastSocketError(),SEND_FUNCTION);
		closesocket (ClientSock);
		return(-1);
	}

    iReturn = recv (ClientSock, out, outlen, 0);

	// Check if there is any data received. If there is, display it.
    if (iReturn == SOCKET_ERROR)
    {
		Client_SaveSocketError(GetLastSocketError(),RECV_FUNCTION);
		closesocket (ClientSock);
		return(-1);
	}

	// Disable sending and receiving on ClientSock.
	shutdown (ClientSock, 0x02);

	// Close the socket.
	closesocket (ClientSock);

//#ifdef _WIN32	
//	WSACleanup ();
//#endif //_WIN32
	
	return(0);
}

//----------------------------------------------------------------------------------------
//Get description of last error
//----------------------------------------------------------------------------------------
char *ClientC_GetLastError()
{
	char *ErrorDescr;

	//Get and format error message
	ErrorDescr=FormatErrorString(&oClientDescription.oLastError);

	return(ErrorDescr);
	
}

//----------------------------------------------------------------------------------------
//Exchange data with server
//----------------------------------------------------------------------------------------
int ClientC_SendMess(int handle,char *in, char *out)
{
	if ((handle>0) && (handle<SERVER_STORAGE_SIZE) && (oClientDescription.ServerInfo[handle].iPort!=0))
	{//handle is valid
		if (Client_TalkToServer(in,(int)strlen(in),out,CLSRVDEF_COMMUNICATION_DATA_SIZE,
						oClientDescription.ServerInfo[handle].strHostname,
						oClientDescription.ServerInfo[handle].iPort)!=0)					
		{//NO answer from server received
			return(1);
		}
		
		//Ok
		return(0);
	}
	else
	{//handle is invalid
		Client_SaveSocketError(INVALID_HANDLE_ERROR,CLIENTC_SENDMESS_FUNCTON);
		return(1);
	}
}

// Nigmatullin+
//----------------------------------------------------------------------------------------
// Send huge size data to server
//----------------------------------------------------------------------------------------
int ClientC_SendData(int handle, void *Data, int DataSize, char *Out)
{
 if ((handle>0) && (handle < SERVER_STORAGE_SIZE) && (oClientDescription.ServerInfo[handle].iPort != 0))
  {//handle is valid
   if (Client_TalkToServer(Data, DataSize, Out, CLSRVDEF_COMMUNICATION_DATA_SIZE,
                           oClientDescription.ServerInfo[handle].strHostname,
                           oClientDescription.ServerInfo[handle].iPort) != 0)
    {//NO answer from server received
     return(1);
    }

   //Ok
   return(0);
  }
 else
  {//handle is invalid
   Client_SaveSocketError(INVALID_HANDLE_ERROR,CLIENTC_SENDMESS_FUNCTON);
   return(1);
  }
}
// Nigmatullin-

//----------------------------------------------------------------------------------------
//Clear one selected server's slot
//----------------------------------------------------------------------------------------
int ClientC_CloseHandle(int handle)
{
	if ((handle>0) && (handle<SERVER_STORAGE_SIZE) && (oClientDescription.ServerInfo[handle].iPort!=0))
	{//Handle is valid

		memset(&oClientDescription.ServerInfo[handle],0,sizeof(oClientDescription.ServerInfo[handle]));
		return(handle);
	}
	else
	{//invalid handle
		Client_SaveSocketError(INVALID_HANDLE_ERROR,CLIENTC_CLOSEHANDLE_FUNCTON);
		return(0);
	}
}

//----------------------------------------------------------------------------------------
//Clear all server's slots
//----------------------------------------------------------------------------------------
int ClientC_Reset()
{
	//Is critical section initialized?
	if (oClientDescription.bInited)
	{
		DeleteCriticalSection(&oClientDescription.oLastError_Protect);
	}

	memset(&oClientDescription,0,sizeof(oClientDescription));

	//All ok
	return(1);
}

//----------------------------------------------------------------------------------------
//Needed for compatibility
//----------------------------------------------------------------------------------------
int ClientC_SetNoRegisterChannelFlag(int flag)
{//Needed for compatibility
	return(0);
}

//----------------------------------------------------------------------------------------
//Set send timeout
//----------------------------------------------------------------------------------------
void ClientC_SetSendTimeout(int millisec)
{
	oClientDescription.SendTimeOut=millisec;
}

//----------------------------------------------------------------------------------------
//Set receive timeout
//----------------------------------------------------------------------------------------
void ClientC_SetRecvTimeout(int millisec)
{
	oClientDescription.RecvTimeOut=millisec;
}
