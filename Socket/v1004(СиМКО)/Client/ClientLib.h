#ifndef _CLIENTLIB__H_
#define _CLIENTLIB__H_

#include "IPLSocket.h"
#include "ClSrvDef.h"
#include "ClientC.h"

//count of server slots
#define SERVER_STORAGE_SIZE	64

//server slot description
typedef struct
{
	char strHostname[256];		//Storage for server name
	int iPort;					//port holder
	char strAppname[256];		//string for storing app name
} Server_Info_t, *pServer_Info_t;

//Client description
typedef struct 
{
	unsigned		SendTimeOut;	//Timeout in ms for send()
	unsigned		RecvTimeOut;	//Timeout in ms for recv()

	bool				bInited;	//Флаг, инициализирована ли критическая секция

	CRITICAL_SECTION	oLastError_Protect;
	ErrorStorage_t		oLastError;		//storage for last error
	Server_Info_t		ServerInfo[SERVER_STORAGE_SIZE];	//array of server's descriptions
} Client_Ion_t, *pClient_Ion_t;

//Send something to server and wait for responce
// Nigmatullin+
//int Client_TalkToServer(char *in,int inlen,char *out,int outlen,char *server,int port);
int Client_TalkToServer(void *Data, int DataSize, char *out, int outlen,char *server, int port);
// Nigmatullin-

//Push error code and source into the buffer
void Client_SaveSocketError(int socketerrorcode,int errorsource);

#endif //_CLIENTLIB__H_
