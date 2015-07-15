#ifndef _ClientC__H_
#define _ClientC__H_

//#define _WIN32 //Target OS (MS Windows or not)
#include "ClSrvDef.h"

#ifndef _WIN32 //Target OS (MS Windows or not)
#define DLLAPI
#endif

#ifndef DLLAPI
#ifdef CLIENT_EXPORTS
#define DLLAPI __declspec(dllexport)	//Comment this when use library
#else	//CLIENT_EXPORTS
#define DLLAPI __declspec(dllimport)	//Uncomment this when use library
#endif //CLIENT_EXPORTS
#endif //DLLAPI

#ifdef __cplusplus
extern "C" 
{
#endif //__cplusplus

DLLAPI int ClientC_Open(char *ServerAddress);

DLLAPI int ClientC_SendMess(int handle,char *in, char *out);

// Nigmatullin+ Send huge size data to server
DLLAPI int ClientC_SendData(int handle, void *Data, int DataSize, char *Out);
// Nigmatullin-

DLLAPI char *ClientC_GetLastError();

DLLAPI int ClientC_CloseHandle(int handle);

DLLAPI int ClientC_Reset();

DLLAPI int ClientC_SetNoRegisterChannelFlag(int flag);

DLLAPI void ClientC_SetSendTimeout(int millisec);

DLLAPI void ClientC_SetRecvTimeout(int millisec);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif //_ClientC__H_
