#ifndef _CSERVER__H_
#define _CSERVER__H_

//#define _WIN32 //Target OS (MS Windows or not)
#include "ClSrvDef.h"

#ifndef _WIN32 //Target OS (MS Windows or not)
#define DLLAPI
#endif

#ifndef DLLAPI
#ifdef SERVER_EXPORTS
#define DLLAPI __declspec(dllexport)	//Comment this when use library
#else	//SERVER_EXPORTS
#define DLLAPI __declspec(dllimport)	//Uncomment this when use library
#endif //SERVER_EXPORTS
#endif //DLLAPI

#ifdef __cplusplus
extern "C"
{
#endif //__cplusplus

	//Standard CallBack
#ifndef _IS_typedef_BarsCallBack
	typedef int BarsCallBack(char *in, char *out, PULONG pOutSize);
#define _IS_typedef_BarsCallBack
#endif //_IS_typedef_BarsCallBack

	//Extended CallBack
#ifndef _IS_typedef_BarsCallBackEx
	typedef int BarsCallBackEx(char *in, char *out, PULONG pOutSize, void *pParam);
#define _IS_typedef_BarsCallBackEx
#endif //_IS_typedef_BarsCallBackEx

	//Standard Error CallBack
#ifndef _IS_typedef_BarsErrorCallBack
	typedef int BarsErrorCallBack(void *pParam);
#define _IS_typedef_BarsErrorCallBack
#endif //_IS_typedef_BarsErrorCallBack

	//Callback for any non error event
#ifndef _IS_typedef_BarsStatusCallBack
	typedef int BarsStatusCallBack(void *pParam);
#define _IS_typedef_BarsStatusCallBack
#endif //_IS_typedef_BarsStatusCallBack

	//Set callback for serving client's threads
	DLLAPI int Server_SetCallBack( BarsCallBack *C );

	//Startup server
	DLLAPI int Server_SetServer(int inPort,char *Name);

 // Close Socket-server
 DLLAPI int Server_Close();

	//Retrieve description for last server error
	DLLAPI char *Server_GetLastServerError(void);

	//Retrieve description for last server status event
	DLLAPI char *Server_GetLastServerStatusEvent(void);

	//Needed for community
	DLLAPI int Server_SetNoRegisterChannelFlag(int flag);

	//Set extended callback for serving client's threads
	DLLAPI int Server_SetCallBackEx( BarsCallBackEx *C, void *param );

	//Set error callback
	DLLAPI int Server_SetErrorCallBack( BarsErrorCallBack *C, void *param );

	//Set status event callback
	DLLAPI int Server_SetStatusCallBack( BarsStatusCallBack *C, void *param );

	DLLAPI void Server_SetSendTimeout(int millisec);

	DLLAPI void Server_SetRecvTimeout(int millisec);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif //_CSERVER__H_
