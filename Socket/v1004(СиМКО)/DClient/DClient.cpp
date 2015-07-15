//=============================================================================
// Проект:     Система мониторинга клиентского обслуживания (СиМКО)
// Модуль:     DClient - сокет-клиент для выполнения консольных команд
// Файл:       DClient.cpp
// Назначение: главный файл модуля
//-----------------------------------------------------------------------------
// Copyright (c) 2013, Нигматуллин Ф.Т.
//=============================================================================

#include <Windows.h>
#include <StdIo.h>
//#include <StdLib.h>

#include "ClientC.h"
#include "DSocketI.h"

//=============================================================================
// WinMain
//=============================================================================
int main(int argc, char* argv[])
{
 if (argc > 2)
  {
   char DServer[128];
   strcpy(DServer, argv[1]);

   char CommandStr[1024] = "";
   for (int i = 2; i < argc; i++)
    {
     strcat(CommandStr, argv[i]);
     strcat(CommandStr, " ");
    }

   char ConnectionStr[120];
   wsprintf(ConnectionStr, "tcp://%s:%d/%s", DServer, DSOCKET_CS_DSOCKET, DSOCKET_CS_DSERVER);

   int hLSock = ClientC_Open(ConnectionStr);
   if (hLSock)
    {
     char ReturnStr[1024];
     ClientC_SendMess(hLSock, CommandStr, ReturnStr);

     ClientC_CloseHandle(hLSock);
    }
  }

 return (0);
}
