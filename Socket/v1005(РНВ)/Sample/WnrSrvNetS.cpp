//=============================================================================
// Проект:     Распознавание номеров вагонов (РНВ)
// Модуль:     WnrSrv - сервер управления распознаванием образов и обработкой изображений
// Файл:       WnrSrvNetS.cpp
// Назначение: Сокет-взаимодействие с внешними приложениями
//-----------------------------------------------------------------------------
// Copyright (c) 2012-2015, Нигматуллин Ф.Т.
//=============================================================================

#include <Windows.h>

#include <Direct.h>
#include <Io.h>
#include <Time.h>

#include "WnrSrv.h"
#include "WnrSrvI.h"
#include "Resource.h"

//============================================================================
// Создание сервера для приема и обработки команд
//============================================================================
BOOL CWnrSrv::NetS_Create()
{
 BOOL Result = TRUE;

 if (!(Ini.App.DebugFlag & DBGF_DISABLESERVER))
  {
   int fSetServer = Server_SetCallBackEx(NetS_CB, this);

   DbgPrint(0, "> Setting unit callback result: %d", fSetServer);

   Server_SetNoRegisterChannelFlag(1);
   if (Server_SetServer(ModuleSocket, ModuleNick) == 0)
    {
     DbgPrint(0, "> Setting unit as a server: Ok");
    }
   else
    {
     Result = FALSE;
     char *Error = Server_GetLastServerError();
     DbgPrint(0, "> Setting unit as a server: %s\n", Error);
     Log->Write("NetS_Create: [%d %s] <%s>", ModuleSocket, ModuleNick, Error);
    }
  }

 return (Result);
}

//=============================================================================
// Прием и обработка внешних команд
//-----------------------------------------------------------------------------
// Формат In (бинарные данные):
// - [int] Команда
// - Далее в зависимости от команды могут следовать ее параметры
// Формат возвращаемого Out:
// - [int] Код возврата
// - Далее в зависимости от команды могут следовать ее параметры
//=============================================================================
int NetS_CB(char *In, char *Out, PULONG pOutSize, void *LocalObject)
{
 int Result = SRV_RES_FAULT;

 CWnrSrv *WnrSrv = (CWnrSrv *)LocalObject;
 if (!WnrSrv)
  {
   if (Out) itoa(Result, Out, 10);
   if (pOutSize) *pOutSize = sizeof(int);
   return (SRV_RES_FAULT);
  }

 WnrSrv->fCProcMode = TRUE;

 int  UnitCmd  = ((int *)In)[0];    // Команда - это первый аргумент входных данных, остальные - в зависимости от команды
 char *pParam  = In  + sizeof(int); // Входные аргументы (первый int - это команда)
 char *pResult = Out + sizeof(int); // Результат (первый int - это код возврата Result)

 // Размер возвращаемых данных по умолчанию устанавливаем в 0
 if (pOutSize) *pOutSize = 0;

 DbgPrint(0, "> Receiving command line <%d>", UnitCmd);

 if (UnitCmd != WNRSRV_CMD_GETDATA)
   WnrSrv->Log->Write("NetS_CB: команда %d", UnitCmd);

 switch (UnitCmd)
  {
        // Проверка связи
   case WNRSRV_CMD_GETSTATUS:
        *pOutSize = WnrSrv->StatusGetArray((int *)pResult);
        Result = SRV_RES_OK;
        break;

        // Очистка результатов расчета
   case WNRSRV_CMD_RESET:
        if (WnrSrv->Rec) 
         {
          BOOL fFullReset = FALSE;
          memcpy(&fFullReset, pParam, sizeof(int));
          pParam += sizeof(int);

          BOOL fTimeReset = FALSE;
          if (!fFullReset) fTimeReset = TRUE;

          WnrSrv->Rec->CalcReset(fFullReset, fTimeReset);
          Result = SRV_RES_OK;

          WnrSrv->StatusLogWrite();
         }
        break;

        // Произвести расчет текущего изображения
   case WNRSRV_CMD_CALC:
        if (WnrSrv->VCap) 
         {
          if (WnrSrv->Rec_Calc(pParam))
            Result = SRV_RES_OK;
          else
            Result = SRV_RES_EXECERROR;
         }
        else
          Result = SRV_RES_EXECERROR;
        break;

        // Смена фазы анализа
   case WNRSRV_CMD_CALCSWITCH:
        if (WnrSrv->Rec) 
         {
          BOOL AnlzSwitchedOn = FALSE;
          memcpy(&AnlzSwitchedOn, pParam, sizeof(int));
          pParam += sizeof(int);

          BOOL AnlzOnlyNumbersOn = FALSE;
          memcpy(&AnlzOnlyNumbersOn, pParam, sizeof(int));
          pParam += sizeof(int);

          if (WnrSrv->Rec_Switch(AnlzSwitchedOn, AnlzOnlyNumbersOn))
            Result = SRV_RES_OK;
          else
            Result = SRV_RES_EXECERROR;

          WnrSrv->StatusLogWrite();
         }
        break;

        // Запуск\останов видеопотока
   case WNRSRV_CMD_VCAPSWITCH:
        if (WnrSrv->VCap) 
         {
          int RunMode_New = VCAP_VIDEOTHREAD_STOPPED;
          memcpy(&RunMode_New, pParam, sizeof(int));
          pParam += sizeof(int);

          int RunMode = WnrSrv->VCap->GetRunMode();
          if (RunMode_New != RunMode)
           {
            WnrSrv->VCap->SetRunMode(RunMode_New);
           }

          Result = SRV_RES_OK;

          WnrSrv->StatusLogWrite();
         }
        break;

        // Получение данных (изображение + результаты расчета)
   case WNRSRV_CMD_GETDATA:
        if (WnrSrv->pDS)
         {
          //!!! Проверить достаточность CLSRVDEF_COMMUNICATION_DATA_SIZE 
          // Параметры команды:
          COPYDATA_PRM CopyDataParam;
          memcpy(&CopyDataParam, pParam, sizeof(COPYDATA_PRM));
          pParam += sizeof(COPYDATA_PRM);

          Result = WnrSrv->Rec_CopyData(&CopyDataParam, pResult, pOutSize);
         }
        break;

   default:
        Result = SRV_RES_NOTSUPPORTED;
        break;
  }

 // Вывод результата на отладочную консоль
 switch (Result)
  {
   case SRV_RES_OK:           DbgPrint(0, "Ok");
        break;
   case SRV_RES_EXECERROR:    DbgPrint(0, "Execution error");
        break;
   case SRV_RES_NOTSUPPORTED: DbgPrint(0, "Command not supported");
        break;
   case SRV_RES_INVALIDARG:   DbgPrint(0, "Invalid arguments");
        break;
   case SRV_RES_BUSY2STACK:   DbgPrint(0, "Busy, set command to stack");
        break;
   default:
        break;
  }

 // Результат работы CallBack 
 if (Out) *(int *)Out = Result;
 *pOutSize += sizeof(int);

 WnrSrv->fCProcMode = FALSE;

 return (Result == SRV_RES_OK ? SRV_RES_OK : SRV_RES_FAULT); // Код возврата == ошибка для сокет-метода только -1
}

//============================================================================
// Поток вызова функции NetS_CB
//============================================================================
DWORD WINAPI NetS_CBThreadCall(void *pArg)
{
 CWnrSrv *WnrSrv = (CWnrSrv *)pArg;
 
 char  OutLine[8]; // Только для простых команд
 ULONG OutSize = 0;
 NetS_CB(WnrSrv->CmdLine, OutLine, &OutSize, WnrSrv);

 return (0);
}

//============================================================================
// Обработка отладочной команды в консоли
//============================================================================
void CWnrSrv::NetS_DbgCall(char *CmdLine)
{
 strcpy(this->CmdLine, CmdLine);
 // Чтобы избежать клинча по DbgPrint при параллельных процессах
 // создаем поток для вызова NetS_CB, а не напрямую вызываем NetServer_CB
 CreateThread(NULL, 0, NetS_CBThreadCall, this, 0, NULL);
}
