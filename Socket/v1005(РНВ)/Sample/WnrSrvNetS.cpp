//=============================================================================
// ������:     ������������� ������� ������� (���)
// ������:     WnrSrv - ������ ���������� �������������� ������� � ���������� �����������
// ����:       WnrSrvNetS.cpp
// ����������: �����-�������������� � �������� ������������
//-----------------------------------------------------------------------------
// Copyright (c) 2012-2015, ����������� �.�.
//=============================================================================

#include <Windows.h>

#include <Direct.h>
#include <Io.h>
#include <Time.h>

#include "WnrSrv.h"
#include "WnrSrvI.h"
#include "Resource.h"

//============================================================================
// �������� ������� ��� ������ � ��������� ������
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
// ����� � ��������� ������� ������
//-----------------------------------------------------------------------------
// ������ In (�������� ������):
// - [int] �������
// - ����� � ����������� �� ������� ����� ��������� �� ���������
// ������ ������������� Out:
// - [int] ��� ��������
// - ����� � ����������� �� ������� ����� ��������� �� ���������
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

 int  UnitCmd  = ((int *)In)[0];    // ������� - ��� ������ �������� ������� ������, ��������� - � ����������� �� �������
 char *pParam  = In  + sizeof(int); // ������� ��������� (������ int - ��� �������)
 char *pResult = Out + sizeof(int); // ��������� (������ int - ��� ��� �������� Result)

 // ������ ������������ ������ �� ��������� ������������� � 0
 if (pOutSize) *pOutSize = 0;

 DbgPrint(0, "> Receiving command line <%d>", UnitCmd);

 if (UnitCmd != WNRSRV_CMD_GETDATA)
   WnrSrv->Log->Write("NetS_CB: ������� %d", UnitCmd);

 switch (UnitCmd)
  {
        // �������� �����
   case WNRSRV_CMD_GETSTATUS:
        *pOutSize = WnrSrv->StatusGetArray((int *)pResult);
        Result = SRV_RES_OK;
        break;

        // ������� ����������� �������
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

        // ���������� ������ �������� �����������
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

        // ����� ���� �������
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

        // ������\������� �����������
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

        // ��������� ������ (����������� + ���������� �������)
   case WNRSRV_CMD_GETDATA:
        if (WnrSrv->pDS)
         {
          //!!! ��������� ������������� CLSRVDEF_COMMUNICATION_DATA_SIZE 
          // ��������� �������:
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

 // ����� ���������� �� ���������� �������
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

 // ��������� ������ CallBack 
 if (Out) *(int *)Out = Result;
 *pOutSize += sizeof(int);

 WnrSrv->fCProcMode = FALSE;

 return (Result == SRV_RES_OK ? SRV_RES_OK : SRV_RES_FAULT); // ��� �������� == ������ ��� �����-������ ������ -1
}

//============================================================================
// ����� ������ ������� NetS_CB
//============================================================================
DWORD WINAPI NetS_CBThreadCall(void *pArg)
{
 CWnrSrv *WnrSrv = (CWnrSrv *)pArg;
 
 char  OutLine[8]; // ������ ��� ������� ������
 ULONG OutSize = 0;
 NetS_CB(WnrSrv->CmdLine, OutLine, &OutSize, WnrSrv);

 return (0);
}

//============================================================================
// ��������� ���������� ������� � �������
//============================================================================
void CWnrSrv::NetS_DbgCall(char *CmdLine)
{
 strcpy(this->CmdLine, CmdLine);
 // ����� �������� ������ �� DbgPrint ��� ������������ ���������
 // ������� ����� ��� ������ NetS_CB, � �� �������� �������� NetServer_CB
 CreateThread(NULL, 0, NetS_CBThreadCall, this, 0, NULL);
}
