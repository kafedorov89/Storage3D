#include "MainForm.h"
using namespace System;
using namespace System::Windows::Forms;
using namespace Storage3Dform;


[STAThread]



int main(array<System::String^> ^args)
{
	Application::EnableVisualStyles();
	Application::SetCompatibleTextRenderingDefault(false);
	Storage3Dform::MainForm form;
	Application::Run(%form);
}



//������� ������������� ���� ������� � �������
StorageLayer^ MainForm::initCurrentLayer(){
				//�������� ����������� � ����� ������� � ��������� ������
				RGBDepthThread = gcnew Thread(gcnew ParameterizedThreadStart(&ThreadProcRGBDepth));
				RGBDepthThread->Start(this);
				//RGBDepthThread->Join();

				//���������� RGB-����������� � ����� ������� � �������� ����������� � ������� ����
				int xRGBRes = scaner3D->cols;
				int yRGBRes = scaner3D->raws;
				int xDepthRes = scaner3D->cols;
				int yDepthRes = scaner3D->raws;

				StorageLayer^ newLayer = gcnew StorageLayer(scaner3D->getRealPoints3DArray(), xDepthRes, yDepthRes, scaner3D->rgbBitmap, xRGBRes, yRGBRes); //�������������� ����� ����

				return newLayer;
	}

			//������� ��� ������ � ������� �������
			//��������������� ��������� ����������� ������� ������ ����������� ����������� � ����� ������� 
void MainForm::ThreadProcRGBDepth(System::Object ^obj)
	{
				 SafeThreadRGBDepth(obj);
	}
			 //������� ������ ��� ��������� RGB-����������� � ����� ������� � �������
void MainForm::SafeThreadRGBDepth(System::Object ^obj)
	{
				 MainForm ^ob = (MainForm^)obj;

				 //while (true){
				 //Thread::Sleep(0);
				 ob->pictureBox_RGB->Image = ob->scaner3D->getRGBImage();
				 ob->pictureBox_Depth->Image = ob->scaner3D->getDepthImage();
				 //}

				 ob->RGBDepthThread->Abort();
	}