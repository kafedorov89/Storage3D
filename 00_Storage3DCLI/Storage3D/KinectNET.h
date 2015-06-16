#pragma once

using namespace System;
using namespace System::ComponentModel;
using namespace System::Collections;
using namespace System::Windows::Forms;
using namespace System::Data;
using namespace System::Drawing;
using namespace System::Collections::Generic;

using namespace System::Drawing::Imaging;
using namespace System::Threading;

using namespace OpenTK;
using namespace OpenNI;


namespace KinectNET{
	ref class Kinect
	{

	public: Kinect();

			//��������� ��� ������ � ��������
	public: ImageGenerator^ imageGen; //�������� RGB-������
	public: DepthGenerator^ depthGen; //�������� Depth-������

	public: ImageMetaData^ imageMD; //���������� ��� �������� ���������� ����������� RGB-�����������
	public: DepthMetaData^ depthMD; //���������� ��� �������� ���������� ����������� Depth-�����������

			//�������� ���������
	public: System::String^ configpath; //���� � ����� ������������ ��� OpenNI ����������
	public: Context^ context; //�������� ������ ��� ������ � �������� ����� ���������� OpenNI
	public: ScriptNode^ scriptNode; //�� ��� � ��� ����� ���� ��� �����, �� ������� ������

	public: int cols; //������ ����������� (X)
	public: int raws; //������ ����������� (Y)

			//��������� ��� ��������� RGB-�����������
	public: MapOutputMode mapModeRGB;
	public: Bitmap^ rgbBitmap; //RGB-�����������, ������� ��� ���������� � ����������� �� ������

			//��������� ��� ��������� Depth-�����������
	public: MapOutputMode mapModeDepth;
	public: Bitmap^ depthBitmap; //Depth-�����������, ������� ��� ���������� � ����������� �� ������
	public: int MaxDepth;

	public: void initKinect(); //������� ������������� ������� 
	public: void findLocalMaxDepth(); //������� ������ ���������� ��������� ����� ������� (��� ��������� ���������� ������������� �����������)
	public: Bitmap^ getRGBImage(); //������� ��������� ���������� RGB ����������� � �������
	public: Bitmap^ getDepthImage(); //������� ��������� ���������� Depth ����������� � �������
	public: Point3D^ ScreenToWorldPoint(int screenX, int screenY, int depth); //������� �������� ��������� � RGB ����������� � ���������� ��������� 3D ������������ � ��
	public: array<Point3D^,2>^ getRealPoints3DArray(); //������� ��������� ����� ������� � �������� ����������� ������������
	};
}