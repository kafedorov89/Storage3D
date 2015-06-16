#pragma once
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

using namespace System;
using namespace System::ComponentModel;
using namespace System::Collections;
using namespace System::Windows::Forms;
using namespace System::Data;
using namespace System::Drawing;
using namespace System::Collections::Generic;

using namespace OpenNI;
using namespace OpenTK;
using namespace cv;

namespace Storage3D{
	//#######################################################################################################
	//����� ����������� �������� ������ � ����� ���������������
	ref class Stored3Dobject
	{
	public: Stored3Dobject(int layerID, int xDeltaRes, int yDeltaRes); //����������� ������ Stored3Dobject

			//-----------------------------------------------------------------------------------------------
			//���� ��������� �������

			//��������� ��������� ������
	public: bool removed; //���� ���������� �������

			//��������� ����� ������� � ������� ���������� ������
	public: UInt32^ UID; //���������� �������� ������������� �������
	public: UInt32^ StorageUID; //���������� ������������� ������ �� ������� �������� ������
	public: int addedLayerID; //������������� ���� �� ������� ��� �������� ������
	public: int removedLayerID; //������������� ���� �� ������� ������ ��� ������

			//����������� �������������� �������
	public: System::String^ ObjectName; //�� ���������� ��������� ����������� �������
	public: System::String^ ObjectTypeName; //�������� ���� ������� 
	public: int ObjectType; //������������� ���� �������
	public: DateTime^ AddedData; //���� ���������� ������� �� �����
	public: DateTime^ RemovedData; //���� �������� ������� �� ������

			//-----------------------------------------------------------------------------------------------
			//�������������� ��������� �������
	public: Point3D^ center; //���������� ������ ���� ������� (����������� ��������)
	public: float volume; //����� ������� (����������� ��������)
	public: Matrix4^ orientation; //���������� ������� ������������ ������� ��������� ������ (����������� ��������)
	
			//���������� �������
	public: float length; //����� ������� (����������� ��������)
	public: float weidth; //������ ������� (����������� ��������)
	public: float height; //������ ������� (����������� ��������)
			
			//��������� ����������� � ����� ������� �����
	public: ArrayList^ objectDelta; //������ ���� <float> ������ ����� ����� ����� ���������� ������� ������� �� ��������� � ����������� ��������� ������
	public: ArrayList^ objectDelta_map; //������ <System::Drawing::Point^> ����������� �������� ������� 
	//public: ArrayList^ objectUpPoints; //������ ���� <Point3D^> �������� �������� ���������� ����� ����������� � ������� ��������� � �������� ������� objectDelta

	//public: array<float, 2>^ objectDelta; //������ ���� <float> ������ ����� ����� ����� ���������� ������� ������� �� ��������� � ����������� ��������� ������
	public: array<Point3D^, 2>^ objectUpPoints; //������ ���� <Point3D^> �������� �������� ���������� ����� ����������� � ������� ��������� � �������� ������� objectDelta
	public: int XDeltaRes; //������ ����� ������� �� oX
	public: int YDeltaRes; //������ ����� ������� �� oY


	public: ArrayList^ objectPointCloud; //������ ���� <Point3D^> �������� �������� ���������� ���� ����� �������. ����������� � ������� ������� CreateObjectPointCloud()
			
			//�������� ����� ��� ����������� ���������������
	public: Point3D^ Vertex1_UpLeft; //������ ������� ������� �������
	public: System::Drawing::Point^ Vertex1_UpLeft_map; //������ ������� ������� ������� � ������� ����� �������
	public: Point3D^ Vertex2_UpCenter; //�������� ����������� �������
	public: System::Drawing::Point^ Vertex2_UpCenter_map; //�������� ����������� ������� � ������� ����� �������
	public: Point3D^ Vertex3_UpRight; //������ ������ ������� �������
	public: System::Drawing::Point^ Vertex3_UpRight_map; //������ ������ ������� ������� � ������� ����� �������
	public: Point3D^ Vertex4_Up; //��������� ������ �������
	public: System::Drawing::Point^ Vertex4_Up_map; //��������� ������� ������� � ������� ����� �������
	public: Point3D^ Vertex5_DownCenter; //������ ����������� �������
	public: System::Drawing::Point^ Vertex5_DownCenter_map; //������ ����������� ������� � ������� ����� �������

			//���������� ������� ��������
			//public: float height; //������ ������� (����������� ��������)
			//public: float radius; //������ ������� (����������� ��������)
			
			//�������� ����� ��� ����������� ��������
			//public: Point3D^ Vertex1_UpCenter; //������ ����������� �����
			//public: Point3D^ Vertex2_UpRadius; //����� �� ������� ����������
			//public: Point3D^ Vertex3_DownCenter; //������ ����������� �����

			//-----------------------------------------------------------------------------------------------
			//������ ��������� �������

			//������� ������� 4-� �������� ����� ������� (������ ������� ������� ��� ������������ � ����� �������)
	public: void SetKey4PointsParall(Point3D^ vertex1_UpLeft, Point3D^ vertex2_UpCenter, Point3D^ vertex3_UpRight, Point3D^ vertex4_Up);

	public: void CalcDimentions(); //������� ���������� �������� �������� ������� (�����)
	private: float CalcMaxDelta(); //������� ������ ������������� ���������� Delta ������ ����� ���������� ������
	private: void CalcHeight(int mode); //������� ���������� ������ �������
	private: void CalcLengthWeidth(); //������� ���������� ���������� ������ � ����� �������
	private: void CalcVolume(); //������� ���������� ������ �������
	private: void CalcCenter(); //������� ���������� ��������� ������ ���� �������
	private: void CalcOrient(); //������� ���������� ��������� ������� ������������ �������������� ���������
	public: bool isObjectPoint(System::Drawing::Point^ mapPointForCheck); //������� ��� �������� �������������� ����� � ������ �������

	private: void CreateObjectPointCloud(); //������� �������� ����������� ������� �� ��� �������� � ������������ ������� Delta

	public: void RemoveObject(); //������� �������� ������� �� ������
	};





	//#######################################################################################################
	//�����, �������� ���������� �� ����� ���� (������������ ������)
	ref class StorageLayer
	{
	public: StorageLayer(array<Point3D^, 2>^ depthMap, int xDepthRes, int yDepthRes, Bitmap^ rgbMap, int xRGBRes, int yRGBRes); //����������� ������ StorageLayer

			//-----------------------------------------------------------------------------------------------
			//���� ����
	public: UInt32^ UID; //���������� �������� ������������� ����
	public: DateTime^ AddedData;
	
	public: ArrayList^ layerNegativeDelta; //������ ���� <float> ������������� ������ ����� ����� �� ������ ���� �� ��������� � �����������
	public: ArrayList^ layerNegativeDelta_map; //������ ���� <Point> �������� ����������� ����� realDepthMap � layerDelta
	public: ArrayList^ layerPositiveDelta; //������ ���� <float> ������������� ������ ����� ����� �� ������ ���� �� ��������� � ����������� 
	public: ArrayList^ layerPositiveDelta_map; //������ ���� <Point> �������� ����������� ����� realDepthMap � layerDelta
	public: array<Point3D^, 2>^ realDepthMap; //����� ������� (� �������� �����������)
	public: ArrayList^ objectIDList; //������ ���� <int> ��������������� �������� ����������� �� ����
	
	public: int XDepthRes;
	public: int YDepthRes;

	public: Bitmap^ RGBMap; //����������� ���� � ������ ������ ����� �������
	public: int XRGBRes;
	public: int YRGBRes;
			//----------------------------------------------------------------------------------------------
			//������ ����

	};





	//#######################################################################################################
	//�����, ����������� ������ ������ � ���������
	ref class Storage
	{
	public: Storage(); //����������� ������ Storage

			//-----------------------------------------------------------------------------------------------
			//���� ������
	public: UInt32^ UID; //���������� �������� ������������� ������
	public: DateTime^ CreatedData; //���� �������� ������

	public: ArrayList^ LayerList; //������ ���� <StorageLayer^> ����� ����������� �� �����
	public: ArrayList^ ObjectList; //������ ���� <Stored3Dobject^> �������� ����������� �� �����
	public: ArrayList^ ObjectIDForRemoveList; //������ ���� <int> ��������������� �������� ���������� ��� �������� �� ������
	
	public: float Delta3DEpsilon; //������ �� ������� ����� �������� ����� ����� ������� �������� �� layerDelta ��������� �������������
	public: float deltaValidPercent; //������� ����� ����� ������� � ������������� ��������� Delta ��� ������� ������ ����� ������ 

			//----------------------------------------------------------------------------------------------
			//������ ������
	public: void AddNewObject(Stored3Dobject^ newObject); //������� ���������� ������ �������
	private: void CalcNewObjectDelta(Stored3Dobject^ newObject); //������� ���������� Delta � �������� ����������� ��� ������������ �������
	public: void FindObjectForRemove(); //������� ������ �������� ��� �������� ����� ���������� ������ ���� (����������� ��� ������� ������������� �������� Delta
	public: void RemoveObject(int objectID); //������� �������� i-�� ������� �� ������ 
	public: void RemoveFoundObjects(); //������� �������� ������ �������� �� ������(����������� ����� ���������� FindObjectForRemove())

	public: void AddNewLayer(StorageLayer^ newLayer); //������� ���������� ������ ����
	private: void CalcNewLayerDelta(StorageLayer^ newLayer); //������� ���������� Delta � �������� ����������� ��� ������������ ����
	
	};
}
