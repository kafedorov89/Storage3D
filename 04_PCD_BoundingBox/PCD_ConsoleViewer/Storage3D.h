#pragma once
#include <string>
#include <ctime>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace std;
using namespace pcl;

namespace Storage3D{
	//#######################################################################################################
	//����� ����������� �������� ������ � ����� ���������������
	class Stored3Dobject
	{
	public: Stored3Dobject(int layerID, int xDeltaRes, int yDeltaRes); //����������� ������ Stored3Dobject

			//-----------------------------------------------------------------------------------------------
			//���� ��������� �������

			//��������� ��������� ������
	public: bool removed; //���� ���������� �������

			//��������� ����� ������� � ������� ���������� ������
	public: int UID; //���������� �������� ������������� �������
	public: int StorageUID; //���������� ������������� ������ �� ������� �������� ������
	public: int addedLayerID; //������������� ���� �� ������� ��� �������� ������
	public: int removedLayerID; //������������� ���� �� ������� ������ ��� ������

			//����������� �������������� �������
	public: string ObjectName; //�� ���������� ��������� ����������� �������
	public: string ObjectTypeName; //�������� ���� ������� 
	public: int ObjectType; //������������� ���� �������
	public: time_t AddedData; //���� ���������� ������� �� �����
	public: time_t RemovedData; //���� �������� ������� �� ������

			//-----------------------------------------------------------------------------------------------
			//�������������� ��������� �������
	public: pcl::PointXYZ startPoint; //���������� ������ ���� ������� (����������� ��������)
	public: pcl::PointXYZ center; //���������� ������ ���� ������� (����������� ��������)
	public: float volume; //����� ������� (����������� ��������)
	public: Eigen::Matrix3f orientation; //���������� ������� ������������ ������� ��������� ������ (����������� ��������)
	
			//���������� �������
	public: float length; //����� ������� (����������� ��������)
	public: float weidth; //������ ������� (����������� ��������)
	public: float height; //������ ������� (����������� ��������)
			
			//��������� ����������� � ����� ������� �����
	public: pcl::PointCloud<pcl::PointXYZ> objectDelta; //������ ����� ����������� � �������

	public: array<Point3D^, 2>^ objectUpPoints; //������ ���� <Point3D^> �������� �������� ���������� ����� ����������� � ������� ��������� � �������� ������� objectDelta
	public: int XDeltaRes; //������ ����� ������� �� oX
	public: int YDeltaRes; //������ ����� ������� �� oY


	public: ArrayList^ objectPointCloud; //������ ���� <Point3D^> �������� �������� ���������� ���� ����� �������. ����������� � ������� ������� CreateObjectPointCloud()
			

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
	class StorageLayer
	{
	public: StorageLayer(); //����������� ������ StorageLayer

			//-----------------------------------------------------------------------------------------------
			//���� ����
	public: int UID; //���������� �������� ������������� ����
	public: time_t AddedData;
	
	public: pcl::PointCloud<pcl::PointXYZ> layerNegativeDelta; //������ ���� <float> ������������� ������ ����� ����� �� ������ ���� �� ��������� � �����������
	public: pcl::PointCloud<pcl::PointXYZ> layerPositiveDelta; //������ ���� <float> ������������� ������ ����� ����� �� ������ ���� �� ��������� � ����������� 
	public: pcl::PointCloud<pcl::PointXYZ> realDepthMap; //����� ������� (� �������� �����������)
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
	class Storage
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
