#pragma once
#include "stdafx.h"

namespace Storage3D{


	class StoredObject
	{
		//-----------------------------------------------------------------------------------------------
		//���� ��������� �������
	public:
		//��������� ��������� ������
		bool removed; //���� ���������� �������

		//��������� ����� ������� � ������� ���������� ������
		int UID; //���������� �������� ������������� �������
		int StorageUID; //���������� ������������� ������ �� ������� �������� ������
		int addedLayerID; //������������� ���� �� ������� ��� �������� ������
		int removedLayerID; //������������� ���� �� ������� ������ ��� ������

		//����������� �������������� �������
		string ObjectName; //�� ���������� ��������� ����������� �������
		string ObjectTypeName; //�������� ���� ������� 
		int ObjectType; //������������� ���� �������
		time_t AddedDate; //����� ���������� ������� �� �����
		time_t RemovedDate; //����� �������� ������� �� ������

		//-----------------------------------------------------------------------------------------------
		//�������������� ��������� �������
		//Validation object
		bool check_valid_object();
		bool isValid;

		pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud; //������ ���� �������� �������� ���������� ���� ����� �������

		int step_degree; //Step in degree for Searching optimal boundingbox
		int max_degree; //Cout degree where searching optimal boundingbox
		float grid_density; //Count of point cloud grid in x, y, z dimentions

		//Moving to position
		Eigen::Vector3f position;
		Eigen::Quaternionf quaternion_to_bbox;
		Eigen::Affine3f jump_to_bbox;

		//Object orientation to set to zero position
		float roll;
		float pitch;
		float yaw;

		//Moving to zero
		Eigen::Affine3f jump_to_zero;

		//Object size
		float width;
		float lenght;
		float height;

		//Object size limit
		float width_max;
		float lenght_max;
		float height_max;
		float width_min;
		float lenght_min;
		float height_min;

		float volume; //Volume of object

		//-----------------------------------------------------------------------------------------------
		//������ ��������� �������
		StoredObject();
		StoredObject(int layerID, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float limit_array[6], int stepdegree = 10, int maxdegree = 90, float griddensity = 5.0f); //����������� ������ Stored3Dobject
		~StoredObject();

		bool check_isinside_point(const pcl::PointXYZ &check_point);
		void find_bbox(); //Finding BoundingBox
		void Remove();
	};

	//�����, �������� ���������� �� ����� ���� (������������ ������)
	class StorageLayer
	{
	public:
		//-----------------------------------------------------------------------------------------------
		//���� ����
		int UID; //���������� �������� ������������� ����
		time_t AddedDate; //����� ���������� ����

		pcl::PointCloud<pcl::PointXYZ>::Ptr layerNegativeDelta; //������ ���� <float> ������������� ������ ����� ����� �� ������ ���� �� ��������� � �����������
		pcl::PointCloud<pcl::PointXYZ>::Ptr layerPositiveDelta; //������ ���� <float> ������������� ������ ����� ����� �� ������ ���� �� ��������� � ����������� 
		pcl::PointCloud<pcl::PointXYZ>::Ptr DepthMap; //����� ������� (� �������� �����������)
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PositiveClasterList; //�������� ��������� � ������ ������������� ������
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> NegativeClasterList; //�������� ��������� � ������ ������������� ������

		vector<StoredObject> objectForAddList; //������ ���� <int> ��������������� �������� ����������� �� ����
		vector<StoredObject> objectEraserList; //������ �������� ������� ������ ��������� ��������� �������

		//----------------------------------------------------------------------------------------------
		//������ ����
		StorageLayer(); //����������� ������ StorageLayer
		~StorageLayer();

		static void StorageLayer::FindClaster(pcl::PointCloud<pcl::PointXYZ>::Ptr deltacloud, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clastervector, float tolerance = 0.3, int minclastersize = 200, int maxclastersize = 25000);
		void FindObjectForAdd(float minx, float miny, float minz, float maxx, float maxy, float maxz); //������� ������ ��������, ����������� �� ����� ����
	};
	

	class Storage
	{
		//-----------------------------------------------------------------------------------------------
		//���� ������
	public:
		int UID; //���������� �������� ������������� ������
		time_t CreatedDate; //���� �������� ������

		vector<StorageLayer> LayerList; //������ ����� ����������� �� �����
		vector<StoredObject> ObjectList; //������ �������� ����������� �� �����
		vector<int> objectIDForRemoveList; //������ ���� <int> ��������������� �������� ���������� ��� �������� �� ������

		float deltaLimit; //������ �� ������� ����� �������� ����� ����� ������� �������� �� layerDelta ��������� �������������
		float deltaValidPercent; // (0 - 1)������� ����� ����� ������� � ������������� ��������� Delta ��� ������� ������ ����� ������ 

		//----------------------------------------------------------------------------------------------
		//������ ������
		Storage(float deltalimit = 0.06f, float deltavalpercnt = 0.7f);
		~Storage();

		void Storage::CalcNewLayerDelta(const pcl::PointCloud<pcl::PointXYZ>::Ptr &oldcloud,
			const pcl::PointCloud<pcl::PointXYZ>::Ptr &newcloud,
			pcl::PointCloud<pcl::PointXYZ>::Ptr &delta_pos_cloud,
			pcl::PointCloud<pcl::PointXYZ>::Ptr &delta_neg_cloud); //������� ���������� Delta � �������� ����������� ��� ������������ ����
		void AddNewObject(StoredObject &newObject); //������� ���������� ������ �������
		void FindObjectForRemove(vector<StoredObject> objecteraserlist); //������� ������ �������� ��� �������� ����� ���������� ������ ���� (����������� ��� ������� ������������� �������� Delta
		void RemoveObjects(); //������� �������� ���� ��������� ��������
		void RemoveObject(int objectID); //������� �������� i-�� ������� �� ������ 
		void AddNewLayer(StorageLayer newLayer); //������� ���������� ������ ����
	};
}