#pragma once
#include "stdafx.h"
#include <iostream>
#include <sys/types.h> 
#include <dirent.h>
#include <regex>
#include <vector>
#include <string>
#include <limits> 

#define NOMINMAX
#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif

#include <Eigen/Eigenvalues>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/search/search.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

//------------------------------------------------------------------

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/transforms.h>

#include <vtkStructuredPointsReader.h>
#include <vtkBoundingBox.h>
#include <vtkOBBTree.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/point_cloud.h>

#include "stored_object.h"
#include "storage_layer.h" 
#include "PCLFunctions.h"

#include <iostream>
#include <time.h>

#include "sqlite/SQLiteDatabase.h"
//#include <ctime>
//#include <time.h>

using namespace std;

class Storage
{
	//-----------------------------------------------------------------------------------------------
	//���� ������
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	int UID; //���������� �������� ������������� ������
	time_t CreatedDate; //���� �������� ������

	vector<StorageLayer*> LayerList; //������ ����� ����������� �� �����
	vector<StoredObject*> ObjectList; //������ �������� ����������� �� �����
	//vector<int> objectIDForRemoveList; //������ ���� <int> ��������������� �������� ���������� ��� �������� �� ������

	float deltaLimit; //������ �� ������� ����� �������� ����� ����� ������� �������� �� layerDelta ��������� �������������
	//float deltaValidPercent; // (0 - 1)������� ����� ����� ������� � ������������� ��������� Delta ��� ������� ������ ����� ������
	bool enablePlaneFiltration; 
	bool enableNoizeFiltration;
	bool enableVoxelFiltration;
	char* dbName;

	float PlaneClasterTollerance;
	int MinPlaneClasterSize;
	int MaxPlaneClasterSize;
	float CloudZStep;
	SQLiteDatabase* DataBase;

	float ObjectLimitSize[6];

	//----------------------------------------------------------------------------------------------
	//������ ������

	Storage(int uid,
		float deltalimit,
		bool enablevoxelfiltration,
		bool enableplanefiltration,
		bool enablenoizefiltration,
		char* dbname);
	~Storage();

	//������� ���������� Delta � �������� ����������� ��� ������������ ����
	void Storage::CalcNewLayerDelta(float PlaneClasterTollerance, int MinPlaneClasterSize, int MaxPlaneClasterSize, float CloudZStep); 
	
	//������� ���������� i-�� ������� �� ���� ������
	void AddObjectFromDatabase(StoredObject& newobject);
	
	//������� ���������� ������ �������
	void AddNewObject(StoredObject& newobject); 
	
	//������� �������� ���� ��������� ��������
	void RemoveObjects(int curLayerUID, float valid_percent, int nearestpoinscount, float object_density); 
	
	//������� �������� i-�� ������� �� ������ 
	void RemoveObject(int objectID); 
	
	//������� ������ ��������, ����������� �� ����� ����
	void FindObjects(int LayerUID, float valid_percent, int nearestpoinscount, float object_density); 
	
	//������� ���������� ������ ����
	void AddNewLayer(StorageLayer& newLayer); 
	
	//������� ������������� ������ �� ���� ������
	void initStorageFromDB();

	//void saveStorageToDB();
	/*vector<int> GetAllObjects();
	vector<int> GetActualObjects();
	vector<int> GetRemovedObjects();
	vector<int> GetObjectsByPointXYZ(pcl::PointXYZ& testpoint);
	vector<int> GetObjectsAddedInTimeInterval(time_t starttime, time_t endtime);*/
};