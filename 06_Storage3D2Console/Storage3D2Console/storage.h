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

using namespace std;

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
	//float deltaValidPercent; // (0 - 1)������� ����� ����� ������� � ������������� ��������� Delta ��� ������� ������ ����� ������
	float DistanceThreshold;
	float zEpsAngle;
	bool planeFiltration;
	bool perpendicularOnly;
	bool noizeFiltration;

	//----------------------------------------------------------------------------------------------
	//������ ������
	Storage(int uid,float deltalimit = 0.06f, float planethreshold = 0.01f, float zepsangle = 0.01f, bool planefiltration = false, bool perpendicularonly = false, bool noizefiltration = true);// float deltavalpercnt = 0.7f
	~Storage();

	void Storage::CalcNewLayerDelta(const pcl::PointCloud<pcl::PointXYZ>::Ptr &oldcloud,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr &newcloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr &delta_pos_cloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr &delta_neg_cloud); //������� ���������� Delta � �������� ����������� ��� ������������ ����
	void AddNewObject(StoredObject& newObject); //������� ���������� ������ �������
	void FindObjectForRemove(vector<StoredObject> objecteraserlist); //������� ������ �������� ��� �������� ����� ���������� ������ ���� (����������� ��� ������� ������������� �������� Delta
	void RemoveObjects(); //������� �������� ���� ��������� ��������
	void RemoveObject(int objectID); //������� �������� i-�� ������� �� ������ 
	void AddNewLayer(StorageLayer& newLayer); //������� ���������� ������ ����
};