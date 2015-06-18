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
#include <pcl/point_cloud.h>
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

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "stored_object.h"

using namespace std;

//�����, �������� ���������� �� ����� ���� (������������ ������)
class StorageLayer
{
public:
	//-----------------------------------------------------------------------------------------------
	//���� ����
	int UID; //���������� �������� ������������� ����
	int storageUID; //���������� �������� ������������� ������
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
	StorageLayer(int layeruid, int storageuid); //����������� ������ StorageLayer
	~StorageLayer();

	static void StorageLayer::FindClaster(pcl::PointCloud<pcl::PointXYZ>::Ptr deltacloud, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clastervector, float tolerance = 0.08, int minclastersize = 200, int maxclastersize = 250000);
	void FindObjectForAdd(float minx, float miny, float minz, float maxx, float maxy, float maxz); //������� ������ ��������, ����������� �� ����� ����
	void SaveLayerToPCD(bool firstLayer = false);
};
