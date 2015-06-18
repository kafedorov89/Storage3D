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

//------------------------------------------------------------------

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <Eigen/Eigenvalues> 
#include <pcl/visualization/point_cloud_handlers.h>
#include <vtkStructuredPointsReader.h>
#include <vtkBoundingBox.h>
#include <vtkOBBTree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <Eigen/Eigenvalues> 
#include <pcl/visualization/point_cloud_handlers.h>
#include <vtkStructuredPointsReader.h>
#include <vtkBoundingBox.h>
#include <vtkOBBTree.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

class StoredObject
{
	//-----------------------------------------------------------------------------------------------
	//���� ��������� �������
public:
	//��������� ��������� ������
	bool removed; //���� ���������� �������

	//��������� ����� ������� � ������� ���������� ������
	int UID; //���������� �������� ������������� �������
	int storageUID; //���������� ������������� ������ �� ������� �������� ������
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
	StoredObject(int layerid, int storageid, int uid, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float limit_array[6], int stepdegree = 10, int maxdegree = 90, float griddensity = 5.0f); //����������� ������ Stored3Dobject
	~StoredObject();

	bool check_isinside_point(const pcl::PointXYZ &check_point);
	void find_bbox(); //Finding BoundingBox
	void Remove();
};