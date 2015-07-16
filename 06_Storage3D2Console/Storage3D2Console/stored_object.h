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

#include "PCLFunctions.h"

using namespace std;

class StoredObject
{
	//-----------------------------------------------------------------------------------------------
	//���� ��������� �������
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
	int ObjectType; //������������� ���� ������� (-1 - Undefined; 0 - Parallelogramm; 1 - VerticalCylinder; 2 - HorizontalCylinder)
	time_t AddedDate; //����� ���������� ������� �� �����
	time_t RemovedDate; //����� �������� ������� �� ������

	//-----------------------------------------------------------------------------------------------
	//�������������� ��������� �������
	//Validation object
	
	bool isValid;
	pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud; //������ �������� �������� ���������� ���� ����� �������

	int step_degree; //Step in degree for Searching optimal boundingbox
	int max_degree; //Cout degree where searching optimal boundingbox
	float objectDensity; //Count of point cloud grid in x, y, z dimentions

	//Moving to position
	Eigen::Vector3f position; //Center of object
	
	Eigen::Quaternionf quaternion_to_bbox;
	Eigen::Quaternionf quaternion_to_zero;

	Eigen::Affine3f* jump_to_bbox; //Moving to object's position
	Eigen::Affine3f* jump_to_zero;//Moving to zero

	//Object orientation to set to zero position
	float roll;
	float pitch;
	float yaw;

	//Object size
	float width;
	float lenght;
	float height;
	Eigen::Vector3f upVertex1; //Up cover vertex 1 
	Eigen::Vector3f upVertex2; //Up cover vertex 2
	Eigen::Vector3f upVertex3; //Up cover vertex 3
	Eigen::Vector3f upVertex4; //Up cover vertex 4
	
	bool isGroup;
	bool isHorizontalGroup;
	bool isVerticalGroup;
	//Approximate parameters of objects count
	int minPossibleObjCount; //Minimum possible count 
	int maxPossibleObjCount; //Maximum possible count

	float square;

	//-----------------------------------------------------------------------------------------------
	//������ ��������� �������
	//����������� �� ��������� ������ Stored3Dobject
	StoredObject();
	
	//����������� ����������� ������ Stored3Dobject
	StoredObject(const StoredObject& storedobject);
	
	//����������� ������ Stored3Dobject
	StoredObject(
		int layerid, 
		int storageid, 
		int uid, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, 
		int stepdegree, 
		int maxdegree, 
		float objectdensity,
		string objectname = "");
	
	//Constructor for init object Stored3Dobject from database
	StoredObject(
		int dbuid,
		string dbname,
		time_t dbadd_date,
		float dbposition_x,
		float dbposition_y,
		float dbposition_z,
		float dbwidth,
		float dblenght,
		float dbheight,
		float dbroll,
		float dbpitch,
		float dbyaw,
		int dbobjtype);

	~StoredObject();

	//������� �������� ��������� ����� �� ������� ����������� ������� (�� ��������� ����� ����� >= 70% �� ������� ��������������)
	void find_valid_object_type(float limit_array[6], float valid_percent);

	//������� �������� ��������� ����� (������ ��� ����� ��������������� ������������ ������)
	bool check_isinside_point(const pcl::PointXYZ &check_point);
	
	//������� ������ 
	void find_bbox();
	
	//������� �������� ������� (������� � ��� ��� ������ �� �������� ����������)
	void Remove();

	//������ ���������� ����������� ����������� ������� � ������� ��������� � ������� ����������� � ������� �� ���� �����
	void CalcJamp();
};