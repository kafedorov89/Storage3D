#pragma once
#include "stdafx.h"
#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <string>

#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <Eigen/Eigenvalues> 
#include <pcl/visualization/point_cloud_handlers.h>

#include <vtkStructuredPointsReader.h>
#include <vtkBoundingBox.h>
#include <vtkOBBTree.h>

#include <limits>

#include <pcl/filters/voxel_grid.h>

class StoredObject
{
public:
	StoredObject(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float limit_array[6], int stepdegree = 10, int maxdegree = 90, float griddensity = 5.0f);
	~StoredObject();

	//Validation object
	bool check_valid_object();
	bool isValid;

	bool check_isinside_point(const pcl::PointXYZ &check_point);

	pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud;
	
	//Finding BoundingBox
	void find_bbox();

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
};

