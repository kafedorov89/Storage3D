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
#include "StoredObject.h"

using namespace std;


int _tmain(int argc, _TCHAR* argv[])
{
	string filename = "point_cloud.pcd";
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	if (pcl::io::loadPCDFile(filename, *cloud) == -1)
		return (-1);

	float limit_array[6] = { 0.2, 0.2, 0.2, 0.4, 0.4, 0.4 };
	StoredObject storedobject = StoredObject(cloud, limit_array);
	
	//storedobject.find_bbox();
	
	//StoredObject::find_bbox(cloud, position, quat, min_x_width, min_y_lenght, min_z_height);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	//viewer->setCameraPosition(1, 0, 0, 0, 1, 0);
	viewer->initCameraParameters();

	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	viewer->addCube(storedobject.position, storedobject.quaternion_to_bbox, storedobject.width, storedobject.lenght, storedobject.height, "OBB");

	//viewer->addPointCloud<pcl::PointXYZ>(zero_point_cloud, "start cloud");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)0 / (float)255, (float)0 / (float)255, (float)255 / (float)255, "start cloud");
	//viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 1.0, "AABB");
	//viewer->addPointCloud<pcl::PointXYZ>(minimal_cloud, "final cloud");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)255 / (float)255, (float)0 / (float)255, (float)255 / (float)255, "OBB");

	//storedobject.check_valid_object();
	pcl::PointXYZ check_point = pcl::PointXYZ(-1.0973517, 0.306992, 2.26491);
	storedobject.check_isinside_point(check_point);
	viewer->addSphere(check_point, 0.01, 1.0, 0.0, 0.0, "check_point_sphere");
	std::cout << "Center position = " << storedobject.position(0) << " " << storedobject.position(1) << " " << storedobject.position(2) << std::endl;

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return (0);
}