// PCD_ConsoleViewer.cpp : Defines the entry point for the console application.
//


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

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/search/search.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/voxel_grid.h>


using namespace pcl;
using namespace std;

int _tmain(int argc, _TCHAR* argv[])
{
	//Initialization of point clouds source, target and two outputs
	pcl::PointCloud<pcl::PointXYZ>::Ptr old_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_old_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_old_cloud (new pcl::PointCloud<pcl::PointXY>);
	
	pcl::PointCloud<pcl::PointXY>::Ptr old2d_cloud(new pcl::PointCloud<pcl::PointXY>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr delta_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr downdelta_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXY>::Ptr delta2d_cloud(new pcl::PointCloud<pcl::PointXY>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr delta_pos_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr delta_neg_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::PointCloud<pcl::PointXYZ> delta_pos_cloud_save;
	pcl::PointCloud<pcl::PointXYZ> delta_neg_cloud_save;

	//Loading old and new point cloud
	pcl::io::loadPCDFile("old_cloud.pcd", *old_cloud);
	pcl::io::loadPCDFile("new_cloud.pcd", *new_cloud);

	//rs_old_cloud = old_cloud;

	pcl::VoxelGrid<pcl::PointXYZ> old_vg;
	old_vg.setInputCloud(old_cloud);
	old_vg.setLeafSize(0.01f, 0.01f, 0.01f);
	old_vg.filter(*voxel_old_cloud);

	old_cloud.swap(voxel_old_cloud);

	pcl::VoxelGrid<pcl::PointXYZ> new_vg;
	new_vg.setInputCloud(new_cloud);
	new_vg.setLeafSize(0.01f, 0.01f, 0.01f);
	new_vg.filter(*voxel_new_cloud);

	new_cloud.swap(voxel_new_cloud);



	//1. Segment differences 
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	//pcl::search::Octree<pcl::PointXYZ>::Ptr tree;// (new pcl::search::Octree(<pcl::PointXYZ>);
	pcl::SegmentDifferences<pcl::PointXYZ> sdiff;
	sdiff.setInputCloud(new_cloud);
	sdiff.setTargetCloud(old_cloud);
	sdiff.setSearchMethod(tree);
	sdiff.setDistanceThreshold(0); //PARAMETR?
	sdiff.segment(*delta_cloud);
	
	std::cout << *delta_cloud << std::endl;
	


	//2D - variant
	//Convert old cloud to 2d-cloud
	for (int i = 0; i < old_cloud->size() - 1; i++){
		pcl::PointXY old2d_Point;
		old2d_Point.x = old_cloud->points[i].x;
		old2d_Point.y = old_cloud->points[i].y;
		old2d_cloud->push_back(old2d_Point);
	}

	//Convert delta cloud to 2d-cloud
	for (int i = 0; i < delta_cloud->size() - 1; i++){
		pcl::PointXY delta2d_Point;
		delta2d_Point.x = delta_cloud->points[i].x;
		delta2d_Point.y = delta_cloud->points[i].y;
		delta2d_cloud->push_back(delta2d_Point);
	}
	
	//std::vector<int> delta_indices = vector<int>();
	std::vector<int> delta_pos_indices = vector<int>();
	std::vector<int> delta_neg_indices = vector<int>();
	pcl::KdTreeFLANN<pcl::PointXY> kdtree;
	kdtree.setInputCloud(old2d_cloud);
	float deltaZ; //Lenght between pair of points from delta_cloud and old_cloud
	float deltaLimit = 0.06;//PARAMETR

	//Finding pair points for delta2d cloud in old2d cloud
	for (int i = 0; i < delta2d_cloud->size() - 1; i++){
		pcl::PointXY searchPoint;
		searchPoint.x = delta2d_cloud->points[i].x,
		searchPoint.y = delta2d_cloud->points[i].y;
		
		std::vector<int> pointId_vector; //Vector with 1 lenght for save index of nearest point in old_cloud
		std::vector<float> pointRadius_vector;

		if (kdtree.nearestKSearch(searchPoint, 1, pointId_vector, pointRadius_vector) > 0)
		{
			pcl::PointXYZ founded_old_point = old_cloud->points[pointId_vector[0]];
			deltaZ = founded_old_point.z - delta_cloud->points[i].z; //Keep that axiz oZ is inverted (goes from up to down)
			//delta_indices.insert(delta_indices.end(), pointId_vector.begin(), pointId_vector.end());
			if (std::abs(deltaZ) > deltaLimit)
			{
				if (deltaZ > 0){
					delta_pos_cloud->push_back(delta_cloud->points[i]);
					delta_pos_cloud->push_back(founded_old_point);
					//delta_pos_indices.insert(delta_pos_indices.end(), pointId_vector.begin(), pointId_vector.end());
				}
				else{
					delta_neg_cloud->push_back(delta_cloud->points[i]);
					delta_neg_cloud->push_back(founded_old_point);
					//delta_neg_indices.insert(delta_neg_indices.end(), pointId_vector.begin(), pointId_vector.end());
				}
			}
		}
	}



	delta_pos_cloud_save = pcl::PointCloud<pcl::PointXYZ>(delta_pos_cloud->size(), 1, pcl::PointXYZ());
	delta_neg_cloud_save = pcl::PointCloud<pcl::PointXYZ>(delta_neg_cloud->size(), 1, pcl::PointXYZ());
	
	for (int i = 0; i < delta_pos_cloud->size() - 1; i++){
		delta_pos_cloud_save[i].x = delta_pos_cloud->at(i).x;
		delta_pos_cloud_save[i].y = delta_pos_cloud->at(i).y;
		delta_pos_cloud_save[i].z = delta_pos_cloud->at(i).z;
	}

	for (int i = 0; i < delta_neg_cloud->size() - 1; i++){
		delta_neg_cloud_save[i].x = delta_neg_cloud->at(i).x;
		delta_neg_cloud_save[i].y = delta_neg_cloud->at(i).y;
		delta_neg_cloud_save[i].z = delta_neg_cloud->at(i).z;
	}

	pcl::io::savePCDFileBinaryCompressed("delta_pos_cloud.pcd", delta_pos_cloud_save);
	pcl::io::savePCDFileBinaryCompressed("delta_neg_cloud.pcd", delta_neg_cloud_save);



	//Visualiztion 
	pcl::visualization::PCLVisualizer vis("3D View");
	
	vis.setBackgroundColor(0, 0, 0);
	vis.addCoordinateSystem(1.0);
	vis.initCameraParameters();

	vis.addPointCloud(delta_pos_cloud, "delta_pos_cloud", 0);
	vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)255 / (float)255, (float)0 / (float)255, (float)0 / (float)255, "delta_pos_cloud");
	vis.addPointCloud(delta_neg_cloud, "delta_neg_cloud", 0);
	vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)0 / (float)255, (float)255 / (float)255, (float)0 / (float)255, "delta_neg_cloud");

	while (!vis.wasStopped())
	{
		vis.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	
	return 0;
}