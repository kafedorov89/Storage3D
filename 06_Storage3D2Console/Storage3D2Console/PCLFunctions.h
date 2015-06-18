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

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/statistical_outlier_removal.h>

void CloudNoizeFiltration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud){
	// Create the filtering object
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);

	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.setNegative(false);
	sor.filter(*filtered_cloud_tmp);
	filtered_cloud.swap(filtered_cloud_tmp);
}

void CloudPlaneFiltration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &planed_cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);

	//Finding plane segments and filtering positive and negative delta
	pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	plane_seg.setOptimizeCoefficients(true);
	plane_seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);// (pcl::SACMODEL_PLANE);
	plane_seg.setMethodType(pcl::SAC_RANSAC);
	plane_seg.setMaxIterations(1000);
	plane_seg.setDistanceThreshold(0.03);
	plane_seg.setInputCloud(cloud);
	plane_seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() > 0){
		for (int i = inliers->indices.size() - 1; i > 0; i--){
			plane_cloud_tmp->push_back(cloud->points[inliers->indices[i]]);
		}
		planed_cloud.swap(plane_cloud_tmp);
	}
}

/*void CloudPerpendicularPlaneFiltration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &planed_cloud){

	
	
	neg_plane_seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);

	neg_plane_seg.setAxis(Eigen::Vector3f(0, 0, 1));
	pos_plane_seg.setAxis(Eigen::Vector3f(0, 0, 1));
	neg_plane_seg.setEpsAngle((20 * 3.14) / 180);
	pos_plane_seg.setEpsAngle((20 * 3.14) / 180);
	//neg_plane_seg.setEpsAngle(0.1);
	//pos_plane_seg.setEpsAngle(0.1);
	//neg_plane_seg.setEpsAngle(zEpsAngle);
	//pos_plane_seg.setEpsAngle(zEpsAngle);
}*/