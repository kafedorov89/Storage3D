#pragma once
#include "stdafx.h"
#include "PCLFunctions.h"

void CloudNoizeFiltration(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud){
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

void CloudPlaneFiltration(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &planed_cloud, float DistanceThreshold){

	pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);

	//Finding plane segments and filtering positive and negative delta
	pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	plane_seg.setOptimizeCoefficients(true);
	//plane_seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);// (pcl::SACMODEL_PLANE);
	plane_seg.setModelType(pcl::SACMODEL_PLANE);
	plane_seg.setMethodType(pcl::SAC_RANSAC);
	plane_seg.setMaxIterations(1000);
	plane_seg.setDistanceThreshold(DistanceThreshold);
	plane_seg.setInputCloud(cloud);
	plane_seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() > 0){
		for (int i = inliers->indices.size() - 1; i > 0; i--){
			plane_cloud_tmp->push_back(cloud->points[inliers->indices[i]]);
		}
		planed_cloud.swap(plane_cloud_tmp);
	}
}