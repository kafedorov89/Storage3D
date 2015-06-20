#pragma once
#include "stdafx.h"
#include "PCLFunctions.h"

void VoxelGridFiltration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &voxeled_cloud, float voxeldensity){
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(voxeldensity, voxeldensity, voxeldensity); //FIXME. Need to create function with flexeble values for leaf cloud size (Cloud, Count point per dimentions -> BoundingBox -> dimentions -> setLeafSize -> LightCloud)
	vg.filter(*filtered_cloud);
	voxeled_cloud.swap(filtered_cloud);
}

void CloudNoizeFiltration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud){
	// Create the filtering object
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);

	sor.setMeanK(10);
	sor.setStddevMulThresh(1.0);
	sor.setNegative(false);
	sor.filter(*filtered_cloud_tmp);
	filtered_cloud.swap(filtered_cloud_tmp);
}

void CloudPlaneFiltration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &planed_cloud, 
	float distancethreshold, 
	bool negative){
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	//Finding plane segments and filtering positive and negative delta
	pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	plane_seg.setOptimizeCoefficients(true);
	//plane_seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);// (pcl::SACMODEL_PLANE);
	plane_seg.setModelType(pcl::SACMODEL_PLANE);
	plane_seg.setMethodType(pcl::SAC_RANSAC);
	plane_seg.setMaxIterations(1000);
	plane_seg.setDistanceThreshold(distancethreshold);
	plane_seg.setInputCloud(cloud);
	plane_seg.segment(*inliers, *coefficients);

	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(negative);
	extract.filter(*plane_cloud_tmp);
	planed_cloud.swap(plane_cloud_tmp);

	/*if (inliers->indices.size() > 0){
		for (int i = inliers->indices.size() - 1; i > 0; i--){
			plane_cloud_tmp->push_back(cloud->points[inliers->indices[i]]);
		}
	}*/
}

void CloudPlaneFiltrationPerp(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &planed_cloud,
	float distancethreshold,
	float zepsangle,
	bool negative){

	pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	//Finding plane segments and filtering positive and negative delta
	pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	plane_seg.setOptimizeCoefficients(true);
	//plane_seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);// (pcl::SACMODEL_PLANE);
	plane_seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
	
	plane_seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
	plane_seg.setMethodType(pcl::SAC_RANSAC);
	plane_seg.setMaxIterations(1000);
	plane_seg.setDistanceThreshold(distancethreshold);
	plane_seg.setAxis(Eigen::Vector3f(0, 0, 1));
	plane_seg.setEpsAngle(zepsangle);
	plane_seg.setInputCloud(cloud);
	plane_seg.segment(*inliers, *coefficients);

	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(negative);
	extract.filter(*plane_cloud_tmp);
	planed_cloud.swap(plane_cloud_tmp);

	/*if (inliers->indices.size() > 0){
	for (int i = inliers->indices.size() - 1; i > 0; i--){
	plane_cloud_tmp->push_back(cloud->points[inliers->indices[i]]);
	}
	}*/
}