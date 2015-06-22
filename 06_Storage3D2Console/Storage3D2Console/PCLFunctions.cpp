#pragma once
#include "stdafx.h"
#include "PCLFunctions.h"

using namespace std;

void VoxelGridFiltration(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &voxeled_cloud, float voxeldensity){
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(voxeldensity, voxeldensity, voxeldensity); //FIXME. Need to create function with flexeble values for leaf cloud size (Cloud, Count point per dimentions -> BoundingBox -> dimentions -> setLeafSize -> LightCloud)
	vg.filter(*filtered_cloud);
 	voxeled_cloud.swap(filtered_cloud);
}

void CloudNoizeFiltration(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud){
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

void CloudPlaneFiltrationPerp(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
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
}

void CloudPlaneFiltration(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &planed_cloud,
	float clastertollerance, int minclastersize, int maxclastersize, float cloud_zstep){

	pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
	

	float minz = FLT_MAX;
	float maxz = -FLT_MAX;
	float cloud_height = 0;
	float cloud_step = cloud_zstep;

	//Get min z coordinate
	//Get max z coordinate
	for (int i = 0; i < cloud->size(); i++){
		if (minz > cloud->points[i].z)
			minz = cloud->points[i].z;
		if (maxz < cloud->points[i].z)
			maxz = cloud->points[i].z;
	}
	
	//Get height of cloud with oZ axiz
	cloud_height = std::abs(maxz - minz);

	for (float istep = minz; istep < maxz; istep += cloud_step){
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> step_clasters;
		pcl::PointCloud<pcl::PointXYZ>::Ptr step_cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointIndices::Ptr step_inliers(new pcl::PointIndices);
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		
		std::cout << istep << std::endl;
		for (int i = 0; i < cloud->size(); i++){
			if (cloud->points[i].z >= istep && cloud->points[i].z < istep + cloud_step){
				step_inliers->indices.push_back(i);
				//plane_cloud_tmp->push_back(cloud->points[i].z);
			}
		}

		extract.setInputCloud(cloud);
		extract.setIndices(step_inliers);
		extract.setNegative(false);
		extract.filter(*step_cloud_tmp);

		//Find clasters in step cloud
		if (step_cloud_tmp->points.size() >= 0){
			FindClasters(step_cloud_tmp, step_clasters, clastertollerance, minclastersize, maxclastersize);

			if (step_clasters.size() > 0){
				for (int k = 0; k < step_clasters.size(); k++){
					for (int p = 0; p < step_clasters[k]->points.size(); p++){
						result_cloud_tmp->points.push_back(step_clasters[k]->points[p]);
					}
				}
			}
		}
	}
	
	planed_cloud.swap(result_cloud_tmp);
}

void FindClasters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clastervector, float tolerance, int minclastersize, int maxclastersize){

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	//tree->setInputCloud(cloud_filtered);
	tree->setInputCloud(cloud);

	vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ>* ec = new pcl::EuclideanClusterExtraction<pcl::PointXYZ>();
	ec->setClusterTolerance(tolerance);
	ec->setMinClusterSize(minclastersize);
	ec->setMaxClusterSize(maxclastersize);
	ec->setSearchMethod(tree);
	ec->setInputCloud(cloud);
	ec->extract(cluster_indices);

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->points.push_back(cloud->points[*pit]);
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		clastervector.push_back(cloud_cluster);
	}

	delete ec;
}

float GetPointDelatZ(pcl::PointXYZ &new_Point, pcl::PointCloud<pcl::PointXY>::Ptr old2d_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr old_cloud, pcl::KdTreeFLANN<pcl::PointXY> &kdtree, int &oldpointindex){
	
	//pcl::PointCloud<pcl::PointXY>::Ptr old2dlayercloud_cloud(new pcl::PointCloud<pcl::PointXY>);

	std::vector<int> pointId_vector; //Vector with 1 lenght for save index of nearest point in old_cloud
	std::vector<float> pointRadius_vector;
	pcl::PointXY *searchPoint = new pcl::PointXY();
	searchPoint->x = new_Point.x;
	searchPoint->y = new_Point.y;

	float deltaZ = 0;
	if (kdtree.nearestKSearch(*searchPoint, 1, pointId_vector, pointRadius_vector) > 0)
	{
		pcl::PointXYZ founded_old_point = old_cloud->points[pointId_vector[0]];
		oldpointindex = pointId_vector[0];
		deltaZ = founded_old_point.z - new_Point.z; //Keep that axiz oZ is inverted (goes from up to down)
	}
	return deltaZ;

}

float GetNPointsDelatZ(pcl::PointXYZ &new_Point, pcl::PointCloud<pcl::PointXY>::Ptr old2d_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr old_cloud, pcl::KdTreeFLANN<pcl::PointXY> &kdtree, int pointscount){

	//pcl::PointCloud<pcl::PointXY>::Ptr old2dlayercloud_cloud(new pcl::PointCloud<pcl::PointXY>);

	std::vector<int> pointId_vector; //Vector with 1 lenght for save index of nearest point in old_cloud
	std::vector<float> pointRadius_vector;
	pcl::PointXY *searchPoint = new pcl::PointXY();
	searchPoint->x = new_Point.x;
	searchPoint->y = new_Point.y;
	float deltaZ = 0;

	if (kdtree.nearestKSearch(*searchPoint, pointscount, pointId_vector, pointRadius_vector) > 0)
	{
		for (int p = 0; p < pointId_vector.size(); p++){
			pcl::PointXYZ founded_old_point = old_cloud->points[pointId_vector[p]];
			deltaZ += founded_old_point.z - new_Point.z; //Keep that axiz oZ is inverted (goes from up to down)
		}

		deltaZ = deltaZ / (float)pointId_vector.size();
	}

	return deltaZ;
}

void Get2DCloudFrom3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3d, pcl::PointCloud<pcl::PointXY>::Ptr &cloud2d){
	//Convert old_cloud to old2d_cloud
	for (int i = 0; i < cloud3d->size() - 1; i++){
		pcl::PointXY* old2d_Point = new pcl::PointXY();
		old2d_Point->x = cloud3d->points[i].x;
		old2d_Point->y = cloud3d->points[i].y;
		cloud2d->push_back(*old2d_Point);
	}
}