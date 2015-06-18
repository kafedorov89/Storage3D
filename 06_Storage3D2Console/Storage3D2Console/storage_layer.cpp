#pragma once
#include "stdafx.h"
#include "storage_layer.h"

using namespace pcl;
using namespace std;

StorageLayer::StorageLayer(){ //Конструктор класса StorageLayer
	layerNegativeDelta = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>); //Массив типа <float> отрицательных разниц высот точек на данном слое по отношению к предыдущему
	layerPositiveDelta = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);  //Массив типа <float> положительных разниц высот точек на данном слое по отношению к предыдущему 
	DepthMap = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);
}
StorageLayer::~StorageLayer(){

}

void StorageLayer::FindClaster(pcl::PointCloud<pcl::PointXYZ>::Ptr deltacloud, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clastervector, float tolerance, int minclastersize, int maxclastersize){
	
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	//tree->setInputCloud(cloud_filtered);
	tree->setInputCloud(deltacloud);

	/*std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(tolerance);
	ec.setMinClusterSize(minclastersize);
	ec.setMaxClusterSize(maxclastersize);
	ec.setSearchMethod(tree);
	ec.setInputCloud(deltacloud);
	ec.extract(cluster_indices);*/

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(tolerance);
	ec.setMinClusterSize(minclastersize);
	ec.setMaxClusterSize(maxclastersize);
	ec.setSearchMethod(tree);
	ec.setInputCloud(deltacloud);
	ec.extract(cluster_indices);

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->points.push_back(deltacloud->points[*pit]);
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		
		
		//Finding plane segments and filtering positive and negative delta
		pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud_claster(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::SACSegmentation<pcl::PointXYZ> claster_plane_seg;
		pcl::ModelCoefficients::Ptr claster_coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr claster_inliers(new pcl::PointIndices);
		claster_plane_seg.setOptimizeCoefficients(true);

		//claster_plane_seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
		//claster_plane_seg.setAxis(Eigen::Vector3f(0, 0, 1));
		//claster_plane_seg.setEpsAngle(0.1);

		claster_plane_seg.setModelType(pcl::SACMODEL_PLANE);

		claster_plane_seg.setMethodType(pcl::SAC_RANSAC);
		claster_plane_seg.setDistanceThreshold(0.1);
		claster_plane_seg.setInputCloud(cloud_cluster);
		claster_plane_seg.segment(*claster_inliers, *claster_coefficients);

		if (claster_inliers->indices.size() > 0){
			for (int i = claster_inliers->indices.size() - 1; i > 0; i--){
				//pcl::PointCloud<PointXYZ>::const_iterator del = ;
				plane_cloud_claster->push_back(cloud_cluster->points[claster_inliers->indices[i]]);
			}
			cloud_cluster.swap(plane_cloud_claster);
		}



		clastervector.push_back(cloud_cluster);
	}
}

void StorageLayer::FindObjectForAdd(float minx, float miny, float minz, float maxx, float maxy, float maxz){ //Функция поиска объектов, добавленных на новом слое
	float obj_size_limit[6] = { minx, miny, minz, maxx, maxy, maxz };

	for (int i = 0; i < PositiveClasterList.size() - 1; i++){
		StoredObject test_object = StoredObject(this->UID, PositiveClasterList[i], obj_size_limit);
		test_object.find_bbox();
		if (test_object.isValid){
			this->objectForAddList.push_back(test_object);
		}
	}
}

void StorageLayer::SaveLayerToPCD(bool firstLayer){
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr delta_pos_cloud_save(new pcl::PointCloud<pcl::PointXYZ>);// = pcl::PointCloud<pcl::PointXYZ>(layerPositiveDelta->size(), 1);// = pcl::PointCloud<pcl::PointXYZ>();
	pcl::PointCloud<pcl::PointXYZ>::Ptr delta_neg_cloud_save(new pcl::PointCloud<pcl::PointXYZ>);// = pcl::PointCloud<pcl::PointXYZ>(layerNegativeDelta->size(), 1);// = pcl::PointCloud<pcl::PointXYZ>();
	pcl::PointCloud<pcl::PointXYZ>::Ptr delta_cloud_save(new pcl::PointCloud<pcl::PointXYZ>);// = pcl::PointCloud<pcl::PointXYZ>(DepthMap->size(), 1);// = pcl::PointCloud<pcl::PointXYZ>();

	for (int i = 0; i < layerPositiveDelta->size() - 1; i++){
		//delta_pos_cloud_save[i] = new pcl::PointXYZ();
		delta_pos_cloud_save->at(i).x = layerPositiveDelta->at(i).x;
		delta_pos_cloud_save->at(i).y = layerPositiveDelta->at(i).y;
		delta_pos_cloud_save->at(i).z = layerPositiveDelta->at(i).z;
	}

	for (int i = 0; i < layerNegativeDelta->size() - 1; i++){
		//delta_neg_cloud_save[i] = new pcl::PointXYZ();
		delta_neg_cloud_save->at(i).x = layerNegativeDelta->at(i).x;
		delta_neg_cloud_save->at(i).y = layerNegativeDelta->at(i).y;
		delta_neg_cloud_save->at(i).z = layerNegativeDelta->at(i).z;
	}

	for (int i = 0; i < DepthMap->size() - 1; i++){
		//delta_cloud_save[i] = new pcl::PointXYZ();
		delta_cloud_save->at(i).x = DepthMap->at(i).x;
		delta_cloud_save->at(i).y = DepthMap->at(i).y;
		delta_cloud_save->at(i).z = DepthMap->at(i).z;
	}*/

	time_t rawtime;
	struct tm * timeinfo;
	std::string timestring;

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	timestring = std::to_string(timeinfo->tm_sec) +
		std::to_string(timeinfo->tm_hour) +
		std::to_string(timeinfo->tm_min) +
		std::to_string(timeinfo->tm_mday) +
		std::to_string(timeinfo->tm_mon) +
		std::to_string(timeinfo->tm_year);
	
	//pcl::io::savePCDFileBinaryCompressed("delta_pos_cloud_" + timestring + ".pcd", *delta_pos_cloud_save);
	//pcl::io::savePCDFileBinaryCompressed("delta_neg_cloud_" + timestring + ".pcd", *delta_neg_cloud_save);
	//pcl::io::savePCDFileBinaryCompressed("delta_cloud_" + timestring + ".pcd", *delta_neg_cloud_save);*/

	if (!firstLayer){
		pcl::io::savePCDFileBinaryCompressed("delta_pos_cloud_" + timestring + ".pcd", *layerPositiveDelta);
		pcl::io::savePCDFileBinaryCompressed("delta_neg_cloud_" + timestring + ".pcd", *layerNegativeDelta);
	}

	pcl::io::savePCDFileBinaryCompressed("layer_cloud_" + timestring + ".pcd", *DepthMap);

	//pcl::io::savePCDFileBinaryCompressed("delta_pos_cloud.pcd", *layerPositiveDelta);
	//pcl::io::savePCDFileBinaryCompressed("delta_neg_cloud.pcd", *layerNegativeDelta);
	//pcl::io::savePCDFileBinaryCompressed("delta_cloud.pcd", *DepthMap);
}