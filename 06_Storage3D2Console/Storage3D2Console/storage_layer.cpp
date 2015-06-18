#pragma once
#include "stdafx.h"
#include "storage_layer.h"

using namespace std;

StorageLayer::StorageLayer(){ //Конструктор класса StorageLayer
	layerNegativeDelta = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>); 
	layerPositiveDelta = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);
	DepthMap = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);

	//objectForAddList = vector<StoredObject>();
	//objectEraserList = vector<StoredObject>();
}

StorageLayer::StorageLayer(int layeruid, int storageuid){ //Конструктор класса StorageLayer
	layerNegativeDelta = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);
	layerPositiveDelta = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);
	DepthMap = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);

	//objectForAddList = vector<StoredObject>();
	//objectEraserList = vector<StoredObject>();
	UID = layeruid;
	storageUID = storageuid;
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

		clastervector.push_back(cloud_cluster);
	}
}

void StorageLayer::FindObjectForAdd(float minx, float miny, float minz, float maxx, float maxy, float maxz){ //Функция поиска объектов, добавленных на новом слое
	float obj_size_limit[6] = { minx, miny, minz, maxx, maxy, maxz };

	for (int i = 0; i < PositiveClasterList.size() - 1; i++){
		
		time_t rawtime;
		time(&rawtime);
		
		StoredObject test_object = StoredObject(this->UID, this->storageUID, (int)rawtime, PositiveClasterList[i], obj_size_limit);
		test_object.find_bbox();
		test_object.check_valid_object();

		//if (test_object.isValid){
		objectForAddList.push_back(test_object);
		//}
		//delete test_object;
	}
}

void StorageLayer::SaveLayerToPCD(bool firstLayer){
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
	
	if (!firstLayer){
		pcl::io::savePCDFileBinaryCompressed("delta_pos_cloud_" + timestring + ".pcd", *layerPositiveDelta);
		pcl::io::savePCDFileBinaryCompressed("delta_neg_cloud_" + timestring + ".pcd", *layerNegativeDelta);
	}

	pcl::io::savePCDFileBinaryCompressed("layer_cloud_" + timestring + ".pcd", *DepthMap);
}