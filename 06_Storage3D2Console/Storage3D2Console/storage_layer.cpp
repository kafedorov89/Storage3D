#pragma once
#include "stdafx.h"
#include "storage_layer.h"

using namespace std;

StorageLayer::StorageLayer(){ //����������� ������ StorageLayer
	layerNegativeDelta = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>); //������ ���� <float> ������������� ������ ����� ����� �� ������ ���� �� ��������� � �����������
	layerPositiveDelta = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);  //������ ���� <float> ������������� ������ ����� ����� �� ������ ���� �� ��������� � ����������� 
	DepthMap = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);
}
StorageLayer::~StorageLayer(){

}

void StorageLayer::FindClaster(pcl::PointCloud<pcl::PointXYZ>::Ptr deltacloud, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clastervector, float tolerance, int minclastersize, int maxclastersize){
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	//tree->setInputCloud(cloud_filtered);
	tree->setInputCloud(deltacloud);

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

void StorageLayer::FindObjectForAdd(float minx, float miny, float minz, float maxx, float maxy, float maxz){ //������� ������ ��������, ����������� �� ����� ����
	float obj_size_limit[6] = { minx, miny, minz, maxx, maxy, maxz };

	for (int i = 0; i < PositiveClasterList.size() - 1; i++){
		StoredObject test_object = StoredObject(this->UID, PositiveClasterList[i], obj_size_limit);
		test_object.find_bbox();
		if (test_object.isValid){
			this->objectForAddList.push_back(test_object);
		}
	}
}