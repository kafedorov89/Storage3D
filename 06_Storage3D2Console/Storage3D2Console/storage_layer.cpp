#pragma once
#include "stdafx.h"
#include "storage_layer.h"

using namespace std;

StorageLayer::StorageLayer(){ //Конструктор класса StorageLayer
	layerNegativeDelta = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>); 
	layerPositiveDelta = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);
	DepthMap = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);

	//last_file_name = "last_layer.pcd";

	//objectForAddList = vector<StoredObject>();
	//objectEraserList = vector<StoredObject>();
}

StorageLayer::StorageLayer(const StorageLayer& storagelayer){
	UID = storagelayer.UID;
	storageUID = storagelayer.storageUID;
	AddedDate = storagelayer.AddedDate;
	PositiveClasterList = storagelayer.PositiveClasterList;
	NegativeClasterList = storagelayer.NegativeClasterList;
	objectForAddList = storagelayer.objectForAddList;
	removerList = storagelayer.removerList;
	planeDensity = storagelayer.planeDensity;
	//last_file_name = storagelayer.last_file_name;
	fileName = storagelayer.fileName;

	layerNegativeDelta = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>(*storagelayer.layerNegativeDelta));
	layerPositiveDelta = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>(*storagelayer.layerPositiveDelta));
	DepthMap = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>(*storagelayer.DepthMap));
}

StorageLayer::StorageLayer(int storageuid){ //Конструктор класса StorageLayer
	layerNegativeDelta = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);
	layerPositiveDelta = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);
	DepthMap = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);

	//Generation layer UID via current time
	time_t rawtime;
	struct tm * timeinfo;
	std::string timestring;
	time(&rawtime);
	UID = (int)rawtime;

	std::stringstream fn;
	fn << "layer_" << UID << ".pcd";
	fileName = fn.str();

	//last_file_name = "last_layer.pcd";
	storageUID = storageuid;
}

StorageLayer::~StorageLayer(){

}