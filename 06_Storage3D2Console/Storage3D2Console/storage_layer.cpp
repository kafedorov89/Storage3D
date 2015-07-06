#pragma once
#include "stdafx.h"
#include "storage_layer.h"

using namespace std;

StorageLayer::StorageLayer(){ //Конструктор класса StorageLayer
	layerNegativeDelta = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>); 
	layerPositiveDelta = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);
	DepthMap = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);

	last_file_name = "last_layer.pcd";

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
	last_file_name = storagelayer.last_file_name;

	layerNegativeDelta = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>(*storagelayer.layerNegativeDelta));
	layerPositiveDelta = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>(*storagelayer.layerPositiveDelta));
	DepthMap = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>(*storagelayer.DepthMap));
}

StorageLayer::StorageLayer(int layeruid, int storageuid){ //Конструктор класса StorageLayer
	layerNegativeDelta = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);
	layerPositiveDelta = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);
	DepthMap = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);

	//objectForAddList = vector<StoredObject>();
	//objectEraserList = vector<StoredObject>();
	last_file_name = "last_layer.pcd";
	UID = layeruid;
	storageUID = storageuid;
}

StorageLayer::~StorageLayer(){

}


void StorageLayer::SaveLayerToPCD(bool firstLayer, bool lastmodeon){
	if (!lastmodeon){
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

		try{
			if (!firstLayer){
				pcl::io::savePCDFileBinaryCompressed("delta_pos_cloud_" + timestring + ".pcd", *layerPositiveDelta);
				pcl::io::savePCDFileBinaryCompressed("delta_neg_cloud_" + timestring + ".pcd", *layerNegativeDelta);
			}

			pcl::io::savePCDFileBinaryCompressed("./database/layer_cloud_" + timestring + ".pcd", *DepthMap);
		}
		catch (std::exception& e){

		}
	}else{
		try{
			std::remove(last_file_name);
		}
		catch (std::exception& e){

		}

		try{
			pcl::io::savePCDFileBinaryCompressed(last_file_name, *DepthMap);
		}
		catch (std::exception& e){

		}
		
	}
}