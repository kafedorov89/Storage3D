#pragma once
#include "stdafx.h"
#include "storage.h"

//using namespace pcl;
using namespace std;

Storage::Storage(int uid, float deltalimit, float planethreshold, float zepsangle, bool enableplanefiltration, bool perpendicularonly, bool enablenoizefiltration, float voxeldensity, bool enablevoxelgrig) //float deltavalpercnt
{
	//LayerList = vector<StorageLayer>(0);
	//ObjectList = vector<std::vector<StoredObject>>(0);
	UID = uid;
	deltaLimit = deltalimit;
	DistanceThreshold = planethreshold;
	zEpsAngle = zepsangle;
	enablePlaneFiltration = enableplanefiltration;
	perpendicularOnly = perpendicularonly;
	enableNoizeFiltration = enablenoizefiltration;
	voxelDensity = voxeldensity;
	enableVoxelFiltration = enablevoxelgrig;
	//deltaValidPercent = deltavalpercnt;
}

Storage::~Storage()
{
}

void Storage::CalcNewLayerDelta(){
	//Initialization of point clouds source, target and two outputs
	pcl::PointCloud<pcl::PointXYZ>::Ptr old_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXY>::Ptr old2d_cloud(new pcl::PointCloud<pcl::PointXY>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr delta_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXY>::Ptr delta2d_cloud(new pcl::PointCloud<pcl::PointXY>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr delta_pos_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr delta_neg_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//oldlayer->DepthMap, newlayer->DepthMap, newlayer->layerPositiveDelta, newlayer->layerNegativeDelta

	int llSize = LayerList.size();

	old_cloud = LayerList[llSize - 2]->DepthMap;
	new_cloud = LayerList[llSize - 1]->DepthMap;
	
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

			if (std::abs(deltaZ) > deltaLimit)
			{
				if (deltaZ > 0){
					delta_pos_cloud->push_back(delta_cloud->points[i]);
					//delta_pos_cloud->push_back(founded_old_point);
				}
				else{
					delta_neg_cloud->push_back(delta_cloud->points[i]);
					//delta_neg_cloud->push_back(founded_old_point);
				}
			}
		}
	}

	this->LayerList[llSize - 1]->layerPositiveDelta.swap(delta_pos_cloud);
	this->LayerList[llSize - 1]->layerNegativeDelta.swap(delta_neg_cloud);
}

void Storage::FindObjectForRemove(vector<StoredObject> objecteraserlist){ //Функция поиска объектов для удаления после добавления нового слоя (запускается при наличии отрицательных значений Delta
	objectIDForRemoveList.clear();

	for (int i = 0; i < objecteraserlist.size() - 1; i++){
		for (int k = 0; k < ObjectList.size() - 1; k++){
			//Проверяем находится ли центр k-го объекта внутри i-го удаляющего объема  
			//if (objecteraserlist[i].check_isinside_point(pcl::PointXYZ(ObjectList[i]->position->x(), ObjectList[i]->position->y(), ObjectList[i]->position->z()))){
			if (objecteraserlist[i].check_isinside_point(pcl::PointXYZ(ObjectList[i]->position(0), ObjectList[i]->position(1), ObjectList[i]->position(2)))){
				objectIDForRemoveList.push_back(ObjectList[i]->UID);
			}
		}
	}

	//Showing objects numbers for remove
	std::cout << "Found for remove: " << std::endl;
	for (int i = 0; i < objectIDForRemoveList.size() - 1; i++){
		std::cout << " " << objectIDForRemoveList[i] << " " << std::endl;
	}
}

void Storage::RemoveObject(int objectID){ //Функция удаления i-го объекта со склада 
	ObjectList[objectID]->Remove();
}

void Storage::RemoveObjects(){ //Функция удаления i-го объекта со склада 
	for (int i = 0; i < objectIDForRemoveList.size() - 1; i++){
		this->RemoveObject(i);
	}
}

void Storage::AddNewLayer(StorageLayer& newlayer){ //Функция добавления нового слоя
	StorageLayer* newLayer = new StorageLayer(newlayer);
	
	if (enableVoxelFiltration)
		VoxelGridFiltration(newlayer.DepthMap, newlayer.DepthMap, voxelDensity);

	if (enableNoizeFiltration)
		CloudNoizeFiltration(newlayer.DepthMap, newlayer.DepthMap);

	if (enablePlaneFiltration)
		CloudPlaneFiltration(newlayer.DepthMap, newlayer.DepthMap, DistanceThreshold);

	LayerList.push_back(newLayer);
	//FIXME
}

void Storage::AddNewObject(StoredObject& newobject){ //Функция добавления нового объекта
	StoredObject* newObject = new StoredObject(newobject);
	ObjectList.push_back(newObject);
	//FIXME
}