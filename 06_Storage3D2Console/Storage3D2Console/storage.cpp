#pragma once
#include "stdafx.h"
#include "storage.h"

//using namespace pcl;
using namespace std;

Storage::Storage(float deltalimit, float planethreshold, float zepsangle, bool planefiltration, bool perpendicularonly, bool noizefiltration) //float deltavalpercnt
{
	//LayerList = vector<StorageLayer>(0);
	//ObjectList = vector<std::vector<StoredObject>>(0);

	deltaLimit = deltalimit;
	DistanceThreshold = planethreshold;
	zEpsAngle = zepsangle;
	planeFiltration = planefiltration;
	perpendicularOnly = perpendicularonly;
	noizeFiltration = noizefiltration;
	//deltaValidPercent = deltavalpercnt;
}

Storage::~Storage()
{
}

void Storage::CalcNewLayerDelta(const pcl::PointCloud<pcl::PointXYZ>::Ptr &oldcloud,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr &newcloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &delta_pos_cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &delta_neg_cloud){

	//Initialization of point clouds source, target and two outputs
	pcl::PointCloud<pcl::PointXYZ>::Ptr old_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::io::loadPCDFile("old_cloud.pcd", *old_cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_old_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//voxel_old_cloud->is_dense = true;

	//pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_old_cloud;
	//voxel_old_cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::Filter<pcl::PCLPointCloud2> voxel_old_cloud2(new pcl::Filter<pcl::PCLPointCloud2>());
	//pcl::PCLPointCloud2::Ptr voxel_old_cloud2(new pcl::PCLPointCloud2());

	//pcl::PCLPointCloud2::Ptr oldcloud2(new pcl::PCLPointCloud2());

	pcl::PointCloud<pcl::PointXY>::Ptr old2d_cloud(new pcl::PointCloud<pcl::PointXY>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_new_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr delta_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXY>::Ptr delta2d_cloud(new pcl::PointCloud<pcl::PointXY>);

	//pcl::PointCloud<pcl::PointXYZ>::Ptr plane_delta_pos_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr plane_delta_neg_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_delta_pos_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_delta_neg_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//new_cloud.swap(newcloud);
	/*pcl::VoxelGrid<pcl::PCLPointCloud2> old_vg;
	pcl::toPCLPointCloud2(*oldcloud, *oldcloud2);
	old_vg.setInputCloud(oldcloud2);
	old_vg.setLeafSize(0.01f, 0.01f, 0.01f); //FIXME. Need to create function with flexeble values for leaf cloud size (Cloud, Count point per dimentions -> BoundingBox -> dimentions -> setLeafSize -> LightCloud)
	old_vg.filter(*voxel_old_cloud2);
	pcl::fromPCLPointCloud2(*voxel_old_cloud2, *old_cloud);*/
	//old_cloud.swap(voxel_old_cloud);

	//pcl::io::loadPCDFile("old_cloud.pcd", *old_cloud);

	pcl::VoxelGrid<pcl::PointXYZ> old_vg;
	old_vg.setInputCloud(oldcloud);
	old_vg.setLeafSize(0.01f, 0.01f, 0.01f);
	old_vg.filter(*voxel_old_cloud);
	old_cloud.swap(voxel_old_cloud);

	pcl::VoxelGrid<pcl::PointXYZ> new_vg;
	new_vg.setInputCloud(newcloud);
	new_vg.setLeafSize(0.01f, 0.01f, 0.01f); //FIXME. Need to create function with flexeble values for leaf cloud size (Cloud, Count point per dimentions -> BoundingBox -> dimentions -> setLeafSize -> LightCloud)
	new_vg.filter(*voxel_new_cloud);
	new_cloud.swap(voxel_new_cloud);

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
					delta_pos_cloud->push_back(founded_old_point);
				}
				else{
					delta_neg_cloud->push_back(delta_cloud->points[i]);
					delta_neg_cloud->push_back(founded_old_point);
				}
			}
		}
	}

	if (noizeFiltration)
	{
		CloudNoizeFiltration(delta_pos_cloud, delta_pos_cloud);
		CloudNoizeFiltration(delta_neg_cloud, delta_neg_cloud);
	}

	if (planeFiltration)
	{

		//zEpsAngle
		//DistanceThreshold
		CloudPlaneFiltration(delta_pos_cloud, delta_pos_cloud);
		CloudPlaneFiltration(delta_neg_cloud, delta_neg_cloud);
	}
}

void Storage::AddNewObject(StoredObject &newObject){ //Функция добавления нового объекта

}

void Storage::FindObjectForRemove(vector<StoredObject> objecteraserlist){ //Функция поиска объектов для удаления после добавления нового слоя (запускается при наличии отрицательных значений Delta
	objectIDForRemoveList.clear();

	for (int i = 0; i < objecteraserlist.size() - 1; i++){
		for (int k = 0; k < ObjectList.size() - 1; k++){
			//Проверяем находится ли центр k-го объекта внутри i-го удаляющего объема  
			if (objecteraserlist[i].check_isinside_point(pcl::PointXYZ(ObjectList[i].position(0), ObjectList[i].position(1), ObjectList[i].position(2)))){
				objectIDForRemoveList.push_back(ObjectList[i].UID);
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
	ObjectList[objectID].Remove();
}

void Storage::RemoveObjects(){ //Функция удаления i-го объекта со склада 
	for (int i = 0; i < objectIDForRemoveList.size() - 1; i++){
		this->RemoveObject(i);
	}
}

void Storage::AddNewLayer(StorageLayer newLayer){ //Функция добавления нового слоя

}