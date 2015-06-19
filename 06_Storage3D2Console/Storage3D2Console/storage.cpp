#pragma once
#include "stdafx.h"
#include "storage.h"

//using namespace pcl;
using namespace std;

Storage::Storage(int uid, float deltalimit, float planethreshold, float zepsangle, bool planefiltration, bool perpendicularonly, bool noizefiltration) //float deltavalpercnt
{
	//LayerList = vector<StorageLayer>(0);
	//ObjectList = vector<std::vector<StoredObject>>(0);
	UID = uid;
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
	pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_old_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXY>::Ptr old2d_cloud(new pcl::PointCloud<pcl::PointXY>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_new_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr delta_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXY>::Ptr delta2d_cloud(new pcl::PointCloud<pcl::PointXY>);

	pcl::VoxelGrid<pcl::PointXYZ> old_vg;
	old_vg.setInputCloud(oldcloud);
	old_vg.setLeafSize(0.008f, 0.008f, 0.008f);
	old_vg.filter(*voxel_old_cloud);
	old_cloud.swap(voxel_old_cloud);

	pcl::VoxelGrid<pcl::PointXYZ> new_vg;
	new_vg.setInputCloud(newcloud);
	new_vg.setLeafSize(0.008f, 0.008f, 0.008f); //FIXME. Need to create function with flexeble values for leaf cloud size (Cloud, Count point per dimentions -> BoundingBox -> dimentions -> setLeafSize -> LightCloud)
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
		CloudPlaneFiltration(delta_pos_cloud, delta_pos_cloud, DistanceThreshold);
		CloudPlaneFiltration(delta_neg_cloud, delta_neg_cloud, DistanceThreshold);
	}
}

void Storage::FindObjectForRemove(vector<StoredObject> objecteraserlist){ //������� ������ �������� ��� �������� ����� ���������� ������ ���� (����������� ��� ������� ������������� �������� Delta
	objectIDForRemoveList.clear();

	for (int i = 0; i < objecteraserlist.size() - 1; i++){
		for (int k = 0; k < ObjectList.size() - 1; k++){
			//��������� ��������� �� ����� k-�� ������� ������ i-�� ���������� ������  
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

void Storage::RemoveObject(int objectID){ //������� �������� i-�� ������� �� ������ 
	ObjectList[objectID]->Remove();
}

void Storage::RemoveObjects(){ //������� �������� i-�� ������� �� ������ 
	for (int i = 0; i < objectIDForRemoveList.size() - 1; i++){
		this->RemoveObject(i);
	}
}

void Storage::AddNewLayer(StorageLayer& newlayer){ //������� ���������� ������ ����
	StorageLayer* newLayer = new StorageLayer(newlayer);
	LayerList.push_back(newLayer);
	//FIXME
}

void Storage::AddNewObject(StoredObject& newobject){ //������� ���������� ������ �������
	StoredObject* newObject = new StoredObject(newobject);
	ObjectList.push_back(newObject);
	//FIXME
}