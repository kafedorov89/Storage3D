#pragma once
#include "stdafx.h"
#include "storage.h"

//using namespace pcl;
using namespace std;

Storage::Storage(int uid,
	float deltalimit,
	bool enablevoxelfiltration,
	bool enableplanefiltration,
	bool enablenoizefiltration)
{
	UID = uid;
	deltaLimit = deltalimit;
	enableVoxelFiltration = enablevoxelfiltration;
	enablePlaneFiltration = enableplanefiltration;
	enableNoizeFiltration = enablenoizefiltration;
}

Storage::~Storage()
{
}

void Storage::CalcNewLayerDelta(float PlaneClasterTollerance, int MinPlaneClasterSize, int MaxPlaneClasterSize, float CloudZStep){
	//Initialization of point clouds source, target and two outputs
	pcl::PointCloud<pcl::PointXYZ>::Ptr old_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXY>::Ptr old2d_cloud(new pcl::PointCloud<pcl::PointXY>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr delta_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr delta_pos_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr delta_neg_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//oldlayer->DepthMap, newlayer->DepthMap, newlayer->layerPositiveDelta, newlayer->layerNegativeDelta

	int llSize = LayerList.size();

	old_cloud = LayerList[llSize - 2]->DepthMap;
	new_cloud = LayerList[llSize - 1]->DepthMap;
	
	Get2DCloudFrom3D(old_cloud, old2d_cloud);

	pcl::KdTreeFLANN<pcl::PointXY> kdtree;
	kdtree.setInputCloud(old2d_cloud);
	
	float deltaZ = 0; //Lenght between pair of points from delta_cloud and old_cloud
	int oldPointIndex = 0;
	//Finding pair points for delta2d cloud in old2d cloud
	for (int i = 0; i < new_cloud->size(); i++){
		deltaZ = GetPointDelatZ(new_cloud->points[i], old2d_cloud, old_cloud, kdtree, oldPointIndex);

		if (std::abs(deltaZ) >= deltaLimit){
			if (deltaZ > 0){
				delta_pos_cloud->push_back(new_cloud->points[i]);
				//delta_pos_cloud->push_back(old_cloud->points[oldPointIndex]);
			}
			else{
				delta_neg_cloud->push_back(new_cloud->points[i]);
				//delta_neg_cloud->push_back(old_cloud->points[oldPointIndex]);
			}
		}
	}

	//Auto calculation claster's parameters
	float cur_planeDensity = LayerList[llSize - 1]->planeDensity;
	

	/*float min_mult = 1;
	float max_mult = 10;
	float toll_milt = 1.5;
	float step_milt = 3;
	float CloudZStep = step_milt * deltaLimit;
	float posPlaneClasterTollerance = toll_milt * cur_planeDensity;
	int posMinPlaneClasterSize = (int)(min_mult * (((float)ObjectLimitSize[0] / (float)cur_planeDensity) * ((float)ObjectLimitSize[1] / (float)cur_planeDensity)));
	int posMaxPlaneClasterSize = (int)(max_mult * (((float)ObjectLimitSize[3] / (float)cur_planeDensity) * ((float)ObjectLimitSize[4] / (float)cur_planeDensity)));

	float negPlaneClasterTollerance = cur_planeDensity;
	int negMinPlaneClasterSize = posMinPlaneClasterSize;
	int negMaxPlaneClasterSize = posMaxPlaneClasterSize;*/


	if (enablePlaneFiltration){
		//CloudPlaneFiltration(newlayer.DepthMap, newlayer.DepthMap, DistanceThreshold);
		CloudPlaneFiltration(delta_pos_cloud, delta_pos_cloud, PlaneClasterTollerance, MinPlaneClasterSize, MaxPlaneClasterSize, CloudZStep);
		CloudPlaneFiltration(delta_neg_cloud, delta_neg_cloud, PlaneClasterTollerance, MinPlaneClasterSize, MaxPlaneClasterSize, CloudZStep);
		//CloudPlaneFiltration(delta_neg_cloud, delta_neg_cloud, negPlaneClasterTollerance, negMinPlaneClasterSize, negMaxPlaneClasterSize, CloudZStep);
	}
	
	LayerList[llSize - 1]->layerPositiveDelta.swap(delta_pos_cloud);
	LayerList[llSize - 1]->layerNegativeDelta.swap(delta_neg_cloud);
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
	StorageLayer* new_Layer = new StorageLayer(newlayer);
	
	if (enableVoxelFiltration)
		VoxelGridFiltration(new_Layer->DepthMap, new_Layer->DepthMap, new_Layer->planeDensity);

	if (enableNoizeFiltration)
		CloudNoizeFiltration(new_Layer->DepthMap, new_Layer->DepthMap);

	LayerList.push_back(new_Layer);
	//FIXME
}


void Storage::FindObjectForAdd(int curLayerUID, float valid_percent, int nearestpoinscount, float objectdensity){ //������� ������ ��������, ����������� �� ����� ����
	//int llSize = LayerList.size();
	
	for (int i = 0; i < LayerList[curLayerUID]->PositiveClasterList.size(); i++){
		
		time_t rawtime;
		time(&rawtime);

		StoredObject *test_object = new StoredObject(LayerList[curLayerUID]->UID, UID, (int)rawtime, LayerList[curLayerUID]->PositiveClasterList[i], 1, 180, objectdensity);
		
		//Finding up cover of 2d_object
		test_object->find_bbox();
		
		//Check length and width of 2d_object
		test_object->check_2d_valid_object(ObjectLimitSize, valid_percent);
		
		if (test_object->isValid){
			//Check height in position (center) point
			pcl::PointCloud<pcl::PointXY>::Ptr oldcloud2d(new pcl::PointCloud<pcl::PointXY>);
			Get2DCloudFrom3D(LayerList[curLayerUID - 1]->DepthMap, oldcloud2d);

			pcl::KdTreeFLANN<pcl::PointXY> kdtree;
			kdtree.setInputCloud(oldcloud2d);

			pcl::PointXYZ *testPoint = new pcl::PointXYZ();
			
			//Getting test point with max Z coordinate
			testPoint->x = test_object->position(0);
			testPoint->y = test_object->position(1);
			testPoint->z = test_object->position(2) - 0.5 * test_object->height; //Minus (-) because oZ axis is up side down
			
			//LayerList[curLayerUID]->objectForAddList.push_back(test_object); //DEBUG

			//int oldpointindex;

			//float center_height = GetPointDelatZ(*testPoint, oldcloud2d, LayerList[curLayerUID - 1]->DepthMap, kdtree, oldpointindex);
			float center_height = GetNPointsDelatZ(*testPoint, oldcloud2d, LayerList[curLayerUID - 1]->DepthMap, kdtree, 10);
			std::cout << "Approximate height = " << center_height << std::endl;

			//Less than one object 
			if (center_height < ObjectLimitSize[2]){
				test_object->isValid = false;
				//Find several or one object
			}
			//About one object
			else if (center_height > ObjectLimitSize[2] && center_height < ObjectLimitSize[5]){
				test_object->isValid = true;
				test_object->position(2) += center_height * 0.5 - 0.5 * test_object->height; //Plus (+) because oZ axis is up side down
				test_object->height = center_height;
				LayerList[curLayerUID]->objectForAddList.push_back(test_object);
			}
			//Several objects
			else if (center_height > ObjectLimitSize[5])
			{
				//Manual settings of obj_count or obj_height 
				//int obj_count 
				//std::cin >> obj_count;

				test_object->isGroup = true;
				int obj_count = (float)center_height / (float)ObjectLimitSize[2]; //FIXME. �������� ���������� ����������� ��� ������� ���������� �������� � ������������ ������� ��������� ����� maxz = ObjectLimitSize[5] � minz = ObjectLimitSize[2]
				float zero_level = testPoint->z + center_height; //Plus (+) because oZ axis is up side down
				float obj_height = center_height / obj_count;

				for (int k = 0; k < obj_count; k++){
					time(&rawtime);

					StoredObject *k_object = new StoredObject(*test_object);
					k_object->isValid = true;
					k_object->UID = (int)rawtime;
					k_object->height = obj_height;
					k_object->position(2) = zero_level - obj_height * 0.5 - k * obj_height; //Minus (-) because oZ axis is up side down
					LayerList[curLayerUID]->objectForAddList.push_back(k_object);
				}
			}
		}
	}
}

void Storage::AddNewObject(StoredObject& newobject){ //������� ���������� ������ �������
	StoredObject* newObject = new StoredObject(newobject);
	ObjectList.push_back(newObject);
	//FIXME
}