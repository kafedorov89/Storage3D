#pragma once
#include "stdafx.h"
#include "storage.h"

//using namespace pcl;
using namespace std;

Storage::Storage(int uid,
	float deltalimit,
	bool enablevoxelfiltration,
	bool enableplanefiltration,
	bool enablenoizefiltration, 
	char* dbname)
{
	UID = uid;
	deltaLimit = deltalimit;
	enableVoxelFiltration = enablevoxelfiltration;
	enablePlaneFiltration = enableplanefiltration;
	enableNoizeFiltration = enablenoizefiltration;
	*dbName = *dbname;
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
	

	/*float min_mult = 1; //FIXME. Add flexible settings for density and points count
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

void Storage::RemoveObjects(int curLayerUID, float valid_percent, int nearestpoinscount, float objectdensity){ //Функция поиска объектов для удаления после добавления нового слоя (запускается при наличии отрицательных значений Delta
	//int llSize = LayerList.size();

	if (LayerList[curLayerUID]->NegativeClasterList.size() > 0){
		for (int i = 0; i < LayerList[curLayerUID]->NegativeClasterList.size(); i++){

			time_t rawtime;
			time(&rawtime);

			StoredObject *test_remover = new StoredObject(LayerList[curLayerUID]->UID, UID, (int)rawtime, LayerList[curLayerUID]->NegativeClasterList[i], 1, 180, objectdensity);

			//Finding up cover of 2d_object
			test_remover->find_bbox();

			//Check length and width of 2d_object
			test_remover->check_2d_valid_object(ObjectLimitSize, valid_percent);

			if (test_remover->isValid){
				//Check height in position (center) point
				pcl::PointCloud<pcl::PointXY>::Ptr oldcloud2d(new pcl::PointCloud<pcl::PointXY>);
				Get2DCloudFrom3D(LayerList[curLayerUID - 1]->DepthMap, oldcloud2d);

				pcl::KdTreeFLANN<pcl::PointXY> kdtree;
				kdtree.setInputCloud(oldcloud2d);

				pcl::PointXYZ *testPoint = new pcl::PointXYZ();

				//Getting test point with max Z coordinate
				testPoint->x = test_remover->position(0);
				testPoint->y = test_remover->position(1);
				testPoint->z = test_remover->position(2) + 0.5 * test_remover->height; //Plus (+) because oZ axis is up side down

				float center_height = -GetNPointsDelatZ(*testPoint, oldcloud2d, LayerList[curLayerUID - 1]->DepthMap, kdtree, 10); //Minus (-) because oZ axis is up side down
				std::cout << "Remover's approximate height = " << center_height << std::endl;
				
				test_remover->position(2) = testPoint->z - 0.5f * center_height; //Minus (-) because oZ axis is up side down
				
				test_remover->height = center_height;

				test_remover->CalcJamp();

				LayerList[curLayerUID]->removerList.push_back(test_remover);
			}
		}

		//Applying founded removers
		if (LayerList[curLayerUID]->removerList.size() > 0)
		{
			std::cout << LayerList[curLayerUID]->removerList.size() << " removers was found." << std::endl;
			std::cout << "Removing objects..." << std::endl;
			//Add founded objects
			for (int i = 0; i < LayerList[curLayerUID]->removerList.size(); i++){
				for (int k = 0; k < ObjectList.size(); k++){
					if (LayerList[curLayerUID]->removerList[i]->check_isinside_point(pcl::PointXYZ(ObjectList[k]->position(0), ObjectList[k]->position(1), ObjectList[k]->position(2)))){
						RemoveObject(k);
						std::cout << i + 1 << " object was removed" << std::endl;
					}
				}
			}
		}
		else{
			std::cout << "Objects for remove wasn't founded..." << std::endl;
		}
	}
	else{
		std::cout << "Objects for remove wasn't founded..." << std::endl;
	}
}

void Storage::RemoveObject(int objectID){ //Функция удаления i-го объекта со склада 
	ObjectList[objectID]->Remove();
}

void Storage::AddNewLayer(StorageLayer& newlayer){ //Функция добавления нового слоя
	StorageLayer* new_Layer = new StorageLayer(newlayer);
	
	if (enableVoxelFiltration)
		VoxelGridFiltration(new_Layer->DepthMap, new_Layer->DepthMap, new_Layer->planeDensity);

	if (enableNoizeFiltration)
		CloudNoizeFiltration(new_Layer->DepthMap, new_Layer->DepthMap);

	LayerList.push_back(new_Layer);
	//FIXME
}


void Storage::FindObjects(int curLayerUID, float valid_percent, int nearestpoinscount, float objectdensity){ //Функция поиска объектов, добавленных на новом слое
	//int llSize = LayerList.size();
	
	if (LayerList[curLayerUID]->PositiveClasterList.size() > 0){
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
				testPoint->z = test_object->position(2) - 0.5f * test_object->height; //Minus (-) because oZ axis is up side down

				//LayerList[curLayerUID]->objectForAddList.push_back(test_object); //DEBUG

				//int oldpointindex;

				//float center_height = GetPointDelatZ(*testPoint, oldcloud2d, LayerList[curLayerUID - 1]->DepthMap, kdtree, oldpointindex);
				float center_height = GetNPointsDelatZ(*testPoint, oldcloud2d, LayerList[curLayerUID - 1]->DepthMap, kdtree, 10);
				std::cout << "Object's approximate height = " << center_height << std::endl;

				//Less than one object 
				if (center_height < ObjectLimitSize[2]){
					test_object->isValid = false;
					//Find several or one object
				}
				//About one object
				else if (center_height > ObjectLimitSize[2] && center_height < ObjectLimitSize[5]){
					test_object->isValid = true;
					test_object->position(2) = testPoint->z + 0.5f * center_height; //Plus (+) because oZ axis is up side down
					test_object->height = center_height;

					test_object->CalcJamp();

					LayerList[curLayerUID]->objectForAddList.push_back(test_object);
				}
				//Several objects
				else if (center_height > ObjectLimitSize[5])
				{
					//Manual settings of obj_count or obj_height 
					//int obj_count 
					//std::cin >> obj_count;

					test_object->isGroup = true;
					int obj_count = (float)center_height / (float)ObjectLimitSize[2]; //FIXME. Возможна набегающая погрешность при большом количестве объектов и одновременно большом диапозоне между maxz = ObjectLimitSize[5] и minz = ObjectLimitSize[2]
					float zero_level = testPoint->z + center_height; //Plus (+) because oZ axis is up side down
					float obj_height = center_height / obj_count;

					for (int k = 0; k < obj_count; k++){
						time(&rawtime);

						StoredObject *k_object = new StoredObject(*test_object);
						k_object->isValid = true;
						k_object->UID = (int)rawtime;
						k_object->height = obj_height;
						k_object->position(2) = zero_level - obj_height * 0.5 - k * obj_height; //Minus (-) because oZ axis is up side down

						k_object->CalcJamp();

						LayerList[curLayerUID]->objectForAddList.push_back(k_object);
					}
				}
			}

			if (LayerList[curLayerUID]->objectForAddList.size() > 0)
			{
				std::cout << LayerList[curLayerUID]->objectForAddList.size() << " new objects was found." << std::endl;
				std::cout << "Adding founded objects..." << std::endl;
				//Add founded objects
				for (int i = 0; i < LayerList[curLayerUID]->objectForAddList.size(); i++){
					AddNewObject(*LayerList[curLayerUID]->objectForAddList[i]);
					std::cout << i + 1 << " object was added" << std::endl;

				}
			}
			else{
				std::cout << "Objects wasn't founded..." << std::endl;
			}
		}
	}
	else{
		std::cout << "Objects wasn't founded..." << std::endl;
	}
}

void Storage::AddNewObject(StoredObject& newobject){ //Функция добавления нового объекта
	StoredObject* newObject = new StoredObject(newobject);
	ObjectList.push_back(newObject);
	//FIXME
}

void Storage::initStorageFromDB(){
	DataBase = new SQLiteDatabase(dbName);
	char* select_query = "SELECT * FROM object WHERE removed = 'FALSE';";

	//Select fields of all actual objects from DB
	vector<vector<string>> result = DataBase->query(select_query);
	
	//Create objects with received fields and add to storage's ObjectList
	for (vector<vector<string>>::iterator it = result.begin(); it < result.end(); ++it)
	{
		vector<string> row = *it;

		int dbuid = std::atoi(row.at(0).c_str());
		
		string dbname = row.at(1);
		
		struct tm tmadd;
		char* addedtimestr = (char *)row.at(2).c_str();
		std::strftime(addedtimestr, sizeof(addedtimestr), "%F %T", &tmadd);
		time_t dbadd_date = mktime(&tmadd);  // t is now your desired time_t

		struct tm tmrem;
		char* addedtimestr = (char *)row.at(3).c_str();
		std::strftime(addedtimestr, sizeof(addedtimestr), "%F %T", &tmrem);
		time_t dbremove_date = mktime(&tmrem);  // t is now your desired time_t
		
		bool dbremoved = str_to_bool(row.at(5));

		float dbposition_x = std::atof(row.at(5).c_str());
		float dbposition_y = std::atof(row.at(6).c_str());
		float dbposition_z = std::atof(row.at(7).c_str());

		float dbwidth = std::atof(row.at(8).c_str());
		float dblenght = std::atof(row.at(9).c_str());
		float dbheight = std::atof(row.at(10).c_str());

		float dbroll = std::atof(row.at(11).c_str());
		float dbpitch = std::atof(row.at(12).c_str());
		float dbyaw = std::atof(row.at(13).c_str());

		AddNewObject(*(new StoredObject(
			dbuid, 
			dbname, 
			dbadd_date, 
			dbremove_date, 
			dbremoved, 
			dbposition_x,
			dbposition_y,
			dbposition_z,
			dbwidth,
			dblenght,
			dbheight,
			dbroll, 
			dbpitch, 
			dbyaw)));
	}
	DataBase->close();
}

void Storage::saveStorageToDB(){
	//For each object in current state of storage checking for existing infornation in database and update this.
	
	for (int i = 0; Object)

	/*UID = dbuid;
	ObjectName = dbname;

	AddedDate = dbadd_date;
	RemovedDate = dbremove_date;
	removed = dbremoved;

	width = dbwidth;
	lenght = dblenght;
	height = dbheight;

	roll = dbroll;
	pitch = dbpitch;
	yaw = dbyaw;

	position(0) = dbposition_x;
	position(1) = dbposition_y;
	position(2) = dbposition_z;*/
}

vector<int> Storage::GetAllObjects(){

}

vector<int> Storage::GetActualObjects(){

}

vector<int> Storage::GetRemovedObjects(){

}

vector<int> Storage::GetObjectsByPointXYZ(pcl::PointXYZ& testpoint){

}

vector<int> Storage::GetObjectsAddedInTimeInterval(time_t starttime, time_t endtime){

}