#pragma once
#include "stdafx.h"
#include <iostream>
#include <sys/types.h> 
#include <dirent.h>
#include <regex>
#include <vector>
#include <string>
#include <limits> 

#define NOMINMAX
#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif

#include <Eigen/Eigenvalues>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/search/search.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

//------------------------------------------------------------------

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/transforms.h>

#include <vtkStructuredPointsReader.h>
#include <vtkBoundingBox.h>
#include <vtkOBBTree.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/point_cloud.h>

#include "stored_object.h"
#include "storage_layer.h" 
#include "PCLFunctions.h"

#include <iostream>
#include <time.h>

#include "sqlite/SQLiteDatabase.h"
//#include <ctime>
//#include <time.h>

using namespace std;

class Storage
{
	//-----------------------------------------------------------------------------------------------
	//Поля склада
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	int UID; //Уникальный числовой идентификатор склада
	time_t CreatedDate; //Дата создания склада

	vector<StorageLayer*> LayerList; //Список слоев добавленных на склад
	vector<StoredObject*> ObjectList; //Список объектов добавленных на склад
	//vector<int> objectIDForRemoveList; //Массив типа <int> идентификаторов объектов отмеченных для удаления со склада

	float deltaLimit; //Допуск на разницу между высотами слоев после которой значения из layerDelta считаются существенными
	//float deltaValidPercent; // (0 - 1)Процент точек карты глубины с отрицательным значением Delta при котором объект будет удален
	bool enablePlaneFiltration; 
	bool enableNoizeFiltration;
	bool enableVoxelFiltration;
	char* dbName;

	float PlaneClasterTollerance;
	int MinPlaneClasterSize;
	int MaxPlaneClasterSize;
	float CloudZStep;
	SQLiteDatabase* DataBase;

	float ObjectLimitSize[6];

	//----------------------------------------------------------------------------------------------
	//Методы склада

	Storage(int uid,
		float deltalimit,
		bool enablevoxelfiltration,
		bool enableplanefiltration,
		bool enablenoizefiltration,
		char* dbname);
	~Storage();

	//Функция вычисления Delta в реальных координатах для добавленного слоя
	void Storage::CalcNewLayerDelta(float PlaneClasterTollerance, int MinPlaneClasterSize, int MaxPlaneClasterSize, float CloudZStep); 
	
	//Функция добавления i-го объекта из базы данных
	void AddObjectFromDatabase(StoredObject& newobject);
	
	//Функция добавления нового объекта
	void AddNewObject(StoredObject& newobject); 
	
	//Функция удаления всех найденных объектов
	void RemoveObjects(int curLayerUID, float valid_percent, int nearestpoinscount, float object_density); 
	
	//Функция удаления i-го объекта со склада 
	void RemoveObject(int objectID); 
	
	//Функция поиска объектов, добавленных на новом слое
	void FindObjects(int LayerUID, float valid_percent, int nearestpoinscount, float object_density); 
	
	//Функция добавления нового слоя
	void AddNewLayer(StorageLayer& newLayer); 
	
	//Функция инициализации склада из базы данных
	void initStorageFromDB();

	//void saveStorageToDB();
	/*vector<int> GetAllObjects();
	vector<int> GetActualObjects();
	vector<int> GetRemovedObjects();
	vector<int> GetObjectsByPointXYZ(pcl::PointXYZ& testpoint);
	vector<int> GetObjectsAddedInTimeInterval(time_t starttime, time_t endtime);*/
};