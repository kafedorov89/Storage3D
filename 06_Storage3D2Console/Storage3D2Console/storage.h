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

using namespace std;

class Storage
{
	//-----------------------------------------------------------------------------------------------
	//Поля склада
public:
	int UID; //Уникальный числовой идентификатор склада
	time_t CreatedDate; //Дата создания склада

	vector<StorageLayer> LayerList; //Список слоев добавленных на склад
	vector<StoredObject> ObjectList; //Список объектов добавленных на склад
	vector<int> objectIDForRemoveList; //Массив типа <int> идентификаторов объектов отмеченных для удаления со склада

	float deltaLimit; //Допуск на разницу между высотами слоев после которой значения из layerDelta считаются существенными
	//float deltaValidPercent; // (0 - 1)Процент точек карты глубины с отрицательным значением Delta при котором объект будет удален
	float DistanceThreshold;
	float zEpsAngle;
	bool planeFiltration;
	bool perpendicularOnly;
	bool noizeFiltration;

	//----------------------------------------------------------------------------------------------
	//Методы склада
	Storage(int uid,float deltalimit = 0.06f, float planethreshold = 0.01f, float zepsangle = 0.01f, bool planefiltration = false, bool perpendicularonly = false, bool noizefiltration = true);// float deltavalpercnt = 0.7f
	~Storage();

	void Storage::CalcNewLayerDelta(const pcl::PointCloud<pcl::PointXYZ>::Ptr &oldcloud,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr &newcloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr &delta_pos_cloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr &delta_neg_cloud); //Функция вычисления Delta в реальных координатах для добавленного слоя
	void AddNewObject(StoredObject& newObject); //Функция добавления нового объекта
	void FindObjectForRemove(vector<StoredObject> objecteraserlist); //Функция поиска объектов для удаления после добавления нового слоя (запускается при наличии отрицательных значений Delta
	void RemoveObjects(); //Функция удаления всех найденных объектов
	void RemoveObject(int objectID); //Функция удаления i-го объекта со склада 
	void AddNewLayer(StorageLayer& newLayer); //Функция добавления нового слоя
};