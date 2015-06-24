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

//------------------------------------------------------------------

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <Eigen/Eigenvalues> 
#include <pcl/visualization/point_cloud_handlers.h>
#include <vtkStructuredPointsReader.h>
#include <vtkBoundingBox.h>
#include <vtkOBBTree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <Eigen/Eigenvalues> 
#include <pcl/visualization/point_cloud_handlers.h>
#include <vtkStructuredPointsReader.h>
#include <vtkBoundingBox.h>
#include <vtkOBBTree.h>
#include <pcl/filters/voxel_grid.h>

#include "PCLFunctions.h"

using namespace std;

class StoredObject
{
	//-----------------------------------------------------------------------------------------------
	//Поля хранимого объекта
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	//Параметры состояния объета
	bool removed; //Флаг удаленного объекта

	//Параметры связи объекта с другими сущностями склада
	int UID; //Уникальный числовой идентификатор объекта
	int storageUID; //Уникальный идентификатор склада на который добавлен объект
	int addedLayerID; //Идентификатор слоя на котором был добавлен объект
	int removedLayerID; //идентификатор слоя на котором объект был удален

	//Собственные идентификаторы объекта
	string ObjectName; //Не уникальное буквенное обозначение объекта
	string ObjectTypeName; //Название типа объекта 
	int ObjectType; //Идентификатор типа объекта
	time_t AddedDate; //Время добавления объекта на склад
	time_t RemovedDate; //Время удаления объекта со склада

	//-----------------------------------------------------------------------------------------------
	//Геометрические параметры объекта
	//Validation object
	
	bool isValid;
	pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud; //Массив хранящий реальные координаты всех точек объекта

	int step_degree; //Step in degree for Searching optimal boundingbox
	int max_degree; //Cout degree where searching optimal boundingbox
	float objectDensity; //Count of point cloud grid in x, y, z dimentions

	//Moving to position
	Eigen::Vector3f position;
	Eigen::Quaternionf quaternion_to_bbox;
	Eigen::Quaternionf quaternion_to_zero;

	Eigen::Affine3f* jump_to_bbox;
	//Moving to zero
	Eigen::Affine3f* jump_to_zero;

	//Object orientation to set to zero position
	float roll;
	float pitch;
	float yaw;

	//Object size
	float width;
	float lenght;
	float height;
	bool isGroup;

	float square;

	//-----------------------------------------------------------------------------------------------
	//Методы хранимого объекта
	StoredObject();
	StoredObject(const StoredObject& storedobject);
	StoredObject(
		int layerid, 
		int storageid, 
		int uid, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, 
		int stepdegree, 
		int maxdegree, 
		float objectdensity,
		string objectname = ""); //Конструктор класса Stored3Dobject
	StoredObject(
		int dbuid,
		string dbname,
		time_t dbadd_date,
		time_t dbremove_date,
		bool dbremoved,
		float dbposition_x,
		float dbposition_y,
		float dbposition_z,
		float dbwidth,
		float dblenght,
		float dbheight,
		float dbroll,
		float dbpitch,
		float dbyaw);

	~StoredObject();

	void check_2d_valid_object(float limit_array[6], float valid_percent);
	//void check_3d_valid_object(float limit_array[6]);
	bool check_isinside_point(const pcl::PointXYZ &check_point);
	void find_bbox(); //Finding BoundingBox
	void Remove();
	void CalcJamp();
};