#pragma once
#include "stdafx.h"

namespace Storage3D{


	class StoredObject
	{
		//-----------------------------------------------------------------------------------------------
		//Поля хранимого объекта
	public:
		//Параметры состояния объета
		bool removed; //Флаг удаленного объекта

		//Параметры связи объекта с другими сущностями склада
		int UID; //Уникальный числовой идентификатор объекта
		int StorageUID; //Уникальный идентификатор склада на который добавлен объект
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
		bool check_valid_object();
		bool isValid;

		pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud; //Массив типа хранящий реальные координаты всех точек объекта

		int step_degree; //Step in degree for Searching optimal boundingbox
		int max_degree; //Cout degree where searching optimal boundingbox
		float grid_density; //Count of point cloud grid in x, y, z dimentions

		//Moving to position
		Eigen::Vector3f position;
		Eigen::Quaternionf quaternion_to_bbox;
		Eigen::Affine3f jump_to_bbox;

		//Object orientation to set to zero position
		float roll;
		float pitch;
		float yaw;

		//Moving to zero
		Eigen::Affine3f jump_to_zero;

		//Object size
		float width;
		float lenght;
		float height;

		//Object size limit
		float width_max;
		float lenght_max;
		float height_max;
		float width_min;
		float lenght_min;
		float height_min;

		float volume; //Volume of object

		//-----------------------------------------------------------------------------------------------
		//Методы хранимого объекта
		StoredObject();
		StoredObject(int layerID, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float limit_array[6], int stepdegree = 10, int maxdegree = 90, float griddensity = 5.0f); //Конструктор класса Stored3Dobject
		~StoredObject();

		bool check_isinside_point(const pcl::PointXYZ &check_point);
		void find_bbox(); //Finding BoundingBox
		void Remove();
	};

	//Класс, хранящий информацию об одном СЛОЕ (сканировании склада)
	class StorageLayer
	{
	public:
		//-----------------------------------------------------------------------------------------------
		//Поля слоя
		int UID; //Уникальный числовой идентификатор слоя
		time_t AddedDate; //Время добавления слоя

		pcl::PointCloud<pcl::PointXYZ>::Ptr layerNegativeDelta; //Массив типа <float> отрицательных разниц высот точек на данном слое по отношению к предыдущему
		pcl::PointCloud<pcl::PointXYZ>::Ptr layerPositiveDelta; //Массив типа <float> положительных разниц высот точек на данном слое по отношению к предыдущему 
		pcl::PointCloud<pcl::PointXYZ>::Ptr DepthMap; //Карта глубины (в реальных координатах)
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PositiveClasterList; //Кластеры найденные в облаке положительной дельты
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> NegativeClasterList; //Кластеры найденные в облаке отрицательной дельты

		vector<StoredObject> objectForAddList; //Массив типа <int> идентификаторов объектов добавленных на слое
		vector<StoredObject> objectEraserList; //Массив объектов которые должны поглотить удаляемые объекты

		//----------------------------------------------------------------------------------------------
		//Методы слоя
		StorageLayer(); //Конструктор класса StorageLayer
		~StorageLayer();

		static void StorageLayer::FindClaster(pcl::PointCloud<pcl::PointXYZ>::Ptr deltacloud, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clastervector, float tolerance = 0.3, int minclastersize = 200, int maxclastersize = 25000);
		void FindObjectForAdd(float minx, float miny, float minz, float maxx, float maxy, float maxz); //Функция поиска объектов, добавленных на новом слое
	};
	

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
		float deltaValidPercent; // (0 - 1)Процент точек карты глубины с отрицательным значением Delta при котором объект будет удален 

		//----------------------------------------------------------------------------------------------
		//Методы склада
		Storage(float deltalimit = 0.06f, float deltavalpercnt = 0.7f);
		~Storage();

		void Storage::CalcNewLayerDelta(const pcl::PointCloud<pcl::PointXYZ>::Ptr &oldcloud,
			const pcl::PointCloud<pcl::PointXYZ>::Ptr &newcloud,
			pcl::PointCloud<pcl::PointXYZ>::Ptr &delta_pos_cloud,
			pcl::PointCloud<pcl::PointXYZ>::Ptr &delta_neg_cloud); //Функция вычисления Delta в реальных координатах для добавленного слоя
		void AddNewObject(StoredObject &newObject); //Функция добавления нового объекта
		void FindObjectForRemove(vector<StoredObject> objecteraserlist); //Функция поиска объектов для удаления после добавления нового слоя (запускается при наличии отрицательных значений Delta
		void RemoveObjects(); //Функция удаления всех найденных объектов
		void RemoveObject(int objectID); //Функция удаления i-го объекта со склада 
		void AddNewLayer(StorageLayer newLayer); //Функция добавления нового слоя
	};
}