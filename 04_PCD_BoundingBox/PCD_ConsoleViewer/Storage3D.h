#pragma once
#include <string>
#include <ctime>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace std;
using namespace pcl;

namespace Storage3D{
	//#######################################################################################################
	//Класс описывающий ХРАНИМЫЙ ОБЪЕКТ в форме параллелепипеда
	class Stored3Dobject
	{
	public: Stored3Dobject(int layerID, int xDeltaRes, int yDeltaRes); //Конструктор класса Stored3Dobject

			//-----------------------------------------------------------------------------------------------
			//Поля хранимого объекта

			//Параметры состояния объета
	public: bool removed; //Флаг удаленного объекта

			//Параметры связи объекта с другими сущностями склада
	public: int UID; //Уникальный числовой идентификатор объекта
	public: int StorageUID; //Уникальный идентификатор склада на который добавлен объект
	public: int addedLayerID; //Идентификатор слоя на котором был добавлен объект
	public: int removedLayerID; //идентификатор слоя на котором объект был удален

			//Собственные идентификаторы объекта
	public: string ObjectName; //Не уникальное буквенное обозначение объекта
	public: string ObjectTypeName; //Название типа объекта 
	public: int ObjectType; //Идентификатор типа объекта
	public: time_t AddedData; //Дата добавления объекта на склад
	public: time_t RemovedData; //Дата удаления объекта со склада

			//-----------------------------------------------------------------------------------------------
			//Геометрические параметры объекта
	public: pcl::PointXYZ startPoint; //Координаты центра масс объекта (вычисляемый параметр)
	public: pcl::PointXYZ center; //Координаты центра масс объекта (вычисляемый параметр)
	public: float volume; //Объем объекта (вычисляемый параметр)
	public: Eigen::Matrix3f orientation; //Ориентация объекта относительно системы координат склада (вычисляемый параметр)
	
			//Габаритные размеры
	public: float length; //Длина объекта (вычисляемый параметр)
	public: float weidth; //Ширина объекта (вычисляемый параметр)
	public: float height; //Высота объекта (вычисляемый параметр)
			
			//Параметры относящиеся к карте разницы высот
	public: pcl::PointCloud<pcl::PointXYZ> objectDelta; //Облако точек относящееся к объекту

	public: array<Point3D^, 2>^ objectUpPoints; //Массив типа <Point3D^> хранящий реальные координаты точек относящихся к объекту совпадает с размером массива objectDelta
	public: int XDeltaRes; //Размер карты глубины по oX
	public: int YDeltaRes; //Размер карты гдубины по oY


	public: ArrayList^ objectPointCloud; //Массив типа <Point3D^> хранящий реальные координаты всех точек объекта. Заполняется с помощью функции CreateObjectPointCloud()
			

			//Габаритные размеры цилиндра
			//public: float height; //Высота объекта (вычисляемый параметр)
			//public: float radius; //Радиус объекта (вычисляемый параметр)
			
			//Ключевые точки для определения цилиндра
			//public: Point3D^ Vertex1_UpCenter; //Нижняя центральная точка
			//public: Point3D^ Vertex2_UpRadius; //Точка на верхней окружности
			//public: Point3D^ Vertex3_DownCenter; //Нижняя центральная точка

			//-----------------------------------------------------------------------------------------------
			//Методы хранимого объекта

			//Функция задания 4-х ключевых точек объекта (модель задания объекта для сканирования с любой стороны)
	public: void SetKey4PointsParall(Point3D^ vertex1_UpLeft, Point3D^ vertex2_UpCenter, Point3D^ vertex3_UpRight, Point3D^ vertex4_Up);

	public: void CalcDimentions(); //Функция вычисления основных размеров объекта (ВхШхД)
	private: float CalcMaxDelta(); //Функция поиска максимального приращения Delta высоты после добавления объека
	private: void CalcHeight(int mode); //Функция вычисления высоты объекта
	private: void CalcLengthWeidth(); //Функция вычисления габаритной ширины и длины объекта
	private: void CalcVolume(); //Функция вычисления объема объекта
	private: void CalcCenter(); //Функция вычисления координат центра масс объекта
	private: void CalcOrient(); //Функция вычисления положения объекта относительно горизонтальной плоскости
	public: bool isObjectPoint(System::Drawing::Point^ mapPointForCheck); //Функция для проверки принадлежности точки к точкам объекта

	private: void CreateObjectPointCloud(); //Функция создания облакаточек объекта по его размерам и вычисленному массиву Delta

	public: void RemoveObject(); //Функция удаления объекта со склада
	};





	//#######################################################################################################
	//Класс, хранящий информацию об одном СЛОЕ (сканировании склада)
	class StorageLayer
	{
	public: StorageLayer(); //Конструктор класса StorageLayer

			//-----------------------------------------------------------------------------------------------
			//Поля слоя
	public: int UID; //Уникальный числовой идентификатор слоя
	public: time_t AddedData;
	
	public: pcl::PointCloud<pcl::PointXYZ> layerNegativeDelta; //Массив типа <float> отрицательных разниц высот точек на данном слое по отношению к предыдущему
	public: pcl::PointCloud<pcl::PointXYZ> layerPositiveDelta; //Массив типа <float> положительных разниц высот точек на данном слое по отношению к предыдущему 
	public: pcl::PointCloud<pcl::PointXYZ> realDepthMap; //Карта глубины (в реальных координатах)
	public: ArrayList^ objectIDList; //Массив типа <int> идентификаторов объектов добавленных на слое
	
	public: int XDepthRes;
	public: int YDepthRes;

	public: Bitmap^ RGBMap; //Изображение слоя в момент съемки карты глубины
	public: int XRGBRes;
	public: int YRGBRes;
			//----------------------------------------------------------------------------------------------
			//Методы слоя

	};





	//#######################################################################################################
	//Класс, описывающий модель СКЛАДА с объектами
	class Storage
	{
	public: Storage(); //Конструктор класса Storage

			//-----------------------------------------------------------------------------------------------
			//Поля склада
	public: UInt32^ UID; //Уникальный числовой идентификатор склада
	public: DateTime^ CreatedData; //Дата создания склада

	public: ArrayList^ LayerList; //Список типа <StorageLayer^> слоев добавленных на склад
	public: ArrayList^ ObjectList; //Список типа <Stored3Dobject^> объектов добавленных на склад
	public: ArrayList^ ObjectIDForRemoveList; //Массив типа <int> идентификаторов объектов отмеченных для удаления со склада
	
	public: float Delta3DEpsilon; //Допуск на разницу между высотами слоев после которой значения из layerDelta считаются существенными
	public: float deltaValidPercent; //Процент точек карты глубины с отрицательным значением Delta при котором объект будет удален 

			//----------------------------------------------------------------------------------------------
			//Методы склада
	public: void AddNewObject(Stored3Dobject^ newObject); //Функция добавления нового объекта
	private: void CalcNewObjectDelta(Stored3Dobject^ newObject); //Функция вычисления Delta в реальных координатах для добавленного объекта
	public: void FindObjectForRemove(); //Функция поиска объектов для удаления после добавления нового слоя (запускается при наличии отрицательных значений Delta
	public: void RemoveObject(int objectID); //Функция удаления i-го объекта со склада 
	public: void RemoveFoundObjects(); //Функция удаления списка объектов со склада(выполянется после выполнения FindObjectForRemove())

	public: void AddNewLayer(StorageLayer^ newLayer); //Функция добавления нового слоя
	private: void CalcNewLayerDelta(StorageLayer^ newLayer); //Функция вычисления Delta в реальных координатах для добавленного слоя
	
	};
}
