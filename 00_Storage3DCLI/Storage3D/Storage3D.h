#pragma once
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

using namespace System;
using namespace System::ComponentModel;
using namespace System::Collections;
using namespace System::Windows::Forms;
using namespace System::Data;
using namespace System::Drawing;
using namespace System::Collections::Generic;

using namespace OpenNI;
using namespace OpenTK;
using namespace cv;

namespace Storage3D{
	//#######################################################################################################
	//Класс описывающий ХРАНИМЫЙ ОБЪЕКТ в форме параллелепипеда
	ref class Stored3Dobject
	{
	public: Stored3Dobject(int layerID, int xDeltaRes, int yDeltaRes); //Конструктор класса Stored3Dobject

			//-----------------------------------------------------------------------------------------------
			//Поля хранимого объекта

			//Параметры состояния объета
	public: bool removed; //Флаг удаленного объекта

			//Параметры связи объекта с другими сущностями склада
	public: UInt32^ UID; //Уникальный числовой идентификатор объекта
	public: UInt32^ StorageUID; //Уникальный идентификатор склада на который добавлен объект
	public: int addedLayerID; //Идентификатор слоя на котором был добавлен объект
	public: int removedLayerID; //идентификатор слоя на котором объект был удален

			//Собственные идентификаторы объекта
	public: System::String^ ObjectName; //Не уникальное буквенное обозначение объекта
	public: System::String^ ObjectTypeName; //Название типа объекта 
	public: int ObjectType; //Идентификатор типа объекта
	public: DateTime^ AddedData; //Дата добавления объекта на склад
	public: DateTime^ RemovedData; //Дата удаления объекта со склада

			//-----------------------------------------------------------------------------------------------
			//Геометрические параметры объекта
	public: Point3D^ center; //Координаты центра масс объекта (вычисляемый параметр)
	public: float volume; //Объем объекта (вычисляемый параметр)
	public: Matrix4^ orientation; //Ориентация объекта относительно системы координат склада (вычисляемый параметр)
	
			//Габаритные размеры
	public: float length; //Длина объекта (вычисляемый параметр)
	public: float weidth; //Ширина объекта (вычисляемый параметр)
	public: float height; //Высота объекта (вычисляемый параметр)
			
			//Параметры относящиеся к карте разницы высот
	public: ArrayList^ objectDelta; //Массив типа <float> разниц высот точек после добавления данного объекта по отношению к предыдущему состоянию склада
	public: ArrayList^ objectDelta_map; //Массив <System::Drawing::Point^> связывающий элементы массива 
	//public: ArrayList^ objectUpPoints; //Массив типа <Point3D^> хранящий реальные координаты точек относящихся к объекту совпадает с размером массива objectDelta

	//public: array<float, 2>^ objectDelta; //Массив типа <float> разниц высот точек после добавления данного объекта по отношению к предыдущему состоянию склада
	public: array<Point3D^, 2>^ objectUpPoints; //Массив типа <Point3D^> хранящий реальные координаты точек относящихся к объекту совпадает с размером массива objectDelta
	public: int XDeltaRes; //Размер карты глубины по oX
	public: int YDeltaRes; //Размер карты гдубины по oY


	public: ArrayList^ objectPointCloud; //Массив типа <Point3D^> хранящий реальные координаты всех точек объекта. Заполняется с помощью функции CreateObjectPointCloud()
			
			//Ключевые точки для определения параллелепипеда
	public: Point3D^ Vertex1_UpLeft; //Первая верхняя крайняя вершина
	public: System::Drawing::Point^ Vertex1_UpLeft_map; //Первая верхняя крайняя вершина в массиве карты глубины
	public: Point3D^ Vertex2_UpCenter; //Верхняяя центральная вершина
	public: System::Drawing::Point^ Vertex2_UpCenter_map; //Верхняяя центральная вершина в массиве карты глубины
	public: Point3D^ Vertex3_UpRight; //Вторая вехняя крайняя вершина
	public: System::Drawing::Point^ Vertex3_UpRight_map; //Вторая вехняя крайняя вершина в массиве карты глубины
	public: Point3D^ Vertex4_Up; //Четвертая вехняя вершина
	public: System::Drawing::Point^ Vertex4_Up_map; //Четвертая верхняя вершина в массиве карты глубины
	public: Point3D^ Vertex5_DownCenter; //Нижняя центральная вершина
	public: System::Drawing::Point^ Vertex5_DownCenter_map; //Нижняя центральная вершина в массиве карты глубины

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
	ref class StorageLayer
	{
	public: StorageLayer(array<Point3D^, 2>^ depthMap, int xDepthRes, int yDepthRes, Bitmap^ rgbMap, int xRGBRes, int yRGBRes); //Конструктор класса StorageLayer

			//-----------------------------------------------------------------------------------------------
			//Поля слоя
	public: UInt32^ UID; //Уникальный числовой идентификатор слоя
	public: DateTime^ AddedData;
	
	public: ArrayList^ layerNegativeDelta; //Массив типа <float> отрицательных разниц высот точек на данном слое по отношению к предыдущему
	public: ArrayList^ layerNegativeDelta_map; //Массив типа <Point> хранящий соотношение точек realDepthMap и layerDelta
	public: ArrayList^ layerPositiveDelta; //Массив типа <float> положительных разниц высот точек на данном слое по отношению к предыдущему 
	public: ArrayList^ layerPositiveDelta_map; //Массив типа <Point> хранящий соотношение точек realDepthMap и layerDelta
	public: array<Point3D^, 2>^ realDepthMap; //Карта глубины (в реальных координатах)
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
	ref class Storage
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
