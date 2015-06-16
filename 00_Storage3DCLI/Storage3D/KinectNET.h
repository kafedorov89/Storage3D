#pragma once

using namespace System;
using namespace System::ComponentModel;
using namespace System::Collections;
using namespace System::Windows::Forms;
using namespace System::Data;
using namespace System::Drawing;
using namespace System::Collections::Generic;

using namespace System::Drawing::Imaging;
using namespace System::Threading;

using namespace OpenTK;
using namespace OpenNI;


namespace KinectNET{
	ref class Kinect
	{

	public: Kinect();

			//Параметры для работы с кинектом
	public: ImageGenerator^ imageGen; //Генратор RGB-данных
	public: DepthGenerator^ depthGen; //Генратор Depth-данных

	public: ImageMetaData^ imageMD; //Переменная для хранения метаданных полученного RGB-изображения
	public: DepthMetaData^ depthMD; //Переменная для хранения метаданных полученного Depth-изображения

			//Основные параметры
	public: System::String^ configpath; //Путь к файлу конфигурации для OpenNI бибилотеки
	public: Context^ context; //Основной объект для работы с кинектом через библиотеку OpenNI
	public: ScriptNode^ scriptNode; //Да кто ж его знает куда это нужно, но убирать нельзя

	public: int cols; //Ширина изображения (X)
	public: int raws; //Высота изображения (Y)

			//Параметры для получения RGB-изображения
	public: MapOutputMode mapModeRGB;
	public: Bitmap^ rgbBitmap; //RGB-изображение, готовое для сохранения и отображения на экране

			//Параметры для получения Depth-изображения
	public: MapOutputMode mapModeDepth;
	public: Bitmap^ depthBitmap; //Depth-изображение, готовое для сохранения и отображения на экране
	public: int MaxDepth;

	public: void initKinect(); //Функция инициализации Кинекта 
	public: void findLocalMaxDepth(); //Функция поиска локального максимума карты глубины (для получения нормальной контрастности изображения)
	public: Bitmap^ getRGBImage(); //Функция получения одиночного RGB изображения с кинекта
	public: Bitmap^ getDepthImage(); //Функция получения одиночного Depth изображения с кинекта
	public: Point3D^ ScreenToWorldPoint(int screenX, int screenY, int depth); //Функция перевода координат с RGB изображения в координаты реального 3D пространства в мм
	public: array<Point3D^,2>^ getRealPoints3DArray(); //Функция получения карты глубины в реальных координатах пространства
	};
}