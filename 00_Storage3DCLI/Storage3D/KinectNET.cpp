#include "KinectNET.h"

using namespace KinectNET;

Kinect::Kinect()
{
	initKinect();
}

//Функция инициализации Кинекта
void Kinect::initKinect(){
	//Создаем основной объект для общения с Кинектом
	configpath = "OpenNIConfig.xml";
	context = Context::CreateFromXmlFile(configpath, scriptNode);

	//Создаем генератор для изображения RGB
	depthGen = gcnew DepthGenerator(context);
	//Создаем генератор для изображения RGB
	imageGen = gcnew ImageGenerator(context);

	mapModeRGB = imageGen->MapOutputMode;
	mapModeDepth = depthGen->MapOutputMode;

	imageMD = gcnew ImageMetaData();
	depthMD = gcnew DepthMetaData();

	cols = imageMD->XRes;
	raws = imageMD->YRes;

	findLocalMaxDepth();
}

//Функция поиска локального максимума карты глубины (для получения нормальной контрастности изображения)
void Kinect::findLocalMaxDepth(){
	int newMaxDepth;
	try
	{
		//Обновляем снимок с кинекта
		context->WaitOneUpdateAll(depthGen);

		imageMD = imageGen->GetMetaData();

		cols = imageMD->XRes;
		raws = imageMD->YRes;
		
	}
	catch (Exception^)
	{
	}
	
	Byte* pDepth = (Byte*)depthGen->DepthMapPtr.ToPointer();

	MaxDepth = 0;

		//Задаем значения каждого пикселя
	for (int y = 0; y < raws; y++) {
		for (int x = 0; x < cols; x++) {
			newMaxDepth = pDepth[y*cols + x];
			if (MaxDepth < newMaxDepth)
				MaxDepth = newMaxDepth;
		}
	}
}

//Функция получения одиночного RGB изображения с кинекта
Bitmap^ Kinect::getRGBImage(){
	Bitmap^ rgbBitmap; //RGB-изображение, готовое для сохранения и отображения на экране
	rgbBitmap = gcnew Bitmap(mapModeRGB.XRes, mapModeRGB.YRes, System::Drawing::Imaging::PixelFormat::Format24bppRgb);

	Rectangle rect = Rectangle(0, 0, rgbBitmap->Width, rgbBitmap->Height);
	BitmapData^ rgbData = rgbBitmap->LockBits(rect, ImageLockMode::WriteOnly, System::Drawing::Imaging::PixelFormat::Format24bppRgb);

	try
	{
		//Обновляем снимок с кинекта
		context->WaitOneUpdateAll(imageGen);
	}
	catch (Exception^)
	{
	}

	imageMD = imageGen->GetMetaData();

	cols = imageMD->XRes;
	raws = imageMD->YRes;

	Monitor::Enter(this);
	Byte* pRGB = (Byte*)imageGen->ImageMapPtr.ToPointer();

	//int byteCount = imageMD->YRes * imageMD->XRes;

	//Задаем значения каждого пикселя
	for (int y = 0; y < raws; y++) {
		Byte* pDest = (Byte*)rgbData->Scan0.ToPointer() + y * rgbData->Stride;

		for (int x = 0; x < cols; x++, pRGB += 3, pDest += 3) {
			pDest[0] = pRGB[2];
			pDest[1] = pRGB[1];
			pDest[2] = pRGB[0];
		}
	}

	rgbBitmap->UnlockBits(rgbData);
	Monitor::Exit(this);

	return rgbBitmap;
}

//Функция получения одиночного Depth изображения с кинекта
Bitmap^ Kinect::getDepthImage(){

	UInt16MapData^ depthMapData; //Карта Depth-данных, из которой можно получить не нормированные значения отдаления точек от кинекта по оси oZ (Карта глубины)

	Bitmap^ depthBitmap; //RGB-изображение, готовое для сохранения и отображения на экране
	depthBitmap = gcnew Bitmap(mapModeDepth.XRes, mapModeDepth.YRes, System::Drawing::Imaging::PixelFormat::Format24bppRgb);

	Rectangle rect = Rectangle(0, 0, depthBitmap->Width, depthBitmap->Height);
	BitmapData^ depthData = depthBitmap->LockBits(rect, ImageLockMode::WriteOnly, System::Drawing::Imaging::PixelFormat::Format24bppRgb);

	//UInt16 MaxDepth; //Максимальная дальность, полученная с кинекта
	float DepthStep; //Шаг изменения цвета для отображения карты глубины в зависимости от максимального значения

	try
	{
		//Обновляем снимок с кинекта
		context->WaitOneUpdateAll(depthGen);

		MaxDepth = depthGen->DeviceMaxDepth;
		DepthStep = (float)255.0 / (float)MaxDepth;
	}
	catch (Exception^)
	{
	}

	depthMD = depthGen->GetMetaData();

	Monitor::Enter(this);

	depthMapData = depthGen->GetDepthMap();

	//int byteCount = imageMD->YRes * imageMD->XRes;
	//Объявляем составляющие цвета пикселя
	UInt16 red, green, blue;

	//Задаем значения каждого пикселя
	for (int y = 0; y < raws; y++) {
		Byte* pDest = (Byte*)depthData->Scan0.ToPointer() + y * depthData->Stride;
		//for (int x = 0; x < cols; x++, pDepth++, pDest += 3) {
		for (int x = 0; x < cols; x++, pDest += 3) {
			int color = (UInt16)(depthMapData[y*cols + x] * DepthStep);

			pDest[0] = color;
			pDest[1] = color;
			pDest[2] = color;

			//pDest[0] = 0;// (UInt16)(depthMapData[y*cols + x] * DepthStep);// ->nBlue;
			//pDest[1] = (UInt16)(depthMapData[y*cols + x] * DepthStep);// ->nGreen;
			//pDest[2] = (UInt16)(depthMapData[y*cols + x] * DepthStep);// ->nRed;

			//pDest[0] = depth;
			//pDest[1] = depth;
			//pDest[2] = depth;
		}
	}

	depthBitmap->UnlockBits(depthData);
	Monitor::Exit(this);

	return depthBitmap;
}

//Функция перевода координат с RGB изображения в координаты реального 3D пространства в мм
Point3D^ Kinect::ScreenToWorldPoint(int screenX, int screenY, int depth){
	float projX, projY, projZ;
	Point3D worldPoint3D, projectivePoint3D;
	projX = screenX;
	projY = screenY;
	projZ = depth;
	projectivePoint3D = Point3D(projX, projY, projZ);

	worldPoint3D = depthGen->ConvertProjectiveToRealWorld(projectivePoint3D);
	return worldPoint3D;
}

//Функция получения карты глубины в реальных координатах пространства
array<Point3D^,2>^ Kinect::getRealPoints3DArray(){
	
	array<Point3D^, 2>^ RealPoints3DArray = gcnew array<Point3D^, 2>(cols, raws);

	UInt16MapData^ depthMapData = depthGen->GetDepthMap();
	
	for (int y = 0; y < raws; y++) {
		for (int x = 0; x < cols; x++) {
			RealPoints3DArray[x,y] = ScreenToWorldPoint(x, y, depthMapData[y*cols + x]);
		}
	}

	return RealPoints3DArray;
}