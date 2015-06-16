#include "Storage3D.h"
#include "OpenCVtools.h"

using namespace System;
using namespace System::ComponentModel;
using namespace System::Collections;
using namespace System::Windows::Forms;
using namespace System::Data;
using namespace System::Drawing;
using namespace System::Collections::Generic;

using namespace OpenNI;
using namespace OpenTK;
using namespace Storage3D;
//using namespace OpenCVtools;



//#######################################################################################################
//Class Stored3Dobject
Stored3Dobject::Stored3Dobject(int layerID, int xDeltaRes, int yDeltaRes){
	objectDelta = gcnew ArrayList(0);
	objectDelta_map = gcnew ArrayList(0);

	Vertex1_UpLeft = gcnew Point3D();
	Vertex2_UpCenter = gcnew Point3D();
	Vertex3_UpRight = gcnew Point3D();
	Vertex4_Up = gcnew Point3D();
	//Vertex5_DownCenter = gcnew Point3D();


	Vertex1_UpLeft_map = gcnew System::Drawing::Point();
	Vertex2_UpCenter_map = gcnew System::Drawing::Point();
	Vertex3_UpRight_map = gcnew System::Drawing::Point();
	Vertex4_Up_map = gcnew System::Drawing::Point();
	//Vertex5_DownCenter_map = gcnew System::Drawing::Point();
	
	//Vertex1_UpLeft_map->

	addedLayerID = layerID;
	XDeltaRes = xDeltaRes;
	YDeltaRes = yDeltaRes;
}

//Функция задания 4-х ключевых точек объекта (модель задания объекта для указания неточного четырехугольника)
void Stored3Dobject::SetKey4PointsParall(Point3D^ vertex1_UpLeft, Point3D^ vertex2_UpCenter, Point3D^ vertex3_UpRight, Point3D^ vertex4_Up){ //Функция задания 4-х ключевых точек объекта
	Vertex1_UpLeft = vertex1_UpLeft;
	Vertex2_UpCenter = vertex2_UpCenter;
	Vertex3_UpRight = vertex3_UpRight;
	Vertex4_Up = vertex4_Up;
}

void Stored3Dobject::CalcDimentions(){
	CalcHeight(2);
	CalcLengthWeidth();
	CalcVolume();
	CalcCenter();
	CalcOrient();
}

//Функция поиска максимального приращения Delta высоты после добавления объекта
float Stored3Dobject::CalcMaxDelta(){
	float maxdeltabuf = 0;
	int validCount = 0;
	
	/*for (int y = 0; y < YDeltaRes; y++){
		for (int x = 0; x < XDeltaRes; x++){
			if (objectDelta[x,y] > maxdeltabuf)
				maxdeltabuf = objectDelta[x,y];
		}
	}*/

	//Вариант при использовании ArrayList вместо обычного массива
	
	//FIXME. Отфильтровать шум. Возможно нужно брать среднее значение.
	for (int i = 0; i < objectDelta->Count - 1; i++){
		if (isObjectPoint((System::Drawing::Point^)objectDelta_map[i])){
			//if ((float)objectDelta[i] > maxdeltabuf)
			maxdeltabuf += (float)objectDelta[i];
			validCount++;
		}
	}

	if (validCount > 0){
		return (float)maxdeltabuf / (float)validCount;
	}
	else{
		return 0;
	}

	
	/*System::Drawing::Point^ iDeltaPoint

	for (int i = 0; i < objectDelta->Count - 1; i++){
		System::Drawing::Point^ iDeltaPoint = (System::Drawing::Point^)objectDelta_map[i];
		
		objectUpPoints[iDeltaPoint->X, iDeltaPoint->Y];
			[Vertex1_UpLeft_map->X, Vertex1_UpLeft_map->Y];
		Vertex2_UpCenter_map
			Vertex3_UpRight_map
			Vertex4_Up_map
	}*/

}

//Функция вычисления высоты объекта
void Stored3Dobject::CalcHeight(int mode){
	if (mode == 2)
	{
		height = -CalcMaxDelta(); //Высоту нужно вычислять по разнице такой же точки с прошлого слоя. А лучше всего по среднему значению высоты
		//Минус перед функцией потому что сканер перевернут
	}
	else if (mode == 3){
		height = Vector3::Subtract(Vector3(Vertex2_UpCenter->X, Vertex2_UpCenter->Y, Vertex2_UpCenter->Z),
			Vector3(Vertex5_DownCenter->X, Vertex5_DownCenter->Y, Vertex5_DownCenter->Z)).Length;
	}
}

//Функция вычисления габаритной ширины и длины объекта
void Stored3Dobject::CalcLengthWeidth(){
	//FIXME. Переписать функцию на определение габаритных размеров
	
	length = Vector3::Subtract(Vector3(Vertex2_UpCenter->X, Vertex2_UpCenter->Y, Vertex2_UpCenter->Z),
		Vector3(Vertex1_UpLeft->X, Vertex1_UpLeft->Y, Vertex1_UpLeft->Z)).Length;
	weidth = Vector3::Subtract(Vector3(Vertex2_UpCenter->X, Vertex2_UpCenter->Y, Vertex2_UpCenter->Z),
		Vector3(Vertex3_UpRight->X, Vertex3_UpRight->Y, Vertex3_UpRight->Z)).Length;
}

void Stored3Dobject::CalcVolume(){
	//volume = ;
}

void Stored3Dobject::CalcCenter(){
	//center = ;
}

void Stored3Dobject::CalcOrient(){
	//orientation = ;
}

//Функция для проверки принадлежности точки к точкам объекта
bool Stored3Dobject::isObjectPoint(System::Drawing::Point^ mapPointForCheck){
	ArrayList^ objectPolygon = gcnew ArrayList(0);
	
	objectPolygon->Add(Vertex1_UpLeft_map);
	objectPolygon->Add(Vertex2_UpCenter_map);
	objectPolygon->Add(Vertex3_UpRight_map);
	objectPolygon->Add(Vertex4_Up_map);

	return OpenCVtools::isInside(objectPolygon, mapPointForCheck);
}

void Stored3Dobject::CreateObjectPointCloud(){

}

void Stored3Dobject::RemoveObject(){
	removed = true;
	RemovedData = DateTime::Now;
}





//#######################################################################################################
//Class StorageLayer
StorageLayer::StorageLayer(array<Point3D^, 2>^ depthMap, int xDepthRes, int yDepthRes, Bitmap^ rgbMap, int xRGBRes, int yRGBRes){
	objectIDList = gcnew ArrayList(0);
	AddedData = DateTime::Now;
	realDepthMap = depthMap;
	XDepthRes = xDepthRes;
	YDepthRes = yDepthRes;
	
	RGBMap = rgbMap;
	XRGBRes = xRGBRes;
	YRGBRes = yRGBRes;
}





//#######################################################################################################
//Class Storage
Storage::Storage(){
	LayerList = gcnew ArrayList(0); //Инициализируем пустой список слоев
	ObjectList = gcnew ArrayList(0); //Инициализируем пустой список объектов
	Delta3DEpsilon = 20; //Задаем допуск на разницу между высотами слоев после которой значения из layerDelta считаются существенными
	deltaValidPercent = 0.8; //Задаем процент точек карты глубины с отрицательным значением Delta при поиске объектов для удаления
}

//Функция добавления нового объекта
void Storage::AddNewObject(Stored3Dobject^ newObject){
	Stored3Dobject^ bufObject = newObject;

	//Вычисляем массив изменения высот objectDelta
	CalcNewObjectDelta(bufObject);

	//Задаем время добавления объекта на склад
	bufObject->AddedData = DateTime::Now;
	
	//Вычисляем основные размерные характеристики объекта по заданным ключевым точкам
	bufObject->CalcDimentions(); //Пока не используется

	//Записываем информацию об объекте в базу данных
	//SQLite запрос на добавление объекта

	//Добавляем объект на склад
	ObjectList->Add(bufObject);

	//Добавляем в слой информацию о добавляемом объекте 
	int objectLayerID = bufObject->addedLayerID;
	StorageLayer^ layerForEdit = (StorageLayer^)LayerList[objectLayerID]; //FIXME. Найти способ редактирования ArrayList не создавая промежуточный объект
	layerForEdit->objectIDList->Add(ObjectList->Count - 1);
	LayerList->RemoveAt(objectLayerID);
	LayerList->Insert(objectLayerID, layerForEdit);
}

//Функция вычисления Delta в реальных координатах для добавленного объекта
void Storage::CalcNewObjectDelta(Stored3Dobject^ newObject){
	//Находим в списке слоев слой связанный с добавляемым объектом и предыдущий слой
	int objectLayerID = newObject->addedLayerID;
	StorageLayer^ objectLayer = (StorageLayer^)LayerList[objectLayerID];
	StorageLayer^ previousLayer = (StorageLayer^)LayerList[objectLayerID - 1];

	newObject->objectUpPoints = gcnew array<Point3D^, 2>(newObject->XDeltaRes, newObject->YDeltaRes);

	float deltabuf = 0; //Переменная для временного хранения разницы в высотах
	//float realDistance = 0; //Переменная для хранения реального расстояния между точкой нового и старого слоя (отличается от deltabuf только если массив точек не нормализован в пространстве)

	//Вычитаем карту глубины с предыдущего слоя из карты глубины нового слоя (считаем что на слой добавлен только один объект)
	
	for (int y = 0; y < objectLayer->YDepthRes; y++){
		for (int x = 0; x < objectLayer->XDepthRes; x++) {
			deltabuf = objectLayer->realDepthMap[x, y]->Z - previousLayer->realDepthMap[x, y]->Z;
			//FIXME. Для объекта необходимо брать только точки с положительной Delta (сейчас в объекте хранится весь слой)
			//if ((Math::Abs(deltabuf) > Delta3DEpsilon) && (deltabuf < 0)){ //&& (newObject->isObjectPoint(gcnew System::Drawing::Point(x, y)))
			if ((Math::Abs(deltabuf) > Delta3DEpsilon) && (deltabuf < 0) && (newObject->isObjectPoint(gcnew System::Drawing::Point(x, y)))){
				//Добавляем значение Delta и запоминаем точку в которой найдено данное значение
				newObject->objectDelta->Add(deltabuf);
				newObject->objectDelta_map->Add(gcnew System::Drawing::Point(x, y));
			}

			//Запоминаем реальные координаты точки слоя в объекте (в любом случае - то есть в объекте полностью повторяется информация о слое)
			newObject->objectUpPoints[x, y] = objectLayer->realDepthMap[x, y];
		}
	}
}

//Функция поиска объектов для удаления после добавления нового слоя (запускается при наличии отрицательных значений Delta)
void Storage::FindObjectForRemove(){
	StorageLayer^ lastLayer = (StorageLayer^)LayerList[LayerList->Count - 1];
	ObjectIDForRemoveList = gcnew ArrayList(0); //Массив типа <int> идентификаторов объектов отмеченных для удаления со склада
	//Для каждой точки из массива layerNegativeDelta последнего слоя
	//Запускаем проверку ниже лежащих слоев
	
	//Проходим по всем слоям кроме верхнего
	for (int i = LayerList->Count - 2; i >= 0; i--){
		StorageLayer^ iLayer = (StorageLayer^)LayerList[i];
		//Если на слое есть хотя бы один объект
		if (iLayer->objectIDList->Count > 0){
			int kForRemove = false;
			//Проходим по всем k-объектам i-го слоя
			for (int k = 0; k < iLayer->objectIDList->Count; k++){
				//Если k-объект еще не отмечен для удаления
				if (!kForRemove){
					int kObjectIDinObjectList = (int)iLayer->objectIDList[k];
					Stored3Dobject^ kObject = (Stored3Dobject^)ObjectList[kObjectIDinObjectList];
					
					//Если объект не отмечен как удаленный
					if (!kObject->removed){
						//Проходим по всем точкам с отрицательной разницей высот на последнем слое

						//И проверяем точки верхней поверхности
						/*for (int j = 0; j < lastLayer->layerNegativeDelta_map->Count; j++){
							System::Drawing::Point^ jLastLayerPointPos = (System::Drawing::Point^)lastLayer->layerNegativeDelta_map[j]; //Получаем расположение точки (x, y) в массиве realDepthMap последнего слоя

							float lastZ = lastLayer->realDepthMap[jLastLayerPointPos->X, jLastLayerPointPos->Y]->Z;
							float kZ = kObject->objectUpPoints[jLastLayerPointPos->X, jLastLayerPointPos->Y]->Z;

							float kDelta = lastZ - kZ;
							//Если точка на последнем слое находится ниже чем точка объекта
							if ((Math::Abs(kDelta) > Delta3DEpsilon) && (kDelta > 0)){ //Знак наоборот, потому что сканер перевернут
								//Добавляем объект в список для удаления
								ObjectIDForRemoveList->Add(kObjectID);
								//RemoveObject(k);
								kForRemove = true;
								break;
							}
						}*/

						
						int negDeltaCount = lastLayer->layerNegativeDelta_map->Count;
						
						int delataValidStat = 0; //Количество точек, находящихся внутри объекта и имеющих отрицательное значение Delta (за то, чтобы удалить объект)
						
						//Проходим по всем точкам с отрицательной разницей высот на последнем слое
						//И проверяем точки верхней поверхности (вариант с проверкой по ключевым точкам)
						for (int j = 0; j < negDeltaCount; j++){
							System::Drawing::Point^ jMapPointPos = (System::Drawing::Point^)lastLayer->layerNegativeDelta_map[j]; //Получаем расположение точки (x, y) в массиве realDepthMap последнего слоя

							//Если точка находится внутри фигуры обрузуемой объектом (определяется по вершинам верхней поверхности объекта)
							if (kObject->isObjectPoint(jMapPointPos)){
								float lastZ = lastLayer->realDepthMap[jMapPointPos->X, jMapPointPos->Y]->Z;
								float kZ = kObject->objectUpPoints[jMapPointPos->X, jMapPointPos->Y]->Z;

								float kDelta = lastZ - kZ;
								
								//Если точка на последнем слое находится ниже чем точка объекта
								if ((Math::Abs(kDelta) > Delta3DEpsilon) && (kDelta > 0)){ //Знак наоборот, потому что сканер перевернут
									delataValidStat++;
								}
							}
						}

						float realDeltaPercent = (float)delataValidStat / (float)(kObject->objectDelta_map->Count);
						
						//Если процент количества точек выше чем
						if (realDeltaPercent >= deltaValidPercent){
							//Добавляем объект в список для удаления
							ObjectIDForRemoveList->Add(kObjectIDinObjectList);
							kForRemove = true;
						}
					}
				}
				else{ //Если объект уже отмечен для удаления то выходим из цикла проверки точек
					break;
				}
			}
		}
	}

	//Сюда можно будет добавить функцию RemoveFoundObjects() для автоматического удаления добавленных в список объектов
	//RemoveFoundObjects();
}

//Функция удаления отмеченных объектов (выполянется после выполнения FindObjectForRemove())
void Storage::RemoveObject(int objectID){
	Stored3Dobject^ objectForRemove = (Stored3Dobject^)ObjectList[objectID];
	objectForRemove->removed = true;
	objectForRemove->RemovedData = DateTime::Now;
	ObjectList->RemoveAt(objectID);
	ObjectList->Insert(objectID, objectForRemove);
}

void Storage::RemoveFoundObjects(){
	for (int i = 0; i < ObjectIDForRemoveList->Count; i++){
		RemoveObject((int)ObjectIDForRemoveList[i]);
	}
}

//Функция добавления нового слоя
void Storage::AddNewLayer(StorageLayer^ newLayer){
	if(LayerList->Count > 0) CalcNewLayerDelta(newLayer);

	//newLayer->objectIDList = gcnew ArrayList(0);

	LayerList->Add(newLayer);
}

//Функция вычисления Delta в реальных координатах для добавленного слоя
void Storage::CalcNewLayerDelta(StorageLayer^ newLayer){
	StorageLayer^ previousLayer = (StorageLayer^)LayerList[LayerList->Count - 1];
	newLayer->layerPositiveDelta = gcnew ArrayList(0);
	newLayer->layerPositiveDelta_map = gcnew ArrayList(0);
	newLayer->layerNegativeDelta = gcnew ArrayList(0);
	newLayer->layerNegativeDelta_map = gcnew ArrayList(0);


	float deltabuf = 0; //Переменная для временного хранения разницы в высотах

	for (int y = 0; y < newLayer->YDepthRes; y++){
		for (int x = 0; x < newLayer->XDepthRes; x++) {
			//Вычитаем карту глубины предыдущего слоя из карты глубины нового слоя
			deltabuf = newLayer->realDepthMap[x, y]->Z - previousLayer->realDepthMap[x, y]->Z;
			if (Math::Abs(deltabuf) > Delta3DEpsilon){
				//Сохраняем массив Delta в текущем слое
				if (deltabuf < 0){ //Знак наоборот, потому что сканер перевернут
					newLayer->layerPositiveDelta->Add(deltabuf);
					newLayer->layerPositiveDelta_map->Add(gcnew System::Drawing::Point(x, y));
				}
				else{
					newLayer->layerNegativeDelta->Add(deltabuf);
					newLayer->layerNegativeDelta_map->Add(gcnew System::Drawing::Point(x, y));
				}
			}
		}
	}
}
