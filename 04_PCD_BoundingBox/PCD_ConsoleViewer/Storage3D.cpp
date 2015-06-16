#pragma once
#include "Storage3D.h"


//#######################################################################################################
//Class Stored3Dobject
Stored3Dobject::Stored3Dobject(int layerID, ){
	

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
StorageLayer::StorageLayer(){
	
}





//#######################################################################################################
//Class Storage
Storage::Storage(){
	LayerList = new ArrayList(0); //Инициализируем пустой список слоев
	ObjectList = new ArrayList(0); //Инициализируем пустой список объектов
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
