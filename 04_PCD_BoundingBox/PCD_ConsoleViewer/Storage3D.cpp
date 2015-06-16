#pragma once
#include "Storage3D.h"


//#######################################################################################################
//Class Stored3Dobject
Stored3Dobject::Stored3Dobject(int layerID, ){
	

}

//������� ��� �������� �������������� ����� � ������ �������
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
	LayerList = new ArrayList(0); //�������������� ������ ������ �����
	ObjectList = new ArrayList(0); //�������������� ������ ������ ��������
	Delta3DEpsilon = 20; //������ ������ �� ������� ����� �������� ����� ����� ������� �������� �� layerDelta ��������� �������������
	deltaValidPercent = 0.8; //������ ������� ����� ����� ������� � ������������� ��������� Delta ��� ������ �������� ��� ��������
}

//������� ���������� ������ �������
void Storage::AddNewObject(Stored3Dobject^ newObject){
	Stored3Dobject^ bufObject = newObject;

	//��������� ������ ��������� ����� objectDelta
	CalcNewObjectDelta(bufObject);

	//������ ����� ���������� ������� �� �����
	bufObject->AddedData = DateTime::Now;
	
	//��������� �������� ��������� �������������� ������� �� �������� �������� ������
	bufObject->CalcDimentions(); //���� �� ������������

	//���������� ���������� �� ������� � ���� ������
	//SQLite ������ �� ���������� �������

	//��������� ������ �� �����
	ObjectList->Add(bufObject);

	//��������� � ���� ���������� � ����������� ������� 
	int objectLayerID = bufObject->addedLayerID;
	StorageLayer^ layerForEdit = (StorageLayer^)LayerList[objectLayerID]; //FIXME. ����� ������ �������������� ArrayList �� �������� ������������� ������
	layerForEdit->objectIDList->Add(ObjectList->Count - 1);
	LayerList->RemoveAt(objectLayerID);
	LayerList->Insert(objectLayerID, layerForEdit);
}

//������� ���������� Delta � �������� ����������� ��� ������������ �������
void Storage::CalcNewObjectDelta(Stored3Dobject^ newObject){
	//������� � ������ ����� ���� ��������� � ����������� �������� � ���������� ����
	int objectLayerID = newObject->addedLayerID;
	StorageLayer^ objectLayer = (StorageLayer^)LayerList[objectLayerID];
	StorageLayer^ previousLayer = (StorageLayer^)LayerList[objectLayerID - 1];

	newObject->objectUpPoints = gcnew array<Point3D^, 2>(newObject->XDeltaRes, newObject->YDeltaRes);

	float deltabuf = 0; //���������� ��� ���������� �������� ������� � �������
	//float realDistance = 0; //���������� ��� �������� ��������� ���������� ����� ������ ������ � ������� ���� (���������� �� deltabuf ������ ���� ������ ����� �� ������������ � ������������)

	//�������� ����� ������� � ����������� ���� �� ����� ������� ������ ���� (������� ��� �� ���� �������� ������ ���� ������)
	
	for (int y = 0; y < objectLayer->YDepthRes; y++){
		for (int x = 0; x < objectLayer->XDepthRes; x++) {
			deltabuf = objectLayer->realDepthMap[x, y]->Z - previousLayer->realDepthMap[x, y]->Z;
			//FIXME. ��� ������� ���������� ����� ������ ����� � ������������� Delta (������ � ������� �������� ���� ����)
			//if ((Math::Abs(deltabuf) > Delta3DEpsilon) && (deltabuf < 0)){ //&& (newObject->isObjectPoint(gcnew System::Drawing::Point(x, y)))
			if ((Math::Abs(deltabuf) > Delta3DEpsilon) && (deltabuf < 0) && (newObject->isObjectPoint(gcnew System::Drawing::Point(x, y)))){
				//��������� �������� Delta � ���������� ����� � ������� ������� ������ ��������
				newObject->objectDelta->Add(deltabuf);
				newObject->objectDelta_map->Add(gcnew System::Drawing::Point(x, y));
			}

			//���������� �������� ���������� ����� ���� � ������� (� ����� ������ - �� ���� � ������� ��������� ����������� ���������� � ����)
			newObject->objectUpPoints[x, y] = objectLayer->realDepthMap[x, y];
		}
	}
}

//������� ������ �������� ��� �������� ����� ���������� ������ ���� (����������� ��� ������� ������������� �������� Delta)
void Storage::FindObjectForRemove(){
	StorageLayer^ lastLayer = (StorageLayer^)LayerList[LayerList->Count - 1];
	ObjectIDForRemoveList = gcnew ArrayList(0); //������ ���� <int> ��������������� �������� ���������� ��� �������� �� ������
	//��� ������ ����� �� ������� layerNegativeDelta ���������� ����
	//��������� �������� ���� ������� �����
	
	//�������� �� ���� ����� ����� ��������
	for (int i = LayerList->Count - 2; i >= 0; i--){
		StorageLayer^ iLayer = (StorageLayer^)LayerList[i];
		//���� �� ���� ���� ���� �� ���� ������
		if (iLayer->objectIDList->Count > 0){
			int kForRemove = false;
			//�������� �� ���� k-�������� i-�� ����
			for (int k = 0; k < iLayer->objectIDList->Count; k++){
				//���� k-������ ��� �� ������� ��� ��������
				if (!kForRemove){
					int kObjectIDinObjectList = (int)iLayer->objectIDList[k];
					Stored3Dobject^ kObject = (Stored3Dobject^)ObjectList[kObjectIDinObjectList];
					
					//���� ������ �� ������� ��� ���������
					if (!kObject->removed){
						
						int negDeltaCount = lastLayer->layerNegativeDelta_map->Count;
						
						int delataValidStat = 0; //���������� �����, ����������� ������ ������� � ������� ������������� �������� Delta (�� ��, ����� ������� ������)
						
						//�������� �� ���� ������ � ������������� �������� ����� �� ��������� ����
						//� ��������� ����� ������� ����������� (������� � ��������� �� �������� ������)
						for (int j = 0; j < negDeltaCount; j++){
							System::Drawing::Point^ jMapPointPos = (System::Drawing::Point^)lastLayer->layerNegativeDelta_map[j]; //�������� ������������ ����� (x, y) � ������� realDepthMap ���������� ����

							//���� ����� ��������� ������ ������ ���������� �������� (������������ �� �������� ������� ����������� �������)
							if (kObject->isObjectPoint(jMapPointPos)){
								float lastZ = lastLayer->realDepthMap[jMapPointPos->X, jMapPointPos->Y]->Z;
								float kZ = kObject->objectUpPoints[jMapPointPos->X, jMapPointPos->Y]->Z;

								float kDelta = lastZ - kZ;
								
								//���� ����� �� ��������� ���� ��������� ���� ��� ����� �������
								if ((Math::Abs(kDelta) > Delta3DEpsilon) && (kDelta > 0)){ //���� ��������, ������ ��� ������ ����������
									delataValidStat++;
								}
							}
						}

						float realDeltaPercent = (float)delataValidStat / (float)(kObject->objectDelta_map->Count);
						
						//���� ������� ���������� ����� ���� ���
						if (realDeltaPercent >= deltaValidPercent){
							//��������� ������ � ������ ��� ��������
							ObjectIDForRemoveList->Add(kObjectIDinObjectList);
							kForRemove = true;
						}
					}
				}
				else{ //���� ������ ��� ������� ��� �������� �� ������� �� ����� �������� �����
					break;
				}
			}
		}
	}

	//���� ����� ����� �������� ������� RemoveFoundObjects() ��� ��������������� �������� ����������� � ������ ��������
	//RemoveFoundObjects();
}

//������� �������� ���������� �������� (����������� ����� ���������� FindObjectForRemove())
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

//������� ���������� ������ ����
void Storage::AddNewLayer(StorageLayer^ newLayer){
	if(LayerList->Count > 0) CalcNewLayerDelta(newLayer);

	//newLayer->objectIDList = gcnew ArrayList(0);

	LayerList->Add(newLayer);
}

//������� ���������� Delta � �������� ����������� ��� ������������ ����
void Storage::CalcNewLayerDelta(StorageLayer^ newLayer){
	StorageLayer^ previousLayer = (StorageLayer^)LayerList[LayerList->Count - 1];
	newLayer->layerPositiveDelta = gcnew ArrayList(0);
	newLayer->layerPositiveDelta_map = gcnew ArrayList(0);
	newLayer->layerNegativeDelta = gcnew ArrayList(0);
	newLayer->layerNegativeDelta_map = gcnew ArrayList(0);


	float deltabuf = 0; //���������� ��� ���������� �������� ������� � �������

	for (int y = 0; y < newLayer->YDepthRes; y++){
		for (int x = 0; x < newLayer->XDepthRes; x++) {
			//�������� ����� ������� ����������� ���� �� ����� ������� ������ ����
			deltabuf = newLayer->realDepthMap[x, y]->Z - previousLayer->realDepthMap[x, y]->Z;
			if (Math::Abs(deltabuf) > Delta3DEpsilon){
				//��������� ������ Delta � ������� ����
				if (deltabuf < 0){ //���� ��������, ������ ��� ������ ����������
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
