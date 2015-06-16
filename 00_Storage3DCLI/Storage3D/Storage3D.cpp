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

//������� ������� 4-� �������� ����� ������� (������ ������� ������� ��� �������� ��������� ����������������)
void Stored3Dobject::SetKey4PointsParall(Point3D^ vertex1_UpLeft, Point3D^ vertex2_UpCenter, Point3D^ vertex3_UpRight, Point3D^ vertex4_Up){ //������� ������� 4-� �������� ����� �������
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

//������� ������ ������������� ���������� Delta ������ ����� ���������� �������
float Stored3Dobject::CalcMaxDelta(){
	float maxdeltabuf = 0;
	int validCount = 0;
	
	/*for (int y = 0; y < YDeltaRes; y++){
		for (int x = 0; x < XDeltaRes; x++){
			if (objectDelta[x,y] > maxdeltabuf)
				maxdeltabuf = objectDelta[x,y];
		}
	}*/

	//������� ��� ������������� ArrayList ������ �������� �������
	
	//FIXME. ������������� ���. �������� ����� ����� ������� ��������.
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

//������� ���������� ������ �������
void Stored3Dobject::CalcHeight(int mode){
	if (mode == 2)
	{
		height = -CalcMaxDelta(); //������ ����� ��������� �� ������� ����� �� ����� � �������� ����. � ����� ����� �� �������� �������� ������
		//����� ����� �������� ������ ��� ������ ����������
	}
	else if (mode == 3){
		height = Vector3::Subtract(Vector3(Vertex2_UpCenter->X, Vertex2_UpCenter->Y, Vertex2_UpCenter->Z),
			Vector3(Vertex5_DownCenter->X, Vertex5_DownCenter->Y, Vertex5_DownCenter->Z)).Length;
	}
}

//������� ���������� ���������� ������ � ����� �������
void Stored3Dobject::CalcLengthWeidth(){
	//FIXME. ���������� ������� �� ����������� ���������� ��������
	
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
	LayerList = gcnew ArrayList(0); //�������������� ������ ������ �����
	ObjectList = gcnew ArrayList(0); //�������������� ������ ������ ��������
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
						//�������� �� ���� ������ � ������������� �������� ����� �� ��������� ����

						//� ��������� ����� ������� �����������
						/*for (int j = 0; j < lastLayer->layerNegativeDelta_map->Count; j++){
							System::Drawing::Point^ jLastLayerPointPos = (System::Drawing::Point^)lastLayer->layerNegativeDelta_map[j]; //�������� ������������ ����� (x, y) � ������� realDepthMap ���������� ����

							float lastZ = lastLayer->realDepthMap[jLastLayerPointPos->X, jLastLayerPointPos->Y]->Z;
							float kZ = kObject->objectUpPoints[jLastLayerPointPos->X, jLastLayerPointPos->Y]->Z;

							float kDelta = lastZ - kZ;
							//���� ����� �� ��������� ���� ��������� ���� ��� ����� �������
							if ((Math::Abs(kDelta) > Delta3DEpsilon) && (kDelta > 0)){ //���� ��������, ������ ��� ������ ����������
								//��������� ������ � ������ ��� ��������
								ObjectIDForRemoveList->Add(kObjectID);
								//RemoveObject(k);
								kForRemove = true;
								break;
							}
						}*/

						
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
