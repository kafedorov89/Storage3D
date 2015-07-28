// Storage3D2Console.cpp : Defines the entry point for the console application.
//
#pragma warning (disable: 4703)

#pragma once
#include "stdafx.h"
#include "kinect_grabber.h"
#include "storage.h"
#include "Storage3D2Settings.h"
#include "PCLFunctions.h"

#define VK_S           0x53

//#include "Scaner3D.h"

//using namespace pcl;
using namespace std;

int _tmain(int argc, _TCHAR* argv[])
{
	//Loading settings file first time
	loadSettingsFile();

	//KinectGrabber grabber = KinectGrabber(0, false);
	KinectGrab *grabber = new KinectGrab();
	bool inited_from_file = false;
	bool inited_from_db = true;
	//Storage *storage = new Storage(delta_limit, plane_threshold);
	Storage* storage = new Storage(1, "Storage3D.sqlite", "./database", "./database/layers", "./database/objects");

	std::cout << "Loading state from database..." << std::endl;
	
	if (work_with_db){
		storage->initObjectsFromDB(); //Keep that initialize db could be used with correct start state (same state as was saved in last exit)
		storage->initLayersFromDB();
		inited_from_db = true;
	}

	pcl::visualization::PCLVisualizer *delta_viewer = new pcl::visualization::PCLVisualizer("Delta Viewer");
	pcl::visualization::PCLVisualizer *pos_claster_viewer = new pcl::visualization::PCLVisualizer("Positive Claster Viewer"); //DEBUG
	pcl::visualization::PCLVisualizer *neg_claster_viewer = new pcl::visualization::PCLVisualizer("Negative Claster Viewer"); //DEBUG
	
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> LayerFromFile;
	int layerID = 0;

	StorageLayer* oldlayer = new StorageLayer(storage->UID);

	//Loading files instead stream from 3D-scanner
	if (working_with_file){
		std::cout << "Working with files..." << std::endl;
		string dirpath = "./database/experimental_pcd/";
		DIR *dir = opendir(dirpath.c_str());

		if (dir)
		{
			struct dirent *ent;
			int i = 0;
			while ((ent = readdir(dir)) != NULL)
			{
				i++;
				if (((std::string)ent->d_name).find(".pcd") != string::npos)
				{
					std::stringstream ss;
					ss << dirpath << ent->d_name;
					std::cout << ss.str() << std::endl;
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
					pcl::io::loadPCDFile(ss.str(), *cloud);

					LayerFromFile.push_back(cloud);
				}
			}
			inited_from_file = true;
		}
		else
		{
			cout << "Error opening directory!" << endl;
		}

		//Load first oldLayer from file
		oldlayer->DepthMap = LayerFromFile[layerID];
	}
	//or
	//Working with 3D-scanner
	else{
		std::cout << "Working with Kinect..." << std::endl;

		try{
			grabber = new KinectGrab(false);
		}
		catch (std::exception& e){
			std::cout << "Kinect error: " << e.what() << "!" << std::endl;
			return 0;
		}

		//Start grabber
		//FIXME. Here should be initialization for scanner Lase, later
		grabber->start();
		
		if (work_with_db){
			oldlayer->DepthMap = storage->LayerList[storage->LayerList.size() - 1]->DepthMap;
		}
		else{
			//Grab layer
			grabber->getCloud();
			//Load first oldLayer from file
			oldlayer->DepthMap = grabber->PointCloudXYZPtr;
		}	
	}

	//Opening and setting pcl vizualizer's windows
	delta_viewer->setBackgroundColor(0, 0, 0);
	delta_viewer->addCoordinateSystem(1.0);
	delta_viewer->setPosition(960, 0);
	delta_viewer->loadCameraParameters("viewer.ini");
	//delta_viewer->initCameraParameters();

	pos_claster_viewer->setBackgroundColor(0, 0, 0); //DEBUG
	pos_claster_viewer->addCoordinateSystem(1.0);
	pos_claster_viewer->setPosition(960, 600);
	pos_claster_viewer->loadCameraParameters("viewer.ini");
	//claster_viewer->initCameraParameters();

	neg_claster_viewer->setBackgroundColor(0, 0, 0);
	neg_claster_viewer->addCoordinateSystem(1.0);
	neg_claster_viewer->setPosition(0, 600);
	neg_claster_viewer->loadCameraParameters("viewer.ini");

	oldlayer->layerDensity = plane_density;
	storage->AddNewLayer(*oldlayer);

	//Main cycle for listen keys press
	std::cout << std::endl << "Ready for scan layers..." << std::endl;
	bool needshowobjects = true;
	while (true){

		//Showing actual and removed objects
		if (needshowobjects){
			needshowobjects = false;
			if (storage->ObjectList.size()){ //DEBUG
				for (int i = 0; i < storage->ObjectList.size(); i++){
					std::stringstream ss;
					ss << "object_" << i;

					pos_claster_viewer->addCube(storage->ObjectList[i]->position,
						storage->ObjectList[i]->quaternion_to_bbox,
						storage->ObjectList[i]->width,
						storage->ObjectList[i]->lenght,
						storage->ObjectList[i]->height, ss.str());

					if (!storage->ObjectList[i]->removed){
						pos_claster_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f, ss.str());
					}
					else{
						pos_claster_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, ss.str());
					}
				}
			}
			else{
				std::cout << "Storage haven't objects..." << std::endl;
			}
		}
		
		//Make new layer with press Space key
		if (GetKeyState(VK_SPACE) < 0 || inited_from_db){
			inited_from_db = false;
			needshowobjects = true;
			layerID++;

			//Set camera position in viewers windows
			delta_viewer->loadCameraParameters("viewer.ini");
			pos_claster_viewer->loadCameraParameters("viewer.ini"); //DEBUG
			neg_claster_viewer->loadCameraParameters("viewer.ini"); //DEBUG

			//Loading settings file again
			loadSettingsFile();

			std::cout << "Adding new storage layer..." << std::endl;
			delta_viewer->removeAllPointClouds();
			delta_viewer->removeAllShapes();
			pos_claster_viewer->removeAllPointClouds(); //DEBUG
			pos_claster_viewer->removeAllShapes(); //DEBUG
			neg_claster_viewer->removeAllPointClouds(); //DEBUG
			neg_claster_viewer->removeAllShapes(); //DEBUG

			//Grab layer
			StorageLayer* newlayer = new StorageLayer(storage->UID);
			//Grab from file
			if (working_with_file){
				if (layerID > (LayerFromFile.size() - 1))
					break;
				newlayer->DepthMap = LayerFromFile[layerID];
			//or
			//Grab from scanner
			}else{
				grabber->getCloud();
				newlayer->DepthMap = grabber->PointCloudXYZPtr;
			}

			//Add layer to storage
			newlayer->layerDensity = plane_density;
			storage->AddNewLayer(*newlayer);
			
			//Set limits for object size from config.ini
			storage->ObjectLimitSize[0] = object_minx;
			storage->ObjectLimitSize[1] = object_miny;
			storage->ObjectLimitSize[2] = object_minz;
			storage->ObjectLimitSize[3] = object_maxx;
			storage->ObjectLimitSize[4] = object_maxy;
			storage->ObjectLimitSize[5] = object_maxz;

			//Calc layers delta
			std::cout << "Calcutating delta..." << std::endl;
			storage->CalcNewLayerDelta(plane_claster_tolerance, min_plane_claster_size,	max_plane_claster_size, cloud_z_step);
		
			//Show positive delta cloud
			delta_viewer->addPointCloud(storage->LayerList[storage->LayerList.size() - 1]->layerPositiveDelta, "delta_pos_cloud", 0); //DEBUG
			delta_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)0 / (float)255, (float)255 / (float)255, (float)0 / (float)255, "delta_pos_cloud"); //DEBUG

			//Show negative delta cloud
			delta_viewer->addPointCloud(storage->LayerList[storage->LayerList.size() - 1]->layerNegativeDelta, "delta_neg_cloud", 0); //DEBUG
			delta_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)255 / (float)255, (float)0 / (float)255, (float)0 / (float)255, "delta_neg_cloud"); //DEBUG

			//oldlayer->DepthMap.swap(newlayer->DepthMap);

			std::cout << "Finding positive clasters..." << std::endl;
			
			//Extract clasters from positive delta
			float cur_layerDensity = storage->LayerList[storage->LayerList.size() - 1]->layerDensity;
			FindClasters(storage->LayerList[storage->LayerList.size() - 1]->layerPositiveDelta, 
				storage->LayerList[storage->LayerList.size() - 1]->PositiveClasterList, 
				obj_claster_tolerance, 
				obj_minpoints, 
				obj_maxpoints); //DEBUG
			std::cout << "Was found " << storage->LayerList[storage->LayerList.size() - 1]->PositiveClasterList.size() << " positive clasters." << std::endl;
			
			if (storage->LayerList[storage->LayerList.size() - 1]->PositiveClasterList.size() > 0){
				for (int i = 0; i < storage->LayerList[storage->LayerList.size() - 1]->PositiveClasterList.size(); i++){
					std::stringstream ss;
					ss << "positive_claster_" << i;

					pos_claster_viewer->addPointCloud(storage->LayerList[storage->LayerList.size() - 1]->PositiveClasterList[i], ss.str());
					pos_claster_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)(rand() % 255) / (float)255.0, (float)(rand() % 255) / (float)255.0, (float)(rand() % 255) / (float)255.0, ss.str());

					pos_claster_viewer->spinOnce();
					boost::this_thread::sleep(boost::posix_time::microseconds(100));
				}
			}
			else{
				std::cout << "Positive clasters wasn't found..." << std::endl;
			}
			
			std::cout << "Finding negative clasters..." << std::endl;
			//Extract clasters from negative delta
			FindClasters(storage->LayerList[storage->LayerList.size() - 1]->layerNegativeDelta, 
				storage->LayerList[storage->LayerList.size() - 1]->NegativeClasterList, 
				obj_claster_tolerance, 
				obj_minpoints, 
				obj_maxpoints);
			std::cout << "Was found " << storage->LayerList[storage->LayerList.size() - 1]->NegativeClasterList.size() << " negative clasters" << std::endl;

			if (storage->LayerList[storage->LayerList.size() - 1]->NegativeClasterList.size() > 0){
				for (int i = 0; i < storage->LayerList[storage->LayerList.size() - 1]->NegativeClasterList.size(); i++){
					std::stringstream ss;
					ss << "negative_claster_" << i;
					neg_claster_viewer->addPointCloud(storage->LayerList[storage->LayerList.size() - 1]->NegativeClasterList[i], ss.str());

					neg_claster_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)(rand() % 255) / (float)255.0, (float)(rand() % 255) / (float)255.0, (float)(rand() % 255) / (float)255.0, ss.str());

					neg_claster_viewer->spinOnce();
					boost::this_thread::sleep(boost::posix_time::microseconds(100));
				}
			}
			else{
				std::cout << "Negative clasters wasn't found..." << std::endl;
			}

			std::cout << "Finding objects for add..." << std::endl;
			//Find objects
			storage->FindObjects(storage->LayerList.size() - 1, valid_percent, nearest_point_count, object_density);

			std::cout << "Removing objects..." << std::endl;
			//Find removers
			storage->RemoveObjects(storage->LayerList.size() - 1, valid_percent, nearest_point_count, object_density);

			std::cout << "Refreshing visualization..." << std::endl;

			//Showing removers
			if (storage->LayerList[storage->LayerList.size() - 1]->removerList.size() > 0){
				for (int i = 0; i < storage->LayerList[storage->LayerList.size() - 1]->removerList.size(); i++){
					std::stringstream ss;
					ss << "remover_" << i;

					pos_claster_viewer->addCube(storage->LayerList[storage->LayerList.size() - 1]->removerList[i]->position,
						storage->LayerList[storage->LayerList.size() - 1]->removerList[i]->quaternion_to_bbox,
						storage->LayerList[storage->LayerList.size() - 1]->removerList[i]->width,
						storage->LayerList[storage->LayerList.size() - 1]->removerList[i]->lenght,
						storage->LayerList[storage->LayerList.size() - 1]->removerList[i]->height, ss.str());
				}
			}
			else{
				std::cout << "Layer haven't removers..." << std::endl;
			}

			delete newlayer;

			//std::cout << "Saving state to database..." << std::endl;
			//storage->saveStorageToDB(); //DEBUG

			std::cout << std::endl << "Ready for scan layers..." << std::endl;
		}

		if (GetKeyState(VK_S) < 0){
			delta_viewer->saveCameraParameters("viewer.ini");// getCameraFile();
			std::cout << "Camera parameters was saved..." << std::endl;
			boost::this_thread::sleep(boost::posix_time::microseconds(1000));
			//break;
		}

		if (GetKeyState(VK_ESCAPE) < 0){
			break;
		}

		delta_viewer->spinOnce();
		pos_claster_viewer->spinOnce(); //DEBUG
		neg_claster_viewer->spinOnce(); //DEBUG
		boost::this_thread::sleep(boost::posix_time::microseconds(100));
	}

	if (!working_with_file)
	{
		try{
			grabber->stop();
		}
		catch (std::exception& e){
			
		}
	}

	return 0;
}

