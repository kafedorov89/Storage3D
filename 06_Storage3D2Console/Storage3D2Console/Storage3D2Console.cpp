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
	//Storage *storage = new Storage(delta_limit, plane_threshold);
	Storage* storage = new Storage(1, delta_limit, enable_voxelgridfiltration, enable_planefiltration, enable_noizefiltration, "Storage3D.sqlite");

	std::cout << "Loading state from database..." << std::endl;
	if (work_with_db){
		storage->initStorageFromDB();
	}

	pcl::visualization::PCLVisualizer *delta_viewer = new pcl::visualization::PCLVisualizer("Delta Viewer");
	pcl::visualization::PCLVisualizer *pos_claster_viewer = new pcl::visualization::PCLVisualizer("Positive Claster Viewer"); //DEBUG
	pcl::visualization::PCLVisualizer *neg_claster_viewer = new pcl::visualization::PCLVisualizer("Negative Claster Viewer"); //DEBUG
	
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> LayerFromFile;
	int layerUID = 0;

	StorageLayer* oldlayer = new StorageLayer(layerUID, storage->UID);

	//Loading files instead stream from 3D-scanner
	if (working_with_file){
		std::cout << "Working with files..." << std::endl;
		string dirpath = "./pcd/experimental_pcd/";
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
					//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = new pcl::PointCloud<pcl::PointXYZ>();
					pcl::io::loadPCDFile(ss.str(), *cloud);

					LayerFromFile.push_back(cloud);
				}
			}
		}
		else
		{
			cout << "Error opening directory!" << endl;
		}

		//Save first layer
		oldlayer->DepthMap = LayerFromFile[layerUID];
	}
	else{
		std::cout << "Working with Kinect..." << std::endl;

		try{
			grabber = new KinectGrab(false);
		}
		catch (std::exception& e){
			std::cout << "Kinect error: " << e.what() << "!" << std::endl;
			//cin.get();
			return 0;
		}

		//Try to find previous saved state of Storage
		string dirpath = "./database/";
		DIR *dir = opendir(dirpath.c_str());

		if (dir)
		{
			struct dirent *ent;
			int i = 0;
			while ((ent = readdir(dir)) != NULL)
			{
				i++;
				if (((std::string)ent->d_name).find(oldlayer->last_file_name) != string::npos)
				{
					std::stringstream ss;
					ss << dirpath << ent->d_name;
					std::cout << ss.str() << std::endl;
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
					//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = new pcl::PointCloud<pcl::PointXYZ>();
					pcl::io::loadPCDFile(ss.str(), *cloud);

					oldlayer->DepthMap = cloud;
					inited_from_file = true;
				}
			}
		}
		else
		{
			cout << "Error opening directory!" << endl;
		}
		
		if (!inited_from_file){

			//Grab layer
			grabber->start();
			grabber->getCloud();

			//Save first layer
			oldlayer->DepthMap = grabber->PointCloudXYZPtr; //FIXME. Add initialization from file or from last saved layer's DepthMap
			oldlayer->SaveLayerToPCD(true, save_only_last);
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

	oldlayer->planeDensity = plane_density;
	storage->AddNewLayer(*oldlayer);

	//Main cycle for listen keys press
	std::cout << std::endl << "Ready for scan layers..." << std::endl;
	while (true){
		if (GetKeyState(VK_SPACE) < 0 || inited_from_file){
			inited_from_file = false;
			layerUID++;

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
			StorageLayer* newlayer = new StorageLayer(layerUID, storage->UID);
			if (working_with_file){
				if (layerUID > (LayerFromFile.size() - 1))
					break;
				newlayer->DepthMap = LayerFromFile[layerUID];
			}else{
				grabber->getCloud();
				newlayer->DepthMap = grabber->PointCloudXYZPtr;
			}

			//Calc delta
			newlayer->planeDensity = plane_density;
			storage->AddNewLayer(*newlayer);
			std::cout << "Calcutating delta..." << std::endl;
			 
			storage->ObjectLimitSize[0] = object_minx;
			storage->ObjectLimitSize[1] = object_miny;
			storage->ObjectLimitSize[2] = object_minz;
			storage->ObjectLimitSize[3] = object_maxx;
			storage->ObjectLimitSize[4] = object_maxy;
			storage->ObjectLimitSize[5] = object_maxz;

			storage->CalcNewLayerDelta(plane_claster_tolerance, min_plane_claster_size,	max_plane_claster_size, cloud_z_step);
			
			if (saving_state){
				newlayer->SaveLayerToPCD(false, save_only_last);
			}

			//Show positive delta cloud
			delta_viewer->addPointCloud(storage->LayerList[storage->LayerList.size() - 1]->layerPositiveDelta, "delta_pos_cloud", 0); //DEBUG
			delta_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)0 / (float)255, (float)255 / (float)255, (float)0 / (float)255, "delta_pos_cloud"); //DEBUG

			//Show negative delta cloud
			delta_viewer->addPointCloud(storage->LayerList[storage->LayerList.size() - 1]->layerNegativeDelta, "delta_neg_cloud", 0); //DEBUG
			delta_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)255 / (float)255, (float)0 / (float)255, (float)0 / (float)255, "delta_neg_cloud"); //DEBUG

			//oldlayer->DepthMap.swap(newlayer->DepthMap);

			//std::cout << "Finding positive clasters..." << std::endl;
			//Extract clasters from positive delta
			//Auto calculation claster's parameters
			float cur_planeDensity = storage->LayerList[storage->LayerList.size() - 1]->planeDensity;
			//float min_mult = 1;
			//float max_mult = 10;
			//float toll_milt = 2;
			//claster_tolerance = toll_milt * cur_planeDensity;
			//minpoints = (int)(min_mult * (((float)object_minx / (float)cur_planeDensity) * ((float)object_miny / (float)cur_planeDensity)));
			//maxpoints = (int)(max_mult * (((float)object_maxx / (float)cur_planeDensity) * ((float)object_maxy / (float)cur_planeDensity)));
			
			
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

					//claster_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 200.0f / (float)255.0, (float)(rand() % 255) / (float)255.0, 200.0f / (float)255.0, ss.str());
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

			//Show actual and removed objects
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
			delete newlayer;

			std::cout << "Saving state to database..." << std::endl;
			storage->saveStorageToDB();

			std::cout << std::endl << "Ready for scan layers..." << std::endl;
		}

		if (GetKeyState(VK_S) < 0){
			delta_viewer->saveCameraParameters("viewer.ini");// getCameraFile();
			std::cout << "Camera parameters was saved..." << std::endl;
			boost::this_thread::sleep(boost::posix_time::microseconds(1000));
			//break;
		}

		/*if (GetKeyState(VK_Q) < 0){
			std::cout << "Ready for "
		}*/

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

