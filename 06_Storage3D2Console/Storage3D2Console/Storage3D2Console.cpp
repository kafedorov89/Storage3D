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
	//Storage *storage = new Storage(delta_limit, plane_threshold);
	Storage* storage = new Storage(1, delta_limit, enable_voxelgridfiltration, enable_planefiltration, enable_noizefiltration);

	pcl::visualization::PCLVisualizer *delta_viewer = new pcl::visualization::PCLVisualizer("Delta Viewer");
	pcl::visualization::PCLVisualizer *pos_claster_viewer = new pcl::visualization::PCLVisualizer("Positive Claster Viewer"); //DEBUG
	pcl::visualization::PCLVisualizer *neg_claster_viewer = new pcl::visualization::PCLVisualizer("Negative Claster Viewer"); //DEBUG
	
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> LayerFromFile;
	int layerUID = 0;

	//Loading files instead stream from 3D-scanner
	if (working_with_file){
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
			cout << "Error opening directory" << endl;
		}
	}

	try{
		grabber = new KinectGrab(false);
	}
	catch (std::exception& e){
		std::cout << "Kinect error: " << e.what() << std::endl;
		//cin.get();
		if(!working_with_file)
			return 0;
	}

	delta_viewer->setBackgroundColor(0, 0, 0);
	delta_viewer->addCoordinateSystem(1.0);
	delta_viewer->loadCameraParameters("viewer.ini");
	//delta_viewer->initCameraParameters();

	pos_claster_viewer->setBackgroundColor(0, 0, 0); //DEBUG
	pos_claster_viewer->addCoordinateSystem(1.0);
	pos_claster_viewer->loadCameraParameters("viewer.ini");
	//claster_viewer->initCameraParameters();

	neg_claster_viewer->setBackgroundColor(0, 0, 0);
	neg_claster_viewer->addCoordinateSystem(1.0);
	neg_claster_viewer->loadCameraParameters("viewer.ini");

	//Grab layer
	StorageLayer* oldlayer = new StorageLayer(layerUID, storage->UID);
	if (working_with_file){
		oldlayer->DepthMap = LayerFromFile[layerUID];
	}
	else{
		//Grab layer
		grabber->start();
		grabber->getCloud();
		
		//Save first layer
		oldlayer->DepthMap = grabber->PointCloudXYZPtr; //FIXME. Add initialization from file or from last saved layer's DepthMap
	}

	if (saving_state){
		oldlayer->SaveLayerToPCD(true);
	}

	oldlayer->planeDensity = plane_density;
	storage->AddNewLayer(*oldlayer);

	//Main cycle for listen keys press
	std::cout << "Ready for scan layers..." << std::endl;
	while (true){
		if (GetKeyState(VK_SPACE) < 0){
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
				newlayer->SaveLayerToPCD();
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
					ss << "PositiveClaster_" << i;

					pos_claster_viewer->addPointCloud(storage->LayerList[storage->LayerList.size() - 1]->PositiveClasterList[i], ss.str());

					//claster_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 200.0f / (float)255.0, (float)(rand() % 255) / (float)255.0, 200.0f / (float)255.0, ss.str());
					pos_claster_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)(rand() % 255) / (float)255.0, (float)(rand() % 255) / (float)255.0, (float)(rand() % 255) / (float)255.0, ss.str());

					pos_claster_viewer->spinOnce();
					boost::this_thread::sleep(boost::posix_time::microseconds(100));
				}
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
					ss << "NegativeClaster_" << i;
					neg_claster_viewer->addPointCloud(storage->LayerList[storage->LayerList.size() - 1]->NegativeClasterList[i], ss.str());

					neg_claster_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)(rand() % 255) / (float)255.0, (float)(rand() % 255) / (float)255.0, (float)(rand() % 255) / (float)255.0, ss.str());

					neg_claster_viewer->spinOnce();
					boost::this_thread::sleep(boost::posix_time::microseconds(100));
				}
			}

			
			

			std::cout << "Finding objects for add..." << std::endl;
			//Find objects
			
			storage->FindObjects(storage->LayerList.size() - 1, valid_percent, nearest_point_count, object_density);
			
			if (storage->LayerList[storage->LayerList.size() - 1]->objectForAddList.size() > 0)
			{
				std::cout << storage->LayerList[storage->LayerList.size() - 1]->objectForAddList.size() << " new objects was found." << std::endl;
				std::cout << "Adding founded objects..." << std::endl;
				//Add founded objects
				for (int i = 0; i < storage->LayerList[storage->LayerList.size() - 1]->objectForAddList.size(); i++){
					storage->AddNewObject(*storage->LayerList[storage->LayerList.size() - 1]->objectForAddList[i]);
					std::cout << i + 1 << "object was added" << std::endl;

				}
			}
			else{
				std::cout << "Objects wasn't founded..." << std::endl;
			}

			//Find removers
			storage->FindRemovers(storage->LayerList.size() - 1, valid_percent, nearest_point_count, object_density);

			if (storage->LayerList[storage->LayerList.size() - 1]->objectEraserList.size() > 0)
			{
				std::cout << storage->LayerList[storage->LayerList.size() - 1]->objectEraserList.size() << " new eraser was found." << std::endl;
				std::cout << "Adding founded eraser..." << std::endl;
				//Add founded objects
				for (int i = 0; i < storage->LayerList[storage->LayerList.size() - 1]->objectEraserList.size(); i++){
					storage->AddNewObject(*storage->LayerList[storage->LayerList.size() - 1]->objectEraserList[i]);
					std::cout << i + 1 << "object was added" << std::endl;

				}
			}
			else{
				std::cout << "Objects wasn't founded..." << std::endl;
			}

			//Remove founded objects
			//storage->RemoveObjects();

			//viewer.setBackgroundColor(0, 0, 0);
			//viewer.addCoordinateSystem(1.0);
			//viewer.initCameraParameters();
			std::cout << "Refreshing visualization..." << std::endl;

			//Show erasers
			for (int i = 0; i < storage->LayerList[storage->LayerList.size() - 1]->objectEraserList.size() - 1; i++){
			std::stringstream ss;
			ss << "Eraser" << i;

			neg_claster_viewer->addCube(storage->LayerList[storage->LayerList.size() - 1]->objectEraserList[i].position,
				storage->LayerList[storage->LayerList.size() - 1]->objectEraserList[i].quaternion_to_bbox,
				storage->LayerList[storage->LayerList.size() - 1]->objectEraserList[i].width,
				storage->LayerList[storage->LayerList.size() - 1]->objectEraserList[i].lenght,
				storage->LayerList[storage->LayerList.size() - 1]->objectEraserList[i].height, ss.str());
			neg_claster_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)200 / (float)255, (float)0 / (float)255, (float)0 / (float)255, ss.str());
			}

			//Show actual and removed objects
 			if (storage->ObjectList.size()){ //DEBUG
				for (int i = 0; i < storage->ObjectList.size(); i++){
					std::stringstream ss;
					ss << "Actual_objects_" << i;

					std::cout << "Position: " << std::endl << "x " << storage->ObjectList[i]->position(0) << std::endl << "y " << storage->ObjectList[i]->position(1) << std::endl << "z " << storage->ObjectList[i]->position(2) << std::endl;
					std::cout << "width: " << storage->ObjectList[i]->width << std::endl;
					std::cout << "lenght: " << storage->ObjectList[i]->lenght << std::endl;
					std::cout << "height: " << storage->ObjectList[i]->height << std::endl;

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
		}

		if (GetKeyState(VK_S) < 0){
			delta_viewer->saveCameraParameters("viewer.ini");// getCameraFile();
			std::cout << "Camera parameters was saved..." << std::endl;
			//break;
		}

		if (GetKeyState(VK_ESCAPE) < 0){
			break;
		}

		delta_viewer->spinOnce();
		//pos_claster_viewer->spinOnce(); //DEBUG
		//neg_claster_viewer->spinOnce(); //DEBUG
		boost::this_thread::sleep(boost::posix_time::microseconds(100));
	}

	if (!working_with_file)
	{
		//try{
		grabber->stop();
		//}
		//catch (std::exception& e){

		//}
	}

	return 0;
}

