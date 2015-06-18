// Storage3D2Console.cpp : Defines the entry point for the console application.
//
#pragma warning (disable: 4703)

#pragma once
#include "stdafx.h"
#include "kinect_grabber.h"
#include "storage.h"
#include "Storage3D2Functions.h"

#define VK_S           0x53

//#include "Scaner3D.h"

//using namespace pcl;
using namespace std;

int _tmain(int argc, _TCHAR* argv[])
{
	//Loading settings file first time
	loadSettingsFile();

	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> LayerFromFile;
	int layerUID = 0;

	if (working_with_file){
		string dirpath = "./experimental_pcd/";
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
	
	//vector<StoredObject> objects_list;
	//KinectGrabber grabber = KinectGrabber(0, false);
	KinectGrab *grabber = new KinectGrab();

	try{
		grabber = new KinectGrab(false);
	}
	catch (std::exception& e){
		std::cout << "Kinect error: " << e.what() << std::endl;
		//cin.get();
		if(!working_with_file)
			return 0;
	}

	//Storage *storage = new Storage(delta_limit, plane_threshold);
	Storage storage = Storage(1);

	pcl::visualization::PCLVisualizer delta_viewer("Delta Viewer");
	pcl::visualization::PCLVisualizer pos_claster_viewer("Positive Claster Viewer");
	pcl::visualization::PCLVisualizer neg_claster_viewer("Negative Claster Viewer");

	delta_viewer.setBackgroundColor(0, 0, 0);
	delta_viewer.addCoordinateSystem(1.0);
	delta_viewer.loadCameraParameters("viewer.ini");
	//delta_viewer.initCameraParameters();

	pos_claster_viewer.setBackgroundColor(0, 0, 0);
	pos_claster_viewer.addCoordinateSystem(1.0);
	pos_claster_viewer.loadCameraParameters("viewer.ini");
	//claster_viewer.initCameraParameters();

	neg_claster_viewer.setBackgroundColor(0, 0, 0);
	neg_claster_viewer.addCoordinateSystem(1.0);
	neg_claster_viewer.loadCameraParameters("viewer.ini");

	//Grab layer
	StorageLayer oldlayer = StorageLayer(layerUID, storage.UID);
	if (working_with_file){
		oldlayer.DepthMap = LayerFromFile[layerUID];
	}
	else{
		//Grab layer
		grabber->start();
		grabber->getCloud();
		
		//Save first layer
		oldlayer.DepthMap = grabber->PointCloudXYZPtr; //FIXME. Add initialization from file or from last saved layer's DepthMap
	}

	if (saving_state){
		oldlayer.SaveLayerToPCD(true);
	}
	
	//grabber->stop();

	//pcl::PointCloud<pcl::PointXYZ>::Ptr old_cloud(new pcl::PointCloud<pcl::PointXYZ>); //Debug
	//pcl::io::loadPCDFile("old_cloud.pcd", *old_cloud); //Debug


	//Main cycle for listen keys press
	while (true){
		if (GetKeyState(VK_SPACE) < 0){
			layerUID++;

			//Set camera position in viewers windows
			delta_viewer.loadCameraParameters("viewer.ini");
			pos_claster_viewer.loadCameraParameters("viewer.ini");
			neg_claster_viewer.loadCameraParameters("viewer.ini");

			//Loading settings file again
			loadSettingsFile();
			
			storage.deltaLimit = delta_limit;
			storage.DistanceThreshold = plane_threshold;
			storage.zEpsAngle = zepsangle;
			storage.planeFiltration = planefiltration;
			
			std::cout << "Adding new storage layer..." << std::endl;
			delta_viewer.removeAllPointClouds();
			delta_viewer.removeAllShapes();
			pos_claster_viewer.removeAllPointClouds();
			pos_claster_viewer.removeAllShapes();
			neg_claster_viewer.removeAllPointClouds();
			neg_claster_viewer.removeAllShapes();

			//Grab layer
			StorageLayer newlayer = StorageLayer(layerUID, storage.UID);
			if (working_with_file){
				if (layerUID > (LayerFromFile.size() - 1))
					break;
				newlayer.DepthMap = LayerFromFile[layerUID];
			}else{
				grabber->getCloud();
				newlayer.DepthMap = grabber->PointCloudXYZPtr;
			}

			//grabber.stop();
			//Calc delta
			storage.AddNewLayer(newlayer);
			std::cout << "Calcutating delta..." << std::endl;
			storage.CalcNewLayerDelta(oldlayer.DepthMap, newlayer.DepthMap, newlayer.layerPositiveDelta, newlayer.layerNegativeDelta);
			
			if (saving_state){
				newlayer.SaveLayerToPCD();
			}
			
			//Show positive delta cloud
			delta_viewer.addPointCloud(newlayer.layerPositiveDelta, "delta_pos_cloud", 0);
			delta_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)0 / (float)255, (float)255 / (float)255, (float)0 / (float)255, "delta_pos_cloud");

			//Show negative delta cloud
			delta_viewer.addPointCloud(newlayer.layerNegativeDelta, "delta_neg_cloud", 0);
			delta_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)255 / (float)255, (float)0 / (float)255, (float)0 / (float)255, "delta_neg_cloud");

			oldlayer.DepthMap.swap(newlayer.DepthMap);

			std::cout << "Finding positive clasters..." << std::endl;
			//Extract clasters from positive delta
			//int minpoints = (int)(newlayer->layerNegativeDelta->size() * relative_claster_min);
			//int maxpoints = (int)(newlayer->layerNegativeDelta->size() * relative_claster_max);

			//int minpoints = (int)(newlayer->layerNegativeDelta->size() * relative_claster_min);
			//int maxpoints = (int)(newlayer->layerNegativeDelta->size() * relative_claster_max);

			StorageLayer::FindClaster(newlayer.layerPositiveDelta, newlayer.PositiveClasterList, claster_tolerance, minpoints, maxpoints);
			std::cout << "Was found " << newlayer.PositiveClasterList.size() << " positive clasters." << std::endl;
			
			if (newlayer.PositiveClasterList.size() > 0){
				for (int i = 0; i < newlayer.PositiveClasterList.size() - 1; i++){
					std::stringstream ss;
					ss << "PositiveClaster_" << i;

					pos_claster_viewer.addPointCloud(newlayer.PositiveClasterList[i], ss.str());

					//claster_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 200.0f / (float)255.0, (float)(rand() % 255) / (float)255.0, 200.0f / (float)255.0, ss.str());
					pos_claster_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)(rand() % 255) / (float)255.0, (float)(rand() % 255) / (float)255.0, (float)(rand() % 255) / (float)255.0, ss.str());

					pos_claster_viewer.spinOnce();
					boost::this_thread::sleep(boost::posix_time::microseconds(100));
				}
			}
			
			std::cout << "Finding negative clasters..." << std::endl;
			//Extract clasters from negative delta
			StorageLayer::FindClaster(newlayer.layerNegativeDelta, newlayer.NegativeClasterList, claster_tolerance, minpoints, maxpoints);
			std::cout << "Was found " << newlayer.NegativeClasterList.size() << " negative clasters" << std::endl;

			if (newlayer.NegativeClasterList.size() > 0){
				for (int i = 0; i < newlayer.NegativeClasterList.size() - 1; i++){
					std::stringstream ss;
					ss << "NegativeClaster_" << i;
					neg_claster_viewer.addPointCloud(newlayer.NegativeClasterList[i], ss.str());

					neg_claster_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)(rand() % 255) / (float)255.0, (float)(rand() % 255) / (float)255.0, (float)(rand() % 255) / (float)255.0, ss.str());

					neg_claster_viewer.spinOnce();
					boost::this_thread::sleep(boost::posix_time::microseconds(100));
				}
			}

			std::cout << "Finding objects for add..." << std::endl;
			//Find objects for add
			newlayer.FindObjectForAdd(object_minx, object_miny, object_minz, object_maxx, object_maxy, object_maxz);

			std::cout << newlayer.objectForAddList.size() - 1 << "new objects was found." << std::endl;
			std::cout << "Adding founded objects..." << std::endl;
			//Add founded objects
			
			for (int i = 0; i < newlayer.objectForAddList.size() - 1; i++){
				storage.AddNewObject(newlayer.objectForAddList[i]);
				std::cout << i + 1 << "object was added" << std::endl;
			}

			//Find objects for remove
			//storage->FindObjectForRemove(newlayer->objectEraserList);

			//Remove founded objects
			//storage->RemoveObjects();

			//viewer.setBackgroundColor(0, 0, 0);
			//viewer.addCoordinateSystem(1.0);
			//viewer.initCameraParameters();
			/*std::cout << "Refreshing visualization..." << std::endl;

			//Show erasers
			for (int i = 0; i < newlayer->objectEraserList.size() - 1; i++){
			std::stringstream ss;
			ss << "Eraser" << i;

			viewer.addCube(newlayer->objectEraserList[i].position,
			newlayer->objectEraserList[i].quaternion_to_bbox,
			newlayer->objectEraserList[i].width,
			newlayer->objectEraserList[i].lenght,
			newlayer->objectEraserList[i].height, ss.str());
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)200 / (float)255, (float)0 / (float)255, (float)0 / (float)255, ss.str());
			}*/

			//Show actual and removed objects
			for (int i = 0; i < storage.ObjectList.size() - 1; i++){
				std::stringstream ss;
				ss << "Actual objects" << i;

				delta_viewer.addCube(storage.ObjectList[i].position,
					storage.ObjectList[i].quaternion_to_bbox,
					storage.ObjectList[i].width,
					storage.ObjectList[i].lenght,
					storage.ObjectList[i].height, ss.str());

				if (!storage.ObjectList[i].removed){
					delta_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)255 / (float)255, (float)255 / (float)255, (float)255 / (float)255, ss.str());
				}
				else{
					delta_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)0 / (float)255, (float)0 / (float)255, (float)255 / (float)255, ss.str());
				}
			}
		}

		if (GetKeyState(VK_S) < 0){
			delta_viewer.saveCameraParameters("viewer.ini");// getCameraFile();
			std::cout << "Camera parameters was saved..." << std::endl;
			//break;
		}

		if (GetKeyState(VK_ESCAPE) < 0){
			break;
		}

		delta_viewer.spinOnce();
		pos_claster_viewer.spinOnce();
		neg_claster_viewer.spinOnce();
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

