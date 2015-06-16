// Storage3D2Console.cpp : Defines the entry point for the console application.
//
#pragma once
#include "stdafx.h"
#include "kinect_grabber.h"
#include "storage.h"


//#include "Scaner3D.h"

using namespace pcl;
using namespace std;

int _tmain(int argc, _TCHAR* argv[])
{
	//vector<StoredObject> objects_list;

	KinectGrabber grabber = KinectGrabber(0, false);
	Storage *storage = new Storage();

	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

	viewer.setBackgroundColor(0, 0, 0);
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();

	//Grab layer
	grabber.start();
	grabber.getCloud();
	StorageLayer *oldlayer = new StorageLayer();
	//Save first layer
	oldlayer->DepthMap = grabber.PointCloudXYZPtr;
	grabber.stop();

	//pcl::PointCloud<pcl::PointXYZ>::Ptr old_cloud(new pcl::PointCloud<pcl::PointXYZ>); //Debug
	//pcl::io::loadPCDFile("old_cloud.pcd", *old_cloud); //Debug

	//Main cycle for listen keys press
	while (true){
		if (GetKeyState(VK_SPACE) < 0){
			//Grab layer
			//grabber.start();
			grabber.getCloud();
			StorageLayer *newlayer = new StorageLayer();
			//Save layer
			newlayer->DepthMap = grabber.PointCloudXYZPtr;
			//grabber.stop();
			//Calc delta
			storage->AddNewLayer(*newlayer);
 			storage->CalcNewLayerDelta(oldlayer->DepthMap, newlayer->DepthMap, newlayer->layerPositiveDelta, newlayer->layerNegativeDelta);
			oldlayer->DepthMap.swap(newlayer->DepthMap);

			//Extract clasters from positive delta
			//StorageLayer::FindClaster(newlayer->layerPositiveDelta, newlayer->PositiveClasterList);
			
			//Extract clasters from negative delta
			//StorageLayer::FindClaster(newlayer->layerNegativeDelta, newlayer->NegativeClasterList);
			
			//Find objects for add
			//newlayer->FindObjectForAdd(0.2, 0.2, 0.2, 0.4, 0.4, 0.4);

			//Add founded objects
			//for (int i = 0; i < newlayer->objectForAddList.size() - 1; i++){
			//	storage->AddNewObject(newlayer->objectForAddList[i]);
			//}
			
			//Find objects for remove
			//storage->FindObjectForRemove(newlayer->objectEraserList);
			
			//Remove founded objects
			//storage->RemoveObjects();

			//viewer.setBackgroundColor(0, 0, 0);
			//viewer.addCoordinateSystem(1.0);
			//viewer.initCameraParameters();

			//Show positive delta cloud
			viewer.addPointCloud(newlayer->layerPositiveDelta, "delta_pos_cloud", 0);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)0 / (float)255, (float)255 / (float)255, (float)0 / (float)255, "delta_pos_cloud");

			//Show negative delta cloud
			viewer.addPointCloud(newlayer->layerNegativeDelta, "delta_neg_cloud", 0);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)255 / (float)255, (float)0 / (float)255, (float)0 / (float)255, "delta_neg_cloud");

			//Show erasers
			/*for (int i = 0; i < newlayer->objectEraserList.size() - 1; i++){
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
			/*for (int i = 0; i < storage->ObjectList.size() - 1; i++){
				std::stringstream ss;
				ss << "Actual objects" << i;

				viewer.addCube(storage->ObjectList[i].position,
					storage->ObjectList[i].quaternion_to_bbox,
					storage->ObjectList[i].width,
					storage->ObjectList[i].lenght,
					storage->ObjectList[i].height, ss.str());

				if (!storage->ObjectList[i].removed){
					viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)255 / (float)255, (float)255 / (float)255, (float)255 / (float)255, ss.str());
				}
				else{
					viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)0 / (float)255, (float)0 / (float)255, (float)255 / (float)255, ss.str());
				}
			}*/
		}

		if (GetKeyState(VK_ESCAPE) < 0){
			break;
		}

		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	grabber.stop();
	
	return 0;
}

