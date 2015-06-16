// PCD_ConsoleViewer.cpp : Defines the entry point for the console application.
//



#include "stdafx.h"

#include <iostream>
#include <sys/types.h> 
#include <dirent.h>

#include <regex>
#include <vector>
#include <string>
#include <limits> 

#define NOMINMAX
#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>


using namespace pcl;
using namespace std;

int _tmain(int argc, _TCHAR* argv[])
{
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_vector;
	
	DIR *dir = opendir("./");
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = new pcl::PointCloud<pcl::PointXYZ>();
	//pcl::io::loadPCDFile("point_cloud", *cloud);

	//viewer.addPointCloud(cloud);
	
	if (dir)
	{
		struct dirent *ent;
		int i = 0;
		while ((ent = readdir(dir)) != NULL)
		{
			i++;
			if (((std::string)ent->d_name).find(".pcd") != string::npos)
			{
				cout << (ent->d_name) << " ";
				
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
				//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = new pcl::PointCloud<pcl::PointXYZ>();
				pcl::io::loadPCDFile(ent->d_name, *cloud);
				
				string cloud_id = "cloud" + std::to_string(i);
				
				//viewer.addPointCloud<pcl::PointXYZRGB>(cloud, cloud_id);

				cloud_vector.push_back(cloud);
				viewer.addPointCloud(cloud_vector[cloud_vector.size() - 1], cloud_id);
				
				viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)(rand() % 255) / (float)255.0, (float)(rand() % 255) / (float)255.0, (float)(rand() % 255) / (float)255.0, cloud_id);
				//break;
				//viewer.updatePointCloud()
			}
		}
	}
	else
	{
		cout << "Error opening directory" << endl;
	}
	
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}