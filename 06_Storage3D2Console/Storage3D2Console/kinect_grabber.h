// KinectGrabber is pcl::Grabber to retrieve the point cloud data from Kinect v1 using Kinect for Windows SDK v1.x.
// This source code is licensed under the MIT license. Please see the License in License.txt.
#pragma once
#include "stdafx.h"
//#ifndef KINECT_GRABBER
//#define KINECT_GRABBER

#ifndef KINECT_GRABBER
#define KINECT_GRABBER

#define NOMINMAX
#include <Windows.h>

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <boost/shared_ptr.hpp>

#include <time.h> 

#include "targetver.h"
#include <stdio.h>
#include <tchar.h>

#include <NuiApi.h>

class KinectGrab
{
public:
	KinectGrab();
	KinectGrab(bool rgbmode, const int index = 0);
	/*virtual ~KinectGrab() throw ();
	virtual void start();
	virtual void stop();
	virtual bool isRunning() const;
	virtual std::string getName() const;
	virtual float getFramesPerSecond() const;*/

	~KinectGrab();// throw ();
	void start();
	void stop();
	bool isRunning() const;
	std::string getName() const;
	float getFramesPerSecond() const;

	pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
	pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

	pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGBPtr;

	bool RGBMode;
	void getCloud();

protected:
	void convertDepthToPointXYZ(NUI_LOCKED_RECT* depthLockedRect);
	void convertRGBDepthToPointXYZRGB(NUI_LOCKED_RECT* colorLockedRect, NUI_LOCKED_RECT* depthLockedRect);

	//boost::thread thread;
	mutable boost::mutex mutex;

	bool quit;
	bool running;

	HRESULT result;
	INuiSensor* sensor;
	INuiCoordinateMapper* mapper;
	HANDLE colorHandle;
	HANDLE depthHandle;

	int width;
	int height;
};

#endif KINECT_GRABBER
