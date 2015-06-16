#pragma once
#include <Windows.h>
#include <NuiApi.h>
#include <time.h>

//#define NOMINMAX
#include <pcl/io/boost.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


class ScanerKinect// : public pcl::Grabber
{
public:
	ScanerKinect(const int index = 0, bool rgbmode = false);
	~ScanerKinect();
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

	void convertDepthToPointXYZ(NUI_LOCKED_RECT* depthLockedRect);
	void convertRGBDepthToPointXYZRGB(NUI_LOCKED_RECT* colorLockedRect, NUI_LOCKED_RECT* depthLockedRect);

	boost::thread thread;
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

