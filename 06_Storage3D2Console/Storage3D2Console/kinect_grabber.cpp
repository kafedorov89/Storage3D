#include "stdafx.h"
#include "kinect_grabber.h"

KinectGrab::KinectGrab(){

}

KinectGrab::KinectGrab(bool rgbmode, const int index){
	sensor = nullptr;
	mapper = nullptr;
	result = S_OK;
	colorHandle = INVALID_HANDLE_VALUE;
	depthHandle = INVALID_HANDLE_VALUE;
	width = 640;
	height = 480;
	running = false;
	quit = false;
	
	RGBMode = rgbmode; //Switch on RGBXYZ mode

	PointCloudXYZPtr = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudXYZRGBPtr = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>(new pcl::PointCloud<pcl::PointXYZRGB>);

	// Retrieved Sensor Count that is Connected to PC 
	int count = 0;
	result = NuiGetSensorCount(&count);
	if (FAILED(result)){
		throw std::exception("Exception : NuiGetSensorCount");
	}

	if (count > index){
		// Create Sensor Instance
		result = NuiCreateSensorByIndex(index, &sensor);
		if (FAILED(result)){
			throw std::exception("Exception : NuiCreateSensorByIndex");
		}

		// Initialize Sensor
		result = sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX);
		if (FAILED(result)){
			throw std::exception("Exception : INuiSensor::NuiInitialize");
		}

		// Retrieved Coordinate Mapper
		result = sensor->NuiGetCoordinateMapper(&mapper);
		if (FAILED(result)){
			throw std::exception("Exception : INuiSensor::NuiGetCoordinateMapper");
		}
	}
	else{
		throw std::exception("Exception : Failed to Find a Kinect Sensor");
	}

	// Retrieved Image Size from Stream Resolution
	unsigned long refWidth = 0;
	unsigned long refHeight = 0;
	
	NuiImageResolutionToSize(NUI_IMAGE_RESOLUTION_640x480, refWidth, refHeight);
	
	width = static_cast<int>(refWidth);
	height = static_cast<int>(refHeight);
}

KinectGrab::~KinectGrab()
{
	stop();

	// End Processing
	sensor->NuiShutdown();
	mapper->Release();
	//thread.join();
}

void KinectGrab::start()
{
	//  Open Color Stream
	result = sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, 0, 2, 0, &colorHandle);
	if (FAILED(result)){
		throw std::exception("Exception : INuiSensor::NuiImageStreamOpen( Color )");
	}

	// Open Depth Stream
	result = sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, NUI_IMAGE_RESOLUTION_640x480, 0, 2, 0, &depthHandle);
	if (FAILED(result)){
		throw std::exception("Exception : INuiSensor::NuiImageStreamOpen( Depth )");
	}
	running = true;
	//getCloud();
}

void KinectGrab::stop()
{
	boost::unique_lock<boost::mutex> lock(mutex);

	quit = true;
	running = false;
	lock.unlock();
}

bool KinectGrab::isRunning() const
{
	boost::unique_lock<boost::mutex> lock(mutex);
	return running;
	lock.unlock();
}

std::string KinectGrab::getName() const{
	return std::string("KinectGrab");
}

float KinectGrab::getFramesPerSecond() const {
	return 30.0f;
}

void KinectGrab::getCloud()
{
	//Initializing clouds
	PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>();
	PointCloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>();
	PointCloudXYZPtr = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudXYZRGBPtr = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	boost::unique_lock<boost::mutex> lock(mutex);

	// Retrieved Color Data from Kinect
	NUI_IMAGE_FRAME colorImageFrame = { 0 };
	result = sensor->NuiImageStreamGetNextFrame(colorHandle, INFINITE, &colorImageFrame);
	if (FAILED(result)){
		throw std::exception("Exception : INuiSensor::NuiImageStreamGetNextFrame( Color )");
	}

	INuiFrameTexture* colorFrameTexture = colorImageFrame.pFrameTexture;
	NUI_LOCKED_RECT colorLockedRect;
	colorFrameTexture->LockRect(0, &colorLockedRect, nullptr, 0);

	colorFrameTexture->UnlockRect(0);
	sensor->NuiImageStreamReleaseFrame(colorHandle, &colorImageFrame);

	// Retrieved Depth Data from Kinect
	NUI_IMAGE_FRAME depthImageFrame = { 0 };
	result = sensor->NuiImageStreamGetNextFrame(depthHandle, INFINITE, &depthImageFrame);
	if (FAILED(result)){
		throw std::exception("Exception : INuiSensor::NuiImageStreamGetNextFrame( Depth )");
	}

	BOOL nearMode = false;
	INuiFrameTexture* depthFrameTexture = nullptr;
	result = sensor->NuiImageFrameGetDepthImagePixelFrameTexture(depthHandle, &depthImageFrame, &nearMode, &depthFrameTexture);
	if (FAILED(result)){
		throw std::exception("Exception : INuiSensor::NuiImageFrameGetDepthImagePixelFrameTexture");
	}
	NUI_LOCKED_RECT depthLockedRect;
	depthFrameTexture->LockRect(0, &depthLockedRect, nullptr, 0);

	depthFrameTexture->UnlockRect(0);
	sensor->NuiImageStreamReleaseFrame(depthHandle, &depthImageFrame);

	lock.unlock();

	/*time_t rawtime;
	struct tm * timeinfo;
	size_t buffer_size = 80;
	std::string timestring;

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	timestring = std::to_string(timeinfo->tm_sec) +
		std::to_string(timeinfo->tm_hour) +
		std::to_string(timeinfo->tm_min) +
		std::to_string(timeinfo->tm_mday) +
		std::to_string(timeinfo->tm_mon) +
		std::to_string(timeinfo->tm_year);

	std::string filename = "point_cloud_" + timestring + ".pcd";

	std::cout << "filename = " << filename;*/

	if (RGBMode){
		convertRGBDepthToPointXYZRGB(&colorLockedRect, &depthLockedRect);
		//pcl::io::savePCDFileBinaryCompressed(filename, PointCloudXYZRGB);

	}
	else{
		convertDepthToPointXYZ(&depthLockedRect);
		//pcl::io::savePCDFileBinaryCompressed(filename, PointCloudXYZ);
	}
}

void KinectGrab::convertDepthToPointXYZ(NUI_LOCKED_RECT* depthLockedRect)
{
	NUI_DEPTH_IMAGE_PIXEL* depthPixel = reinterpret_cast<NUI_DEPTH_IMAGE_PIXEL*>(depthLockedRect->pBits);

	int i = 0;
	PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>(width, height, pcl::PointXYZ());
	PointCloudXYZ.is_dense = false;

	for (int y = 0; y < height; y++){
		for (int x = 0; x < width; x++, i++){
			pcl::PointXYZ point;

			NUI_DEPTH_IMAGE_POINT depthPoint;
			depthPoint.x = x;
			depthPoint.y = y;
			depthPoint.depth = depthPixel[y * width + x].depth;

			// Coordinate Mapping Depth to Real Space, and Setting PointCloud XYZ
			Vector4 skeletonPoint;
			mapper->MapDepthPointToSkeletonPoint(NUI_IMAGE_RESOLUTION_640x480, &depthPoint, &skeletonPoint);

			point.x = skeletonPoint.x;
			point.y = skeletonPoint.y;
			point.z = skeletonPoint.z;

			PointCloudXYZ[i] = pcl::PointXYZ(point);
			PointCloudXYZPtr->push_back(point);
		}
	}
}

void KinectGrab::convertRGBDepthToPointXYZRGB(NUI_LOCKED_RECT* colorLockedRect, NUI_LOCKED_RECT* depthLockedRect)
{
	NUI_DEPTH_IMAGE_PIXEL* depthPixel = reinterpret_cast<NUI_DEPTH_IMAGE_PIXEL*>(depthLockedRect->pBits);

	int i = 0;
	PointCloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>(width, height, pcl::PointXYZRGB());
	PointCloudXYZRGB.is_dense = false;

	for (int y = 0; y < height; y++){
		for (int x = 0; x < width; x++, i++){
			pcl::PointXYZRGB point;

			NUI_DEPTH_IMAGE_POINT depthPoint;
			depthPoint.x = x;
			depthPoint.y = y;
			depthPoint.depth = depthPixel[y * width + x].depth;

			// Coordinate Mapping Depth to Real Space, and Setting PointCloud XYZ
			Vector4 skeletonPoint;
			mapper->MapDepthPointToSkeletonPoint(NUI_IMAGE_RESOLUTION_640x480, &depthPoint, &skeletonPoint);

			point.x = skeletonPoint.x;
			point.y = skeletonPoint.y;
			point.z = skeletonPoint.z;

			// Coordinate Mapping Depth to Color Space, and Setting PointCloud RGB
			NUI_COLOR_IMAGE_POINT colorPoint;
			mapper->MapDepthPointToColorPoint(NUI_IMAGE_RESOLUTION_640x480, &depthPoint, NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, &colorPoint);

			if (0 <= colorPoint.x && colorPoint.x < width && 0 <= colorPoint.y && colorPoint.y < height){
				unsigned int index = colorPoint.y * colorLockedRect->Pitch + colorPoint.x * 4;
				point.b = colorLockedRect->pBits[index + 0];
				point.g = colorLockedRect->pBits[index + 1];
				point.r = colorLockedRect->pBits[index + 2];
			}

			PointCloudXYZRGB[i] = pcl::PointXYZRGB(point);
			PointCloudXYZRGBPtr->push_back(point);
		}
	}
}