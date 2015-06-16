#include "stdafx.h"

// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
	#define _SCL_SECURE_NO_WARNINGS
#endif

#include "kinect_grabber.h"
//#include <pcl/visualization/cloud_viewer.h>


int _tmain( int argc, _TCHAR* argv[] )
{
	// Create Cloud Viewer
	//pcl::visualization::CloudViewer viewer( "Point Cloud Viewer" );

	// Create KinectGrabber
	pcl::KinectGrabber* grabber = new pcl::KinectGrabber();

	// Start Retrieve Data
	//grabber->start();

	//Manual grabber mode
	while(true){
		// Input Key ( Exit ESC key )
		if( GetKeyState( VK_SPACE ) < 0 ){
			// Start Retrieve Data
			grabber->start();
		}
		if (GetKeyState(VK_ESCAPE) < 0){
			break;
		}
	}

	//Socket listner
	//??

	// Stop Retrieve Data
	grabber->stop();

	return 0;
}

