#pragma once

#include <limits.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

using namespace System;
using namespace System::ComponentModel;
using namespace System::Collections;
using namespace System::Windows::Forms;
using namespace System::Data;
using namespace System::Drawing;
using namespace System::Collections::Generic;

using namespace System::Drawing::Imaging;
using namespace System::Threading;

using namespace OpenTK;
using namespace OpenNI;

using namespace cv;
using namespace std;

ref class OpenCVtools
{
public:
	OpenCVtools();

//private: static int intersect(System::Drawing::Point^ a, System::Drawing::Point^ b, System::Drawing::Point^ c);
public: static bool isInside(ArrayList^ polygon, System::Drawing::Point^ p);

//private:static bool onSegment(System::Drawing::Point^ p, System::Drawing::Point^ q, System::Drawing::Point^ r);
//private:static  int orientation(System::Drawing::Point^ p, System::Drawing::Point^ q, System::Drawing::Point^ r);
//private:static  bool doIntersect(System::Drawing::Point^ p1, System::Drawing::Point^ q1, System::Drawing::Point^ p2, System::Drawing::Point^ q2);



};



