#pragma once
#pragma comment (lib, "opencv_world300d.lib")



#include "OpenCVtools.h"

OpenCVtools::OpenCVtools()
{
}



// Returns true if the point p lies inside the polygon[] with n vertices
bool OpenCVtools::isInside(ArrayList^ poly, System::Drawing::Point^ p){
	
	vector<cv::Point> cvPoligon(0);
	cv::Point2f cvPoint = cv::Point2f();

	cvPoint.x = (float)p->X;
	cvPoint.y = (float)p->Y;

	array<Object^, 1>^ objPoly = poly->ToArray();
	array<System::Drawing::Point^, 1>^ V = gcnew array<System::Drawing::Point^, 1>(objPoly->Length);

	for (int i = 0; i < objPoly->Length; i++){
		System::Drawing::Point^ iSysPoint = (System::Drawing::Point^)objPoly[i];
		cv::Point iPoint = cv::Point((int)iSysPoint->X, (int)iSysPoint->Y);
		cvPoligon.push_back(iPoint);
		//cvPoligon[i] = ;
	}

	int result = pointPolygonTest(cvPoligon, cvPoint, false);
	if (result == 0 || result == 1){
		return true;
	}
	else{
		return false;
	}
}



/*int OpenCVtools::intersect(System::Drawing::Point^ a, System::Drawing::Point^ b, System::Drawing::Point^ c)
{
	int ax = a->X - c->X;
	int ay = a->Y - c->Y;
	int bx = b->X - c->X;
	int by = b->Y - c->Y;

	int s = Math::Sign(ax * by - ay * bx);
	if (ay < 0 ^ by < 0)
	{
		if (by < 0)
			return s;
		return -s;
	}
	if (s == 0 && (ay == 0 || by == 0) && ax * bx <= 0)
		return 0;
	return 1;
}

bool OpenCVtools::isInside(ArrayList^ poly, System::Drawing::Point^ a){
	System::Drawing::Point^ middle = a;
	int result = 0;
	for (int i = 1; i < poly->Count; i++)
	{
		result += intersect((System::Drawing::Point^)poly[i - 1], (System::Drawing::Point^)poly[i], middle);
	}
	return result < 3;
}

bool OpenCVtools::isInside(ArrayList^ poly, System::Drawing::Point^ a)
{
	int i;
	double angle = 0;
	Point p1, p2;

	for (i = 0; i<n; i++) {
		p1.h = polygon[i].h - p.h;
		p1.v = polygon[i].v - p.v;
		p2.h = polygon[(i + 1) % n].h - p.h;
		p2.v = polygon[(i + 1) % n].v - p.v;
		angle += Angle2D(p1.h, p1.v, p2.h, p2.v);
	}

	if (ABS(angle) < PI)
		return(FALSE);
	else
		return(TRUE);
}

//Return the angle between two vectors on a plane
//The angle is from vector 1 to vector 2, positive anticlockwise
//The result is between -pi -> pi
double Poligon2D::Angle2D(double x1, double y1, double x2, double y2)
{
	double dtheta, theta1, theta2;

	theta1 = atan2(y1, x1);
	theta2 = atan2(y2, x2);
	dtheta = theta2 - theta1;
	while (dtheta > PI)
		dtheta -= TWOPI;
	while (dtheta < -PI)
		dtheta += TWOPI;

	return(dtheta);
}*/


/*bool OpenCVtools::isInside(ArrayList^ poly, System::Drawing::Point^ a){
	int n2 = 0;
	bool f = false;
	array<Object^, 1>^ objV = poly->ToArray();
	array<System::Drawing::Point^, 1>^ V = gcnew array<System::Drawing::Point^, 1>(objV->Length);

	for (int i = 0; i < objV->Length; i++){
		V[i] = (System::Drawing::Point^)objV[i];
	}

	for (long n1 = 1; n1 < V->Length; n1++)
	{
		int n2 = (long)(n1 + 1) % (long)V->Length;
		bool A = a->Y > V[n1]->Y;
		bool B = a->Y > V[n2]->Y;
		if ((A || B) && (!A || !B)){
			double expr = (double)(V[n1]->X + (V[n2]->X - V[n1]->X) * (a->Y - V[n1]->Y)) / (double)(V[n2]->Y - V[n1]->Y);
			if (a->X > expr){
				f = !f;
			}
		}
	}

	return f;
}*/


/*// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool OpenCVtools::onSegment(System::Drawing::Point^ p, System::Drawing::Point^ q, System::Drawing::Point^ r)
{
	if (q->X <= Math::Max(p->X, r->X) && q->X >= Math::Min(p->X, r->X) &&
		q->Y <= Math::Max(p->Y, r->Y) && q->Y >= Math::Min(p->Y, r->Y))
		return true;
	return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int OpenCVtools::orientation(System::Drawing::Point^ p, System::Drawing::Point^ q, System::Drawing::Point^ r)
{
	int val = (q->Y - p->Y) * (r->X - q->X) -
		(q->X - p->X) * (r->Y - q->Y);

	if (val == 0) return 0;  // colinear
	return (val > 0) ? 1 : 2; // clock or counterclock wise
}

// The function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool OpenCVtools::doIntersect(System::Drawing::Point^ p1, System::Drawing::Point^ q1, System::Drawing::Point^ p2, System::Drawing::Point^ q2)
{
	// Find the four orientations needed for general and
	// special cases
	int o1 = orientation(p1, q1, p2);
	int o2 = orientation(p1, q1, q2);
	int o3 = orientation(p2, q2, p1);
	int o4 = orientation(p2, q2, q1);

	// General case
	if (o1 != o2 && o3 != o4)
	{
		if (!o4)
		{
			return false;
		}
		if (!o3)
		{
			if (o4 != 1)
			{
				return false;
			}
		}
		return true;
	}

	// Special Cases
	// p1, q1 and p2 are colinear and p2 lies on segment p1q1
	if (o1 == 0 && onSegment(p1, p2, q1)) return true;

	// p1, q1 and p2 are colinear and q2 lies on segment p1q1
	if (o2 == 0 && onSegment(p1, q2, q1)) return true;

	// p2, q2 and p1 are colinear and p1 lies on segment p2q2
	if (o3 == 0 && onSegment(p2, p1, q2)) return true;

	// p2, q2 and q1 are colinear and q1 lies on segment p2q2
	if (o4 == 0 && onSegment(p2, q1, q2)) return true;

	return false; // Doesn't fall in any of the above cases
}

// Returns true if the point p lies inside the polygon[] with n vertices
bool OpenCVtools::isInside(ArrayList^ poly, System::Drawing::Point^ p){
	array<Object^, 1>^ objP = poly->ToArray();
	int n = objP->Length;
	array<System::Drawing::Point^, 1>^ polygon = gcnew array<System::Drawing::Point^, 1>(objP->Length);

	for (int i = 0; i < n; i++){
		polygon[i] = (System::Drawing::Point^)objP[i];
	}

	// There must be at least 3 vertices in polygon[]
	if (n < 3)  return false;

	// Create a point for line segment from p to infinite
	System::Drawing::Point^ extreme = gcnew System::Drawing::Point(UINT_MAX, p->Y);

	// Count intersections of the above line with sides of polygon
	int count = 0, i = 0;
	do
	{
		int next = (i + 1) % n;

		// Check if the line segment from 'p' to 'extreme' intersects
		// with the line segment from 'polygon[i]' to 'polygon[next]'
		if (doIntersect(polygon[i], polygon[next], p, extreme))
		{
			// If the point 'p' is colinear with line segment 'i-next',
			// then check if it lies on segment. If it lies, return true,
			// otherwise false
			if (orientation(polygon[i], p, polygon[next]) == 0)
				return onSegment(polygon[i], p, polygon[next]);

			count++;
		}
		i = next;
	} while (i != 0);

	// Return true if count is odd, false otherwise
	return count & 1;  // Same as (count%2 == 1)
}*/

/*bool OpenCVtools::isInside(ArrayList^ poly, System::Drawing::Point^ p){
	array<Object^, 1>^ objP = poly->ToArray();
	int n = objP->Length;
	array<System::Drawing::Point^, 1>^ polygon = gcnew array<System::Drawing::Point^, 1>(objP->Length);

	for (int i = 0; i < n; i++){
		polygon[i] = (System::Drawing::Point^)objP[i];
	}

	// There must be at least 3 vertices in polygon[]
	if (n < 3)  return false;

	// Create a point for line segment from p to infinite
	System::Drawing::Point^ extreme = gcnew System::Drawing::Point(UINT_MAX, p->Y);

	// Count intersections of the above line with sides of polygon
	int count = 0, i = 0;
	do
	{
		if (doIntersect(polygon[i], polygon[(i + 1) % n], p, extreme))
		{
			if (onSegment(polygon[i], p, polygon[(i + 1) % n]))
				return true;
			count++;
		}
		i = (i + 1) % n;
	} while (i != 0);

	// Return true if count is odd, false otherwise
	return count & 1;  // Same as (count%2 == 1)
}*/


/*enum { INSIDE, OUTSIDE, BOUNDARY };         // ïîëîæåíèå òî÷êè
//     ÂÍÓÒĞÈ, ÂÍÅ,     ÍÀ ÃĞÀÍÈÖÅ
enum { TOUCHING, CROSSING, INESSENTIAL };   // ïîëîæåíèå ğåáğà
//     ÊÀÑÀÒÅËbÍÎÅ, ÏÅĞÅÑÅÊÀŞÙÅÅ, ÍÅÑÓÙÅÑÒÂÅÍÍÎÅ

int OpenCVtools::pointInPolygon(Point &a, Polygon &p)
{
	int parity = 0;
	for (int i = 0; i < p.size(); i++, p.advance(CLOCKWISE)) {
		Edge e = p.edge();
		switch (edgeType(a, e)) {
		case TOUCHING:
			return BOUNDARY;
		case CROSSING:
			parity = 1 - parity;
		}
	}
	return (parity ? INSIDE : OUTSIDE);
}

int OpenCVtools::edgeType(Point &a, Edge &e)
{
	Point v = e.org;
	Point w = e.dest;
	switch (a.classify(e)) {
	case LEFT:
		return ((v.y<a.y) && (a.y <= w.y)) ? CROSSING : INESSENTIAL;
	case RIGHT:
		return ((w.y<a.y) && (a.y <= v.y)) ? CROSSING : INESSENTIAL;
	case BETWEEN:
	case ORIGIN:
	case DESTINATION:
		return TOUCHING;
	default:
		return INESSENTIAL;
	}
}*/