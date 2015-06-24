#pragma once
#include "stdafx.h"
#include "stored_object.h"

using namespace std;

StoredObject::StoredObject(){

}

StoredObject::StoredObject(const StoredObject& storedobject){
	removed = storedobject.removed;
	UID = storedobject.UID;
	storageUID = storedobject.storageUID;
	addedLayerID = storedobject.addedLayerID;
	//removedLayerID = storedobject.removedLayerID;
	//ObjectName = storedobject.ObjectName;
	//ObjectTypeName = storedobject.ObjectTypeName;
	//ObjectType = storedobject.ObjectType;
	AddedDate = storedobject.AddedDate;
	//RemovedDate = storedobject.RemovedDate;
	isValid = storedobject.isValid;
	object_cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>(*storedobject.object_cloud));
	step_degree = storedobject.step_degree;
	max_degree = storedobject.max_degree;
	objectDensity = storedobject.objectDensity;
	roll = storedobject.roll;
	pitch = storedobject.pitch;
	yaw = storedobject.yaw;
	width = storedobject.width;
	lenght = storedobject.lenght;
	height = storedobject.height;
	square = storedobject.square;
	ObjectName = storedobject.ObjectName;

	//position = new Eigen::Vector3f(*storedobject.position);
	//quaternion_to_bbox = storedobject.quaternion_to_bbox;//new Eigen::Quaternionf(*storedobject.quaternion_to_bbox);
	//jump_to_bbox = new Eigen::Affine3f(*storedobject.jump_to_bbox);
	//jump_to_zero = new Eigen::Affine3f(*storedobject.jump_to_zero);

	position = storedobject.position;
	quaternion_to_bbox = storedobject.quaternion_to_bbox;
	quaternion_to_zero = storedobject.quaternion_to_zero;
	jump_to_bbox = storedobject.jump_to_bbox;
	jump_to_zero = storedobject.jump_to_zero;
}

StoredObject::StoredObject(
	int layerid, 
	int storageid, 
	int uid, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, 
	int stepdegree, 
	int maxdegree, 
	float objectdensity, 
	string objectname)
{
	addedLayerID = layerid;
	
	time_t rawtime;
	time(&rawtime);
	AddedDate = rawtime;
	
	storageUID = storageid;
	UID = uid;
	
	width = FLT_MAX;
	lenght = FLT_MAX;
	height = FLT_MAX;

	isGroup = false;
	square = FLT_MAX;

	step_degree = stepdegree;
	max_degree = maxdegree;

	objectDensity = objectdensity;
	VoxelGridFiltration(cloud, object_cloud, objectdensity);

	yaw = 0; //Angle oZ in degrees
	roll = 0; //Angle oX in degrees
	pitch = 0; //Angle oX in degrees

	isValid = false;

	ObjectName = objectname;
	

	//find_bbox();
	//check_valid_object();
	//check_isinside_point(pcl::PointXYZ(0, 0, 0));
}

//Constructor for init object from database
StoredObject::StoredObject(
	int dbuid, 
	string dbname, 
	time_t dbadd_date, 
	time_t dbremove_date, 
	bool dbremoved, 
	float dbposition_x, 
	float dbposition_y, 
	float dbposition_z, 
	float dbwidth, 
	float dblenght, 
	float dbheight, 
	float dbroll, 
	float dbpitch, 
	float dbyaw)
{
	UID = dbuid;
	ObjectName = dbname;

	AddedDate = dbadd_date;
	RemovedDate = dbremove_date;
	removed = dbremoved;

	width = dbwidth;
	lenght = dblenght;
	height = dbheight;

	roll = dbroll;
	pitch = dbpitch;
	yaw = dbyaw;

	position(0) = dbposition_x;
	position(1) = dbposition_y;
	position(2) = dbposition_z;

	//find_bbox();
	//check_valid_object();
	//check_isinside_point(pcl::PointXYZ(0, 0, 0));
}

StoredObject::~StoredObject()
{
}

void StoredObject::check_2d_valid_object(float limit_array[6], float valid_percent){
	
	std::cout << " width = " << width << std::endl;
	std::cout << " lenght = " << lenght << std::endl;
	
	if ((width > limit_array[0] && width < limit_array[3]) && (lenght > limit_array[1] && lenght < limit_array[4])){
		
		//Cheking for points count in founded rectangle relate of square. 70% should be covered by points
		if ((float)object_cloud->size() / ((width / objectDensity) * (lenght / objectDensity)) > valid_percent){
			std::cout << " Valid 2d object" << std::endl;
			isValid = true;
		}
		else{
			std::cout << " Not Valid 2d object" << std::endl;
			isValid = false;
		}
	}
	else{
		std::cout << " Not Valid 2d object" << std::endl;
		isValid = false;
	}
}

bool StoredObject::check_isinside_point(const pcl::PointXYZ &check_point){
	//Eigen::Affine3f* transform_rotate = new Eigen::Affine3f(pcl::getTransformation(-position(0), -position(1), -position(2), 0, 0, 0));
	pcl::PointXYZ* point_in_zero = new pcl::PointXYZ(pcl::transformPoint(check_point, *jump_to_zero));
	//pcl::PointXYZ* point_in_zero = new pcl::PointXYZ(pcl::transformPoint(check_point, *transform_rotate));

	float minx = -width / 2.0f;
	float maxx = width / 2.0f;

	float miny = -lenght / 2.0f;
	float maxy = lenght / 2.0f;

	float minz = -height / 2.0f;
	float maxz = height / 2.0f;

	std::cout << "x:" << minx << " < " << point_in_zero->x << " < " << maxx << std::endl;
	std::cout << "y:" << miny << " < " << point_in_zero->y << " < " << maxy << std::endl;
	std::cout << "z:" << minz << " < " << point_in_zero->z << " < " << maxz << std::endl;

	if (minx <= point_in_zero->x && point_in_zero->x <= maxx && 
		miny <= point_in_zero->y && point_in_zero->y <= maxy && 
		minz <= point_in_zero->z && point_in_zero->z <= maxz){
		std::cout << " Is Inside " << std::endl;
		return true;
	}
	else{
		std::cout << " Is Outside " << std::endl;
		return false;
	}

	delete point_in_zero;
}

void StoredObject::find_bbox(){
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr zero_point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr minimal_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::MomentOfInertiaEstimation<pcl::PointXYZ>* feature_extractor = new pcl::MomentOfInertiaEstimation<pcl::PointXYZ>();
	std::vector<float> moment_of_inertia;
	std::vector<float> eccentricity;

	Eigen::Vector3f mass_center = Eigen::Vector3f();

	float cur_x_width, cur_y_lenght;
	float cur_square;

	double cloud_point[3];

	feature_extractor->setInputCloud(object_cloud);
	feature_extractor->compute();
	feature_extractor->getMassCenter(mass_center);

	vtkBoundingBox* boundingBox = new vtkBoundingBox();
	for (int i = 0; i < object_cloud->size() - 1; i++){
		cloud_point[0] = object_cloud->points[i].x;
		cloud_point[1] = object_cloud->points[i].y;
		cloud_point[2] = object_cloud->points[i].z;
		boundingBox->AddPoint(cloud_point);
	}

	cur_x_width = boundingBox->GetBound(1) - boundingBox->GetBound(0);
	cur_y_lenght = boundingBox->GetBound(3) - boundingBox->GetBound(2);
	height = boundingBox->GetBound(5) - boundingBox->GetBound(4);

	Eigen::Affine3f* move_to_zero = new Eigen::Affine3f(pcl::getTransformation(-mass_center(0), -mass_center(1), -mass_center(2), 0, 0, 0));
	pcl::transformPointCloud(*object_cloud, *zero_point_cloud, *move_to_zero);

	int step_count = (int)((float)max_degree / (float)step_degree) * ((float)max_degree / (float)step_degree) * ((float)max_degree / (float)step_degree);
	int i = 0;

	int y_pitch = 0;
	int x_roll = 0;

	for (int z_yaw = 0; z_yaw < max_degree; z_yaw += step_degree){
		i++;
		Eigen::Affine3f* transform_rotate = new Eigen::Affine3f(pcl::getTransformation(0, 0, 0, DEG2RAD(x_roll), DEG2RAD(y_pitch), DEG2RAD(z_yaw)));
		pcl::transformPointCloud(*zero_point_cloud, *minimal_cloud, *transform_rotate);

		vtkBoundingBox* bBox = new vtkBoundingBox();
		for (int i = 0; i < minimal_cloud->size(); i++){
			cloud_point[0] = minimal_cloud->points[i].x;
			cloud_point[1] = minimal_cloud->points[i].y;
			cloud_point[2] = minimal_cloud->points[i].z;
			bBox->AddPoint(cloud_point);
		}

		cur_x_width = bBox->GetBound(1) - bBox->GetBound(0);
		cur_y_lenght = bBox->GetBound(3) - bBox->GetBound(2);

		cur_square = cur_x_width * cur_y_lenght;

		if (square > cur_square){
			square = cur_square;

			width = cur_x_width;
			lenght = cur_y_lenght;

			yaw = z_yaw;
		}

		delete bBox;
	}

	position = Eigen::Vector3f(mass_center(0), mass_center(1), mass_center(2));
	CalcJamp();
	
	delete boundingBox;
	delete feature_extractor;
}

void StoredObject::Remove(){
	//Mark as removed
	removed = true;

	//Set removed time
	time_t nowtimesec;
	time(&nowtimesec);
	RemovedDate = nowtimesec;
}

void StoredObject::CalcJamp(){
	Eigen::Matrix3f rotational_matrix_OBB3f = Eigen::Matrix3f();
	Eigen::Matrix4f rotational_matrix_OBB4f = Eigen::Matrix4f();
	Eigen::Matrix3f rotational_matrix_ZERO3f = Eigen::Matrix3f();
	Eigen::Matrix4f rotational_matrix_ZERO4f = Eigen::Matrix4f();
	
	//jump_to_zero = new Eigen::Affine3f(pcl::getTransformation(-position(0), -position(1), -position(2), DEG2RAD(-roll), DEG2RAD(-pitch), DEG2RAD(-yaw)));
	jump_to_zero = new Eigen::Affine3f(pcl::getTransformation(-position(0), -position(1), -position(2), DEG2RAD(0), DEG2RAD(0), DEG2RAD(0)));
	jump_to_bbox = new Eigen::Affine3f(pcl::getTransformation(position(0), position(1), position(2), DEG2RAD(roll), DEG2RAD(pitch), DEG2RAD(yaw)));

	rotational_matrix_OBB4f = jump_to_bbox->matrix();
	rotational_matrix_ZERO4f = jump_to_zero->matrix();

	rotational_matrix_OBB3f(0, 0) = rotational_matrix_OBB4f(0); //FIXME
	rotational_matrix_OBB3f(0, 1) = rotational_matrix_OBB4f(1); //FIXME
	rotational_matrix_OBB3f(0, 2) = rotational_matrix_OBB4f(2); //FIXME
	rotational_matrix_OBB3f(1, 0) = rotational_matrix_OBB4f(4); //FIXME
	rotational_matrix_OBB3f(1, 1) = rotational_matrix_OBB4f(5); //FIXME
	rotational_matrix_OBB3f(1, 2) = rotational_matrix_OBB4f(6); //FIXME
	rotational_matrix_OBB3f(2, 0) = rotational_matrix_OBB4f(8); //FIXME
	rotational_matrix_OBB3f(2, 1) = rotational_matrix_OBB4f(9); //FIXME
	rotational_matrix_OBB3f(2, 2) = rotational_matrix_OBB4f(10); //FIXME

	rotational_matrix_ZERO3f(0, 0) = rotational_matrix_ZERO4f(0); //FIXME
	rotational_matrix_ZERO3f(0, 1) = rotational_matrix_ZERO4f(1); //FIXME
	rotational_matrix_ZERO3f(0, 2) = rotational_matrix_ZERO4f(2); //FIXME
	rotational_matrix_ZERO3f(1, 0) = rotational_matrix_ZERO4f(4); //FIXME
	rotational_matrix_ZERO3f(1, 1) = rotational_matrix_ZERO4f(5); //FIXME
	rotational_matrix_ZERO3f(1, 2) = rotational_matrix_ZERO4f(6); //FIXME
	rotational_matrix_ZERO3f(2, 0) = rotational_matrix_ZERO4f(8); //FIXME
	rotational_matrix_ZERO3f(2, 1) = rotational_matrix_ZERO4f(9); //FIXME
	rotational_matrix_ZERO3f(2, 2) = rotational_matrix_ZERO4f(10); //FIXME

	quaternion_to_bbox = Eigen::Quaternionf(rotational_matrix_OBB3f); //FIXME
	quaternion_to_zero = Eigen::Quaternionf(rotational_matrix_ZERO3f); //FIXME
}