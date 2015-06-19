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
	object_cloud = storedobject.object_cloud;
	step_degree = storedobject.step_degree;
	max_degree = storedobject.max_degree;
	grid_density = storedobject.grid_density;
	roll = storedobject.roll;
	pitch = storedobject.pitch;
	yaw = storedobject.yaw;
	width = storedobject.width;
	lenght = storedobject.lenght;
	height = storedobject.height;
	width_max = storedobject.width_max;
	lenght_max = storedobject.lenght_max;
	height_max = storedobject.height_max;
	width_min = storedobject.width_min;
	lenght_min = storedobject.lenght_min;
	height_min = storedobject.height_min;
	volume = storedobject.volume;

	//position = new Eigen::Vector3f(*storedobject.position);
	//quaternion_to_bbox = storedobject.quaternion_to_bbox;//new Eigen::Quaternionf(*storedobject.quaternion_to_bbox);
	//jump_to_bbox = new Eigen::Affine3f(*storedobject.jump_to_bbox);
	//jump_to_zero = new Eigen::Affine3f(*storedobject.jump_to_zero);

	position = storedobject.position;
	quaternion_to_bbox = storedobject.quaternion_to_bbox;
	jump_to_bbox = storedobject.jump_to_bbox;
	jump_to_zero = storedobject.jump_to_zero;
}

StoredObject::StoredObject(int layerid, int storageid, int uid, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float limit_array[6], int stepdegree, int maxdegree, float griddensity)
{
	addedLayerID = layerid;
	storageUID = storageid;
	UID = uid;
	
	width = FLT_MAX;
	lenght = FLT_MAX;
	height = FLT_MAX;

	//Object size limit
	width_min = limit_array[0];
	lenght_min = limit_array[1];
	height_min = limit_array[2];

	width_max = limit_array[3];
	lenght_max = limit_array[4];
	height_max = limit_array[5];

	volume = FLT_MAX;

	step_degree = stepdegree;
	max_degree = maxdegree;
	object_cloud = cloud;

	yaw = 0; //Angle oZ in degrees
	roll = 0; //Angle oX in degrees
	pitch = 0; //Angle oX in degrees

	isValid = false;

	grid_density = griddensity;

	//find_bbox();
	//check_valid_object();
	//check_isinside_point(pcl::PointXYZ(0, 0, 0));
}

StoredObject::~StoredObject()
{
}

bool StoredObject::check_valid_object(){
	std::cout << " width = " << width << std::endl;
	std::cout << " lenght = " << lenght << std::endl;
	std::cout << " height = " << height << std::endl;

	if ((width > width_min && width < width_max) && (lenght > lenght_min && lenght < lenght_max) && (height > height_min && height < height_max)){
		std::cout << " Valid object" << std::endl;
		return true;
	}
	else{
		std::cout << " Not Valid object" << std::endl;
		return false;
	}
}

bool StoredObject::check_isinside_point(const pcl::PointXYZ &check_point){
	pcl::PointXYZ* point_in_zero = new pcl::PointXYZ(pcl::transformPoint(check_point, *jump_to_zero));

	if (((-width / 2.0f) <= point_in_zero->x) && 
		(point_in_zero->x <= (width / 2.0f)) && 
		((-lenght / 2.0f) <= point_in_zero->y) && 
		(point_in_zero->y <= (lenght / 2.0f)) && 
		((-height / 2.0f) <= point_in_zero->z) && 
		(point_in_zero->z <= (height / 2.0f))){
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
	pcl::PointCloud<pcl::PointXYZ>::Ptr light_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr zero_point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr minimal_cloud(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::MomentOfInertiaEstimation<pcl::PointXYZ>* feature_extractor = new pcl::MomentOfInertiaEstimation<pcl::PointXYZ>();

	std::vector<float> moment_of_inertia;
	std::vector<float> eccentricity;

	//Eigen::Matrix3f* rotational_matrix_OBB3f = new Eigen::Matrix3f();
	//Eigen::Matrix4f* rotational_matrix_OBB4f = new Eigen::Matrix4f();
	Eigen::Matrix3f rotational_matrix_OBB3f = Eigen::Matrix3f();
	Eigen::Matrix4f rotational_matrix_OBB4f = Eigen::Matrix4f();

	Eigen::Vector3f mass_center = Eigen::Vector3f();

	float cur_z_height, cur_x_width, cur_y_lenght;// cur_xy_square;
	float cur_volume;

	double cloud_point[3];

	pcl::VoxelGrid<pcl::PointXYZ>* vg = new pcl::VoxelGrid<pcl::PointXYZ>();

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
	cur_z_height = boundingBox->GetBound(5) - boundingBox->GetBound(4);

	vg->setInputCloud(object_cloud);
	vg->setLeafSize(cur_x_width / grid_density, cur_y_lenght / grid_density, cur_z_height / grid_density);

	vg->filter(*light_cloud);

	Eigen::Affine3f* move_to_zero = new Eigen::Affine3f(pcl::getTransformation(-mass_center(0), -mass_center(1), -mass_center(2), 0, 0, 0));
	pcl::transformPointCloud(*light_cloud, *zero_point_cloud, *move_to_zero);

	int step_count = (int)((float)max_degree / (float)step_degree) * ((float)max_degree / (float)step_degree) * ((float)max_degree / (float)step_degree);
	int i = 0;
	for (int z_yaw = 0; z_yaw < max_degree; z_yaw += step_degree){
		for (int y_pitch = 0; y_pitch < max_degree; y_pitch += step_degree){
			for (int x_roll = 0; x_roll < max_degree; x_roll += step_degree){
				i++;
				std::cout << step_count - i << std::endl << std::endl;
				Eigen::Affine3f* transform_rotate = new Eigen::Affine3f(pcl::getTransformation(0, 0, 0, DEG2RAD(x_roll), DEG2RAD(y_pitch), DEG2RAD(z_yaw)));
				//Eigen::Affine3f transform_rotate = pcl::getTransformation(0, 0, 0, DEG2RAD(x_roll), 0, DEG2RAD(z_yaw));
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
				cur_z_height = bBox->GetBound(5) - bBox->GetBound(4);

				cur_volume = cur_x_width * cur_y_lenght * cur_z_height;

				if (volume > cur_volume){
					volume = cur_volume;

					width = cur_x_width;
					lenght = cur_y_lenght;
					height = cur_z_height;

					roll = x_roll;
					pitch = y_pitch;
					yaw = z_yaw;
				}

				delete bBox;
			}
		}
	}

	jump_to_zero = new Eigen::Affine3f(pcl::getTransformation(-mass_center(0), -mass_center(1), -mass_center(2), DEG2RAD(roll), DEG2RAD(pitch), DEG2RAD(yaw)));
	jump_to_bbox = new Eigen::Affine3f(pcl::getTransformation(mass_center(0), mass_center(1), mass_center(2), DEG2RAD(-roll), DEG2RAD(-pitch), DEG2RAD(-yaw)));
	
	//rotational_matrix_OBB4f = new Eigen::Matrix4f(jump_to_bbox->matrix());
	rotational_matrix_OBB4f = jump_to_bbox->matrix();
	
	//float a = rotational_matrix_OBB4f[0];// (0);
	
	rotational_matrix_OBB3f(0, 0) = rotational_matrix_OBB4f(0); //FIXME
	rotational_matrix_OBB3f(0, 1) = rotational_matrix_OBB4f(1); //FIXME
	rotational_matrix_OBB3f(0, 2) = rotational_matrix_OBB4f(2); //FIXME
	rotational_matrix_OBB3f(1, 0) = rotational_matrix_OBB4f(4); //FIXME
	rotational_matrix_OBB3f(1, 1) = rotational_matrix_OBB4f(5); //FIXME
	rotational_matrix_OBB3f(1, 2) = rotational_matrix_OBB4f(6); //FIXME
	rotational_matrix_OBB3f(2, 0) = rotational_matrix_OBB4f(8); //FIXME
	rotational_matrix_OBB3f(2, 1) = rotational_matrix_OBB4f(9); //FIXME
	rotational_matrix_OBB3f(2, 2) = rotational_matrix_OBB4f(10); //FIXME 

	/*rotational_matrix_OBB3f[0] = rotational_matrix_OBB4f[0];
	rotational_matrix_OBB3f[1] = rotational_matrix_OBB4f[1];
	rotational_matrix_OBB3f[2] = rotational_matrix_OBB4f[2];
	rotational_matrix_OBB3f[3] = rotational_matrix_OBB4f[4];
	rotational_matrix_OBB3f[4] = rotational_matrix_OBB4f[5];
	rotational_matrix_OBB3f[5] = rotational_matrix_OBB4f[6];
	rotational_matrix_OBB3f[6] = rotational_matrix_OBB4f[8];
	rotational_matrix_OBB3f[7] = rotational_matrix_OBB4f[9];
	rotational_matrix_OBB3f[8] = rotational_matrix_OBB4f[10];*/

	/**rotational_matrix_OBB3f << rotational_matrix_OBB4f[0], rotational_matrix_OBB4f[1], rotational_matrix_OBB4f[2],
		rotational_matrix_OBB4f[4], rotational_matrix_OBB4f[5], rotational_matrix_OBB4f[6],
		rotational_matrix_OBB4f[8], rotational_matrix_OBB4f[9], rotational_matrix_OBB4f[10];*/

	/**rotational_matrix_OBB3f << rotational_matrix_OBB4f->eigenvalues[0], rotational_matrix_OBB4f->eigenvalues[1], rotational_matrix_OBB4f->eigenvalues[2],
		rotational_matrix_OBB4f->eigenvalues[4], rotational_matrix_OBB4f->eigenvalues[5], rotational_matrix_OBB4f->eigenvalues[6],
		rotational_matrix_OBB4f->eigenvalues[8], rotational_matrix_OBB4f->eigenvalues[9], rotational_matrix_OBB4f->eigenvalues[10];*/

	/**rotational_matrix_OBB3f << rotational_matrix_OBB4f->eigenvalues(0), rotational_matrix_OBB4f->eigenvalues(1), rotational_matrix_OBB4f->eigenvalues(2),
		rotational_matrix_OBB4f->eigenvalues(4), rotational_matrix_OBB4f->eigenvalues(5), rotational_matrix_OBB4f->eigenvalues(6),
		rotational_matrix_OBB4f->eigenvalues(8), rotational_matrix_OBB4f->eigenvalues(9), rotational_matrix_OBB4f->eigenvalues(10);*/

	/*rotational_matrix_OBB3f << rotational_matrix_OBB4f->eigenvalues()[0], rotational_matrix_OBB4f->eigenvalues()[1], rotational_matrix_OBB4f->eigenvalues()[2],
		rotational_matrix_OBB4f->eigenvalues()[4], rotational_matrix_OBB4f->eigenvalues()[5], rotational_matrix_OBB4f->eigenvalues()[6],
		rotational_matrix_OBB4f->eigenvalues()[8], rotational_matrix_OBB4f->eigenvalues()[9], rotational_matrix_OBB4f->eigenvalues()[10];*/

	//position = new Eigen::Vector3f(mass_center->x(), mass_center->y(), mass_center->z());
	//quaternion_to_bbox = new Eigen::Quaternionf(rotational_matrix_OBB3f); //FIXME

	position = Eigen::Vector3f(mass_center(0), mass_center(1), mass_center(2));
	quaternion_to_bbox = Eigen::Quaternionf(rotational_matrix_OBB3f); //FIXME

	delete move_to_zero;
	delete boundingBox;
	//delete rotational_matrix_OBB3f;
	//delete rotational_matrix_OBB4f;
	//delete mass_center;
	delete vg;
	delete feature_extractor;
}

void StoredObject::Remove(){
	//Mark as removed
	this->removed = true;

	//Set removed time
	time_t nowtimesec;
	std::asctime(std::localtime(&nowtimesec));
	this->RemovedDate = nowtimesec;
}