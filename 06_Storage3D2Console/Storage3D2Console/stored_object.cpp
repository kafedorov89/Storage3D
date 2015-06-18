#pragma once
#include "stdafx.h"
#include "stored_object.h"

using namespace std;

StoredObject::StoredObject(){

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
	pcl::PointXYZ point_in_zero;
	point_in_zero = pcl::transformPoint(check_point, jump_to_zero);

	if (((-width / 2.0f) <= point_in_zero.x) && (point_in_zero.x <= (width / 2.0f)) && ((-lenght / 2.0f) <= point_in_zero.y) && (point_in_zero.y <= (lenght / 2.0f)) && ((-height / 2.0f) <= point_in_zero.z) && (point_in_zero.z <= (height / 2.0f))){
		std::cout << " Is Inside " << std::endl;
		return true;
	}
	else{
		std::cout << " Is Outside " << std::endl;
		return false;
	}
}

void StoredObject::find_bbox(){
	pcl::PointCloud<pcl::PointXYZ>::Ptr light_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr zero_point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr minimal_cloud(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;

	std::vector <float> moment_of_inertia;
	std::vector <float> eccentricity;

	Eigen::Matrix3f rotational_matrix_OBB3f;
	Eigen::Matrix4f rotational_matrix_OBB4f;

	Eigen::Vector3f mass_center;

	float cur_z_height, cur_x_width, cur_y_lenght;// cur_xy_square;
	float cur_volume;

	double cloud_point[3];

	pcl::VoxelGrid<pcl::PointXYZ> vg;

	feature_extractor.setInputCloud(object_cloud);
	feature_extractor.compute();
	feature_extractor.getMassCenter(mass_center);

	vtkBoundingBox boundingBox;
	for (int i = 0; i < object_cloud->size() - 1; i++){
		cloud_point[0] = object_cloud->points[i].x;
		cloud_point[1] = object_cloud->points[i].y;
		cloud_point[2] = object_cloud->points[i].z;
		boundingBox.AddPoint(cloud_point);
	}

	cur_x_width = boundingBox.GetBound(1) - boundingBox.GetBound(0);
	cur_y_lenght = boundingBox.GetBound(3) - boundingBox.GetBound(2);
	cur_z_height = boundingBox.GetBound(5) - boundingBox.GetBound(4);

	vg.setInputCloud(object_cloud);
	vg.setLeafSize(cur_x_width / grid_density, cur_y_lenght / grid_density, cur_z_height / grid_density);

	vg.filter(*light_cloud);

	Eigen::Affine3f move_to_zero = pcl::getTransformation(-mass_center(0), -mass_center(1), -mass_center(2), 0, 0, 0);
	pcl::transformPointCloud(*light_cloud, *zero_point_cloud, move_to_zero);

	int step_count = (int)((float)max_degree / (float)step_degree) * ((float)max_degree / (float)step_degree) * ((float)max_degree / (float)step_degree);
	int i = 0;
	for (int z_yaw = 0; z_yaw < max_degree; z_yaw += step_degree){
		for (int y_pitch = 0; y_pitch < max_degree; y_pitch += step_degree){
			for (int x_roll = 0; x_roll < max_degree; x_roll += step_degree){
				i++;
				std::cout << step_count - i << std::endl << std::endl;
				Eigen::Affine3f transform_rotate = pcl::getTransformation(0, 0, 0, DEG2RAD(x_roll), DEG2RAD(y_pitch), DEG2RAD(z_yaw));
				//Eigen::Affine3f transform_rotate = pcl::getTransformation(0, 0, 0, DEG2RAD(x_roll), 0, DEG2RAD(z_yaw));
				pcl::transformPointCloud(*zero_point_cloud, *minimal_cloud, transform_rotate);

				vtkBoundingBox bBox;
				for (int i = 0; i < minimal_cloud->size() - 1; i++){
					cloud_point[0] = minimal_cloud->points[i].x;
					cloud_point[1] = minimal_cloud->points[i].y;
					cloud_point[2] = minimal_cloud->points[i].z;
					bBox.AddPoint(cloud_point);
				}

				cur_x_width = bBox.GetBound(1) - bBox.GetBound(0);
				cur_y_lenght = bBox.GetBound(3) - bBox.GetBound(2);
				cur_z_height = bBox.GetBound(5) - bBox.GetBound(4);

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
			}
		}
	}

	jump_to_zero = pcl::getTransformation(-mass_center(0), -mass_center(1), -mass_center(2), DEG2RAD(roll), DEG2RAD(pitch), DEG2RAD(yaw));

	jump_to_bbox = pcl::getTransformation(mass_center(0), mass_center(1), mass_center(2), DEG2RAD(-roll), DEG2RAD(-pitch), DEG2RAD(-yaw));
	rotational_matrix_OBB4f = jump_to_bbox.matrix();
	rotational_matrix_OBB3f(0) = rotational_matrix_OBB4f(0);
	rotational_matrix_OBB3f(1) = rotational_matrix_OBB4f(1);
	rotational_matrix_OBB3f(2) = rotational_matrix_OBB4f(2);
	rotational_matrix_OBB3f(3) = rotational_matrix_OBB4f(4);
	rotational_matrix_OBB3f(4) = rotational_matrix_OBB4f(5);
	rotational_matrix_OBB3f(5) = rotational_matrix_OBB4f(6);
	rotational_matrix_OBB3f(6) = rotational_matrix_OBB4f(8);
	rotational_matrix_OBB3f(7) = rotational_matrix_OBB4f(9);
	rotational_matrix_OBB3f(8) = rotational_matrix_OBB4f(10);

	position = Eigen::Vector3f(mass_center(0), mass_center(1), mass_center(2));
	quaternion_to_bbox = Eigen::Quaternionf(rotational_matrix_OBB3f);
}

void StoredObject::Remove(){
	//Mark as removed
	this->removed = true;

	//Set removed time
	time_t nowtimesec;
	std::asctime(std::localtime(&nowtimesec));
	this->RemovedDate = nowtimesec;
}