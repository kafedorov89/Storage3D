// CylinderRecognition.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZ PointT;

int _tmain(int argc, _TCHAR* argv[])
{
	// All the objects needed
	pcl::PCDReader reader;
	pcl::PassThrough<PointT> pass;
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	pcl::PCDWriter writer;
	pcl::ExtractIndices<PointT> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

	// Datasets
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

	// Read in the cloud data
	reader.read("PointCloud.pcd", *cloud);
	std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

	// Apply foxel filtration
	float voxeldensity = 0.02;
	
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(voxeldensity, voxeldensity, voxeldensity); //FIXME. Need to create function with flexeble values for leaf cloud size (Cloud, Count point per dimentions -> BoundingBox -> dimentions -> setLeafSize -> LightCloud)
	vg.filter(*cloud_filtered);
	cloud.swap(cloud_filtered);

	writer.write("voxeled.pcd", *cloud, false);

	// Build a passthrough filter to remove spurious NaNs
	float minZ = 0;
	float maxZ = 5;
	
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(minZ, maxZ);
	pass.filter(*cloud_filtered);
	std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

	writer.write("filtered.pcd", *cloud_filtered, false);

	// Estimate point normals
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud_filtered);
	ne.setKSearch(10);
	ne.compute(*cloud_normals);

	
	//Cycle for find planes

	int min_plane_size = 200;
	float planeNormalDistanceWeight = 0.1;
	int planeMaxIterations = 1000;
	float planeDistanceThreshold = voxeldensity * 2;

	int k = 0;
	while (true){
		pcl::PointCloud<PointT>::Ptr cloud_filtered_k(new pcl::PointCloud<PointT>);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_k(new pcl::PointCloud<pcl::Normal>);
	
		pcl::PointCloud<PointT>::Ptr cloud_k_plane(new pcl::PointCloud<PointT>());
		pcl::ModelCoefficients::Ptr coefficients_k_plane(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers_k_plane(new pcl::PointIndices);

		// Create the segmentation object for the planar model and set all the parameters
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);

		seg.setNormalDistanceWeight(planeNormalDistanceWeight);
		seg.setMaxIterations(planeMaxIterations);
		seg.setDistanceThreshold(planeDistanceThreshold);

		seg.setInputCloud(cloud_filtered);
		seg.setInputNormals(cloud_normals);

		/*seg.setInputNormals(cloud_normals);
		seg.setInputCloud(cloud_filtered);
		seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(1000);
		seg.setDistanceThreshold(0.01);
		seg.setAxis(Eigen::Vector3f(0, 0, 1));
		seg.setEpsAngle(0.02);*/

		// Obtain the plane inliers and coefficients
		seg.segment(*inliers_k_plane, *coefficients_k_plane);
		//std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

		// Extract the planar inliers from the input cloud
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers_k_plane);
		extract.setNegative(false);
		extract.filter(*cloud_k_plane);

		if (cloud_k_plane->points.empty() || cloud_k_plane->points.size() < min_plane_size){
			std::cout << "Can't find the plane component. Cycle will stop." << std::endl;
			
			writer.write("filtered2.pcd", *cloud_filtered, false);

			break;
		}
		else{
			// Write the planar inliers to disk
			std::cerr << "In point cloud was found plane: " << k << std::endl << "Points count: " << cloud_k_plane->points.size() << std::endl;

			//Write k-plane to disk
			std::stringstream ss;
			ss << "plane_" << k << ".pcd";
			writer.write(ss.str(), *cloud_k_plane, false);
		}
		//writer.write("plane.pcd", *cloud_k_plane, false);

		// Remove the planar inliers, extract the rest
		extract.setNegative(true);
		extract.filter(*cloud_filtered_k);
		extract_normals.setNegative(true);
		extract_normals.setInputCloud(cloud_normals);
		extract_normals.setIndices(inliers_k_plane);
		extract_normals.filter(*cloud_normals_k);

		cloud_filtered.swap(cloud_filtered_k);
		cloud_normals.swap(cloud_normals_k);

		//writer.write("filtered2.pcd", *cloud_filtered2, false);
		k++;
	}

	
	//Cycle for find cylinders

	float cylinderNormalDistanceWeight = 0.1;
	int cylinderMaxIterations = 1000;
	float DistanceThreshold = voxeldensity * 2;
	float cylinderMinRadius = 0.01;
	float cylinderMaxRadius = 0.2;
	int min_cylinder_size = 250;
	
	int i = 0;
	while (true){
		pcl::PointCloud<PointT>::Ptr cloud_i_cylinder(new pcl::PointCloud<PointT>());
		pcl::ModelCoefficients::Ptr coefficients_i_cylinder(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers_i_cylinder(new pcl::PointIndices);
		
		pcl::PointCloud<PointT>::Ptr cloud_filtered_i(new pcl::PointCloud<PointT>);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_i(new pcl::PointCloud<pcl::Normal>);
		
		// Create the segmentation object for cylinder segmentation and set all the parameters
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_CYLINDER);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setNormalDistanceWeight(cylinderNormalDistanceWeight);
		seg.setMaxIterations(cylinderMaxIterations);
		seg.setDistanceThreshold(DistanceThreshold);
		seg.setRadiusLimits(cylinderMinRadius, cylinderMaxRadius);
		seg.setInputCloud(cloud_filtered);
		seg.setInputNormals(cloud_normals);

		// Obtain the cylinder inliers and coefficients
		seg.segment(*inliers_i_cylinder, *coefficients_i_cylinder);
		//std::cerr << "Cylinder coefficients: " << *coefficients_icylinder << std::endl;

		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers_i_cylinder);
		extract.setNegative(false);
		extract.filter(*cloud_i_cylinder);
		
		// Remove i-cylinder from point cloud
		extract.setNegative(true);
		extract.filter(*cloud_filtered_i);
		
		// Remove i-cylinder point from normal's point cloud
		extract_normals.setNegative(true);
		extract_normals.setInputCloud(cloud_normals);
		extract_normals.setIndices(inliers_i_cylinder);

		extract_normals.filter(*cloud_normals_i);

		cloud_filtered.swap(cloud_filtered_i);
		cloud_normals.swap(cloud_normals_i);

		if (cloud_i_cylinder->points.empty() || cloud_i_cylinder->points.size() < min_cylinder_size){
			std::cout << "Can't find the cylindrical component. Cycle will stop." << std::endl;
			break;
		}
		else
		{
			std::cerr << "In point cloud was found cylinder: " << i << std::endl << "Points count: " << cloud_i_cylinder->points.size() << std::endl;

			//Write i-cylinder to disk
			std::stringstream ss;
			ss << "cylinder_" << i << ".pcd";
			writer.write(ss.str(), *cloud_i_cylinder, false);
		}
		i++;
	}

	return (0);
}

