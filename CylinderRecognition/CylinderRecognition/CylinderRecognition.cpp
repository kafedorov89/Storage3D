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
	
	pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
	

	// Read in the cloud data
	reader.read("PointCloud.pcd", *cloud);
	std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

	// Apply foxel filtration
	float voxeldensity = 0.03;
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(voxeldensity, voxeldensity, voxeldensity); //FIXME. Need to create function with flexeble values for leaf cloud size (Cloud, Count point per dimentions -> BoundingBox -> dimentions -> setLeafSize -> LightCloud)
	vg.filter(*cloud_filtered);
	cloud.swap(cloud_filtered);

	writer.write("voxeled.pcd", *cloud, false);

	// Build a passthrough filter to remove spurious NaNs
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0, 5);
	pass.filter(*cloud_filtered);
	std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

	writer.write("filtered.pcd", *cloud_filtered, false);

	// Estimate point normals
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud_filtered);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);

	// Create the segmentation object for the planar model and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	seg.setNormalDistanceWeight(0.1);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.05);
	seg.setInputCloud(cloud_filtered);
	seg.setInputNormals(cloud_normals);
	// Obtain the plane inliers and coefficients
	seg.segment(*inliers_plane, *coefficients_plane);
	std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

	// Extract the planar inliers from the input cloud
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);

	// Write the planar inliers to disk
	pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
	extract.filter(*cloud_plane);
	std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;
	writer.write("plane.pcd", *cloud_plane, false);

	// Remove the planar inliers, extract the rest
	extract.setNegative(true);
	extract.filter(*cloud_filtered2);
	extract_normals.setNegative(true);
	extract_normals.setInputCloud(cloud_normals);
	extract_normals.setIndices(inliers_plane);
	extract_normals.filter(*cloud_normals2);

	writer.write("filtered2.pcd", *cloud_filtered2, false);

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
		seg.setNormalDistanceWeight(0.1);
		seg.setMaxIterations(10000);
		seg.setDistanceThreshold(0.05);
		seg.setRadiusLimits(0.1, 0.4);
		seg.setInputCloud(cloud_filtered2);
		seg.setInputNormals(cloud_normals2);

		// Obtain the cylinder inliers and coefficients
		seg.segment(*inliers_i_cylinder, *coefficients_i_cylinder);
		//std::cerr << "Cylinder coefficients: " << *coefficients_icylinder << std::endl;

		extract.setInputCloud(cloud_filtered2);
		extract.setIndices(inliers_i_cylinder);
		extract.setNegative(false);
		extract.filter(*cloud_i_cylinder);
		
		// Remove i-cylinder from point cloud
		extract.setNegative(true);
		extract.filter(*cloud_filtered_i);
		
		// Remove i-cylinder point from normal's point cloud
		extract_normals.setNegative(true);
		extract_normals.setInputCloud(cloud_normals2);
		extract_normals.setIndices(inliers_i_cylinder);
		extract_normals.filter(*cloud_normals_i);

		cloud_filtered2.swap(cloud_filtered_i);
		cloud_normals2.swap(cloud_normals_i);

		if (cloud_i_cylinder->points.empty()){
			std::cout << "Can't find the cylindrical component. Cycle will stop." << std::endl;
			break;
		}
		else
		{
			std::cerr << "In point cloud was found cylinder: " << i << std::endl << "Points count: " << cloud_i_cylinder->points.size() << std::endl;

			//Write i-cylinder to disk
			std::stringstream ss;
			ss << "cylinder" << i << ".pcd";
			writer.write(ss.str(), *cloud_i_cylinder, false);
		}
		i++;
	}

	return (0);
}

