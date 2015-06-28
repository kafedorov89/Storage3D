#pragma once
#include "stdafx.h"

/*#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/search/search.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

//------------------------------------------------------------------

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/transforms.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>

#include <pcl/filters/statistical_outlier_removal.h>
*/
using namespace std;
#include <vector>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PointIndices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

void VoxelGridFiltration(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &voxeled_cloud, float voxeldensity);
void CloudNoizeFiltration(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud);
//void CloudPlaneFiltration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &planed_cloud, float distancethreshold, bool negative = false);
void CloudPlaneFiltration(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &planed_cloud, float clastertollerance = 0.01, int minclastersize = 20, int maxclastersize = 20000, float cloud_zstep = 0.01);
void CloudPlaneFiltrationPerp(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &planed_cloud, float distancethreshold, float zepsangle = 0.1, bool negative = false);
void FindClasters(pcl::PointCloud<pcl::PointXYZ>::Ptr deltacloud, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clastervector, float tolerance = 0.01, int minclastersize = 20, int maxclastersize = 20000);
float GetPointDelatZ(pcl::PointXYZ &new_Point, pcl::PointCloud<pcl::PointXY>::Ptr old2d_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr old_cloud, pcl::KdTreeFLANN<pcl::PointXY> &kdtree, int &oldpointindex);
float GetNPointsDelatZ(pcl::PointXYZ &new_Point, pcl::PointCloud<pcl::PointXY>::Ptr old2d_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr old_cloud, pcl::KdTreeFLANN<pcl::PointXY> &kdtree, int pointscount = 1);
void Get2DCloudFrom3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3d, pcl::PointCloud<pcl::PointXY>::Ptr &cloud2d);
bool str_to_bool(string boolstr);