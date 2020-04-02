#ifndef REGISTRATION_H_
#define REGISTRATION_H_
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/filters/passthrough.h>
#include <pcl/keypoints/harris_3d.h>
#include "Visual.h"
typedef pcl::PointXYZ PT;
typedef pcl::PointCloud<PT> PTC;
class Registration
{
private:
	int debug;
	pcl::search::KdTree<PT>::Ptr search_method_xyz_;
public:
	/*
	* Constructor of the registration class.
	* @param component: the cloud of the component [input]
	* @return void
	*/
    Registration(std::string debug); 
	/*
	* Uses a voxelgrid with leafize equal to 5mm to downsample a given point cloud.
	* @param cloud: the cloud to be downsampled [input]
	* @param cloud_sparse: the downsampled cloud [output]
	* @return void
	*/
    void downsample(PTC::Ptr cloud, PTC::Ptr cloud_sparse, double& leafSize,int max_numberOfPoints=std::numeric_limits<int>::max());
	pcl::PointCloud<pcl::Normal>::Ptr getNormals(PTC::Ptr cloud, double searchRadius);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr Registration::getLocalFeatures(PTC::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, double searchRadius);

	/*
	* Find the initial 6D pose of the component in the cam cloud pointer using FPFH.
	* @return void
	*/
	bool Registration::findInitialGuess(PTC::Ptr cloud_source, pcl::PointCloud<pcl::Normal>::Ptr source_normals, PTC::Ptr cloud_target, pcl::PointCloud<pcl::Normal>::Ptr target_normals, double& score, Eigen::Matrix4f& transformation, double minSampleDistance, double correspondenceDistance, int maxIterations, double searchRadius);
	/*
	* Calculate the centroid of the cam cloud and use it to demean cam
	* @return void
	*/
    void demean(PTC::Ptr cloudCentroid, PTC::Ptr cloudDemean, bool inverse=false);
	/*
* Performs the pose refinement using ICP. Good initial pose estimation needed.
* @return void
*/
	bool cloudRegistration(PTC::Ptr cloud_source, pcl::PointCloud<pcl::Normal>::Ptr source_normals, PTC::Ptr cloud_target, pcl::PointCloud<pcl::Normal>::Ptr target_normals, double& score, Eigen::Matrix4f& transformation,double correspondenceDistance,int maxIterations, double epsilon=0);
    
};
#endif