/*
Copyright (c) 2020
TU Berlin, Institut für Werkzeugmaschinen und Fabrikbetrieb
Fachgebiet Industrielle Automatisierungstechnik
Authors: Justin Heinz
All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and /or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
DISCLAIMER: THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/filters/passthrough.h>
#include <pcl/keypoints/harris_3d.h>
#include "Visual.h"
typedef pcl::PointXYZ PT;
typedef pcl::PointCloud<PT> PTC;
/**@brief Class for the subproblem of the registration.

This class allows postprocessing functionalities, estimation of normals, downsampling and performing the registration using SAC Initial Alignment or ICP. 
Source: http://pointclouds.org/documentation/tutorials/template_alignment.php
@author Justin Heinz
*/
class Registration
{
private:
	/**Debug level (0 - none,1 - some,2 - all)*/
	int debug;
	/**Search tree used for normals and FPFH estimation*/
	pcl::search::KdTree<PT>::Ptr search_method_xyz_;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	/** Constructor of the registration class.
	 @param[in] debug - how many debug info should be displayed (0 - none,1 - some,2 - all)
	*/
    Registration(std::string debug); 
	/** Uses a voxelgrid to downsample a given point cloud.
	@param[in] cloud - the cloud to be downsampled
	@param[out] cloud_sparse - the downsampled cloud
	@param[in] leafSize - Leafsize to use for the voxel grid
	@param[in] max_numberOfPoints - maximum number of points in the cloud after downsampling. When the number is higher, the leafsize is increased and the downsampling is performed again.
	*/
	template<typename T>
    void downsample(typename pcl::PointCloud<T>::Ptr cloud, typename pcl::PointCloud<T>::Ptr cloud_sparse, double& leafSize,int max_numberOfPoints=std::numeric_limits<int>::max());
	/**Uses a statistical outlier removal filter to postprocess clouds.
	@param[in,out] cloud - the cloud to postprocess
	@param[in] meanK - the number of neighbors to use for calculating the mean
	@param[in] thresh - the maximum allowed distance of a point with respect to the calculated mean as a multiple of the standard deviation*/
	template<typename T>
	void postprocess(typename pcl::PointCloud<T>::Ptr cloud, int meanK, double thresh);
	/**Estimate the normals for a cloud.
	@param[in] cloud - cloud used for normal estimation
	@param[in] searchRadius - used for normal estimation
	@return
	* pointer to calculated normals
	*/
	pcl::PointCloud<pcl::Normal>::Ptr getNormals(PTC::Ptr cloud, double searchRadius);
	/**Esimate the fast point feature histograms for a cloud
	@param[in] cloud - cloud used for FPFH estimation
	@param[in] cloud_normals - the normals of the cloud
	@param[in] searchRadius - the radius used for FPFH estimation
	@return
	* pointer to the estimated FPFHs
	*/
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr Registration::getLocalFeatures(PTC::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, double searchRadius);
	/**
	Find the initial 6D pose of the component in the cam cloud pointer using FPFH.
	Source: http://pointclouds.org/documentation/tutorials/template_alignment.php
	@param[in,out] cloud_source - cloud which has to be aligned
	@param[in] source_normals - normals of the source cloud
	@param[in] cloud_target - target cloud
	@param[in] target_normals - normals of the target cloud
	@param[out] score - fitness score of the registration
	@param[out] transformation - estimated transformation to align clouds
	@param[in] minSampleDistance - minimum distance between chosen samples
	@param[in] correspondenceDistance - maximum allowed initial distance between two features of the clouds
	@param[in] maxIterations - number of iterations used
	@param[in] searchRadius - used for the FPFH estimation
	*/
	bool findInitialGuess(PTC::Ptr cloud_source, pcl::PointCloud<pcl::Normal>::Ptr source_normals, PTC::Ptr cloud_target, pcl::PointCloud<pcl::Normal>::Ptr target_normals, double& score, Eigen::Matrix4f& transformation, double minSampleDistance, double correspondenceDistance, int maxIterations, double searchRadius);
	/**
	Calculate the centroid of a cloud and use it to demean a second cloud.
	@param[in] cloudCentroid - cloud used for the centroid calculation
	@param[in,out] cloudDemean - cloud to demean
	@param[in] inverse - If true, the calculated centroid is inverted. Assuming an already demeaned cloudDemean, this flag can be used to move cloudDemean to the centroid of cloudCentroid.
	*/
    void demean(PTC::Ptr cloudCentroid, PTC::Ptr cloudDemean, bool inverse=false);
	/**Performs the pose refinement using ICP. Good initial pose estimation needed.
	@param[in,out] cloud_source - cloud which has to be aligned
	@param[in] source_normals - normals of the source cloud
	@param[in] cloud_target - target cloud
	@param[in] target_normals - normals of the target cloud
	@param[out] score - fitness score of the registration
	@param[out] transformation - estimated transformation to align clouds
	@param[in] correspondenceDistance - maximum allowed initial distance between two features of the clouds
	@param[in] maxIterations - number of iterations used
	@param[in] maxFitnessScore - When greater than zero, this parameter is used as a second termination criterium. When the fitness score between two consecutive steps is below this value, ICP is considered to have converged.
	*/
	bool cloudRegistration(PTC::Ptr cloud_source, pcl::PointCloud<pcl::Normal>::Ptr source_normals, PTC::Ptr cloud_target, pcl::PointCloud<pcl::Normal>::Ptr target_normals, double& score, Eigen::Matrix4f& transformation,double correspondenceDistance,int maxIterations, double maxFitnessScore =0);
    
};
#endif