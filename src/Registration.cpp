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
#include "Registration.h"
Registration::Registration(std::string debug) {
	this->debug = stoi(debug);
	this->search_method_xyz_.reset(new pcl::search::KdTree<PT>);
}
template<typename T>
void Registration::downsample(typename pcl::PointCloud<T>::Ptr cloud, typename pcl::PointCloud<T>::Ptr  cloud_sparse, double& leafSize, int max_numberOfPoints) {
	pcl::VoxelGrid<T> grid;
	grid.setLeafSize(leafSize, leafSize, leafSize);
	grid.setInputCloud(cloud);
	grid.filter(*cloud_sparse);
	while (cloud_sparse->width > max_numberOfPoints) {
		leafSize *= 1.1;
		PCL_WARN("There are too many points after downsampling. Leafsize is increased to "); PCL_WARN(std::to_string(leafSize).c_str()); PCL_WARN("!\n");
		grid.setLeafSize(leafSize, leafSize, leafSize);
		grid.setInputCloud(cloud_sparse);
		grid.filter(*cloud_sparse);
	}
}
template void Registration::downsample<PT>(PTC::Ptr cloud, PTC::Ptr cloud_sparse, double& leafSize, int max_numberOfPoints);
template void Registration::downsample<pcl::PointXYZRGB>(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sparse, double& leafSize, int max_numberOfPoints);
template<typename T>
void Registration::postprocess(typename pcl::PointCloud<T>::Ptr cloud, int meanK, double thresh) {
	double leafSize = 0.0002; //0.1mm would lead to a warning of the voxel grid filter
	downsample<T>(cloud, cloud, leafSize);
	pcl::StatisticalOutlierRemoval<T> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(meanK);
	sor.setStddevMulThresh(thresh);
	sor.filter(*cloud);
}
template void Registration::postprocess<PT>(PTC::Ptr cloud, int meanK, double thresh);
template void Registration::postprocess<pcl::PointXYZRGB>(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int meanK, double thresh);
void Registration::demean(PTC::Ptr cloudCentroid, PTC::Ptr cloudDemean,bool inverse) {
	Eigen::Vector4f centroid;

	pcl::compute3DCentroid(*cloudCentroid, centroid);
	if (inverse)centroid = -centroid;
	pcl::demeanPointCloud(*cloudDemean, centroid, *cloudDemean);
}
pcl::PointCloud<pcl::Normal>::Ptr Registration::getNormals(PTC::Ptr cloud, double searchRadius)
{
	pcl::PointCloud<pcl::Normal>::Ptr normals_ = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<PT, pcl::Normal> norm_est;
	norm_est.setInputCloud(cloud);
	norm_est.setSearchMethod(search_method_xyz_);
	norm_est.setRadiusSearch(searchRadius);
	norm_est.compute(*normals_);
	if (debug > 0){
		Visual Visualizer("[DEBUG] Normals", cloud);
		Visualizer.addNormals(cloud, normals_);
		if (debug == 2)	Visualizer.processOutput();
	}
	return normals_;
}
pcl::PointCloud<pcl::FPFHSignature33>::Ptr Registration::getLocalFeatures(PTC::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,double searchRadius)
{
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr features_ = pcl::PointCloud<pcl::FPFHSignature33>::Ptr(new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
	fpfh_est.setInputNormals(cloud_normals);
	fpfh_est.setSearchSurface(cloud);
	fpfh_est.setInputCloud(cloud);
	fpfh_est.setSearchMethod(search_method_xyz_);
	fpfh_est.setRadiusSearch(searchRadius);
	fpfh_est.compute(*features_);
	return features_;
}
bool Registration::findInitialGuess(PTC::Ptr cloud_source, pcl::PointCloud<pcl::Normal>::Ptr source_normals, PTC::Ptr cloud_target, pcl::PointCloud<pcl::Normal>::Ptr target_normals, double& score, Eigen::Matrix4f& transformation,double minSampleDistance, double correspondenceDistance, int maxIterations, double searchRadius) {
	pcl::SampleConsensusInitialAlignment<PT, PT, pcl::FPFHSignature33> sac_ia_;
	sac_ia_.setMinSampleDistance(minSampleDistance);
	sac_ia_.setMaxCorrespondenceDistance(correspondenceDistance);
	sac_ia_.setMaximumIterations(maxIterations);
	sac_ia_.setTargetFeatures(getLocalFeatures(cloud_target, target_normals,searchRadius));
	sac_ia_.setSourceFeatures(getLocalFeatures(cloud_source, source_normals, searchRadius));
	sac_ia_.setInputTarget(cloud_target); //comp
	sac_ia_.setInputCloud(cloud_source);//cam
	sac_ia_.align(*cloud_source);
	transformation = sac_ia_.getFinalTransformation();
	score = sac_ia_.getFitnessScore();
	return sac_ia_.hasConverged();
}
bool Registration::cloudRegistration(PTC::Ptr cloud_source, pcl::PointCloud<pcl::Normal>::Ptr source_normals, PTC::Ptr cloud_target, pcl::PointCloud<pcl::Normal>::Ptr target_normals, double& score, Eigen::Matrix4f& transformation,double correspondenceDistance, int maxIterations, double maxFitnessScore) {
	pcl::PointCloud<pcl::PointNormal>::Ptr source_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr target_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud_source, *source_normals, *source_with_normals);
	pcl::concatenateFields(*cloud_target, *target_normals, *target_with_normals);
	pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp; //uses point to plane instead of point to point
	icp.setMaxCorrespondenceDistance(correspondenceDistance);
	icp.setMaximumIterations(maxIterations);
	if (maxFitnessScore > 0)
		icp.setEuclideanFitnessEpsilon(maxFitnessScore);
	icp.setInputSource(source_with_normals);
	icp.setInputTarget(target_with_normals);
	icp.align(*source_with_normals);
	transformation= icp.getFinalTransformation();
	score = icp.getFitnessScore();
	pcl::transformPointCloud(*cloud_source, *cloud_source, transformation);
	return icp.hasConverged();
}
