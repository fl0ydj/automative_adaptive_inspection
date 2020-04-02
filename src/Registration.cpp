#include "Registration.h"
Registration::Registration(std::string debug) {
	this->debug = stoi(debug);
	this->search_method_xyz_.reset(new pcl::search::KdTree<PT>);
}
void Registration::downsample(PTC::Ptr cloud,PTC::Ptr cloud_sparse, double& leafSize, int max_numberOfPoints) {
	pcl::VoxelGrid<PT> grid;
	grid.setLeafSize(leafSize, leafSize, leafSize);
	grid.setInputCloud(cloud);
	grid.filter(*cloud_sparse);
	while (cloud_sparse->width > max_numberOfPoints) {
		leafSize *= 0.1;
		PCL_WARN("There are too many points after downsampling. Leafsize is increased to "); PCL_WARN(std::to_string(leafSize).c_str()); PCL_WARN("!\n");
		grid.setLeafSize(leafSize, leafSize, leafSize);
		grid.setInputCloud(cloud_sparse);
		grid.filter(*cloud_sparse);
	}
}
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
	norm_est.setRadiusSearch(searchRadius); //number of Points used
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
	pcl::SampleConsensusInitialAlignment<PT, PT, pcl::FPFHSignature33> sac_ia_; //6 min für Kamera1 //VIELLEICHT GIBT ES DA EIN LOKALES MINIMUM?? -> 
	sac_ia_.setMinSampleDistance(minSampleDistance);
	sac_ia_.setMaxCorrespondenceDistance(correspondenceDistance);
	sac_ia_.setMaximumIterations(maxIterations);
	sac_ia_.setTargetFeatures(getLocalFeatures(cloud_target, target_normals,searchRadius));
	sac_ia_.setSourceFeatures(getLocalFeatures(cloud_source, source_normals, searchRadius));
	sac_ia_.setInputTarget(cloud_target); //comp
	sac_ia_.setInputCloud(cloud_source);//cam
	sac_ia_.align(*cloud_source); //SCHAFFT ER NICHT //PCL_WARN("Initial Registration failed. Check alignment manually! Consider doing a new recording.");
	transformation = sac_ia_.getFinalTransformation();
	score = sac_ia_.getFitnessScore();
	return sac_ia_.hasConverged();
}
bool Registration::cloudRegistration(PTC::Ptr cloud_source, pcl::PointCloud<pcl::Normal>::Ptr source_normals, PTC::Ptr cloud_target, pcl::PointCloud<pcl::Normal>::Ptr target_normals, double& score, Eigen::Matrix4f& transformation,double correspondenceDistance, int maxIterations, double epsilon) {
	pcl::PointCloud<pcl::PointNormal>::Ptr source_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr target_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud_source, *source_normals, *source_with_normals);
	pcl::concatenateFields(*cloud_target, *target_normals, *target_with_normals);
	pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp; //uses point to plane instead of point to point
	icp.setMaxCorrespondenceDistance(correspondenceDistance);
	//vlt http://pointclouds.org/documentation/tutorials/random_sample_consensus.php?
	icp.setMaximumIterations(maxIterations); //oder weniger hier
	if (epsilon > 0)
		icp.setTransformationEpsilon(epsilon);
	icp.setInputSource(source_with_normals);
	icp.setInputTarget(target_with_normals); //bisschen weniger dense!
	icp.align(*source_with_normals); //here guess for initial transformation
	transformation= icp.getFinalTransformation();
	score = icp.getFitnessScore();
	pcl::transformPointCloud(*cloud_source, *cloud_source, transformation);
	return icp.hasConverged();
}
