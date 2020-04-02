#include "Comparison.h"
Comparison::Comparison(std::string debug) { this->debug = stoi(debug); }
/* CITATION: Oliver
* Calculates the distance for each point of a first cloud to its nearest point in the second cloud and assigns different point-colors depending on the distance.
* @param cloud_a_error_point_inidces: strores the indices of all found error points [output]
* @param hausdorff: returns the max found distance of all points [output]
* @return void
*/
bool Comparison::hausdorff(PTC::Ptr cam, PTC::Ptr comp, pcl::PointCloud<pcl::PointXYZRGB>::Ptr comp_hausdorff,std::vector<int>& defects,float& hausdorff, float& avg_dist, bool outlierDeletion)
{
	bool containsDefects=false;
	pcl::KdTreeFLANN<PT> tree_comp;
	tree_comp.setInputCloud(comp);
	float max_dist_a = -std::numeric_limits<float>::max();

	// threshold for deviation such as construction deviation or uncertancies 
	// --> we assume that everyting over this value is a bad spot
	float threshold_outlier = 0.01*0.01;
	float threshold_defect = 0.005*0.005; //2mm -> our proposed certainty
	float threshold_warning = 0.001*0.001;//1mm
	std::vector<int> outliers; //in cam 
	std::vector<float> _sqr_distances;
	// compute max distance
	for (size_t i = 0; i < (*cam).points.size(); ++i)
	{
		std::vector<int> indices(1);
		std::vector<float> sqr_distance(1);

		tree_comp.nearestKSearch((*cam).points[i], 1, indices, sqr_distance);
		defects.push_back(indices[0]);
		if (sqr_distance[0] > threshold_outlier&& outlierDeletion) {
			outliers.push_back(i);
			sqr_distance[0] = 0;
		}
		else {
			if (sqr_distance[0] > max_dist_a)
				max_dist_a = sqr_distance[0];
			pcl::PointXYZRGB point;
			if (sqr_distance[0] >= threshold_defect)	// mark red
			{
				point = pcl::PointXYZRGB(uint8_t(255), uint8_t(0), uint8_t(0));
				defects.push_back(indices[0]);
				containsDefects = true;
			}
			else if (sqr_distance[0] >= threshold_warning) // mark yellow
			{
				point = pcl::PointXYZRGB(uint8_t(255), uint8_t(200), uint8_t(0));
			}
			else // mark green
			{
				point = pcl::PointXYZRGB(uint8_t(0), uint8_t(255), uint8_t(0));
			}
			point.x = (*comp)[indices[0]].x;
			point.y = (*comp)[indices[0]].y;
			point.z = (*comp)[indices[0]].z;
			(*comp_hausdorff).push_back(point);
			_sqr_distances.push_back(sqr_distance[0]);
		}
	}
	cout << "-> Deleting outliers...";
	pcl::ExtractIndices<PT> extract;
	extract.setInputCloud(cam);
	pcl::PointIndices::Ptr outlierPtr(new pcl::PointIndices);
	outlierPtr->indices = outliers;
	extract.setIndices(outlierPtr);
	extract.setNegative(true);
	extract.filter(*cam);
	// compute mean distance
	float sum = 0;
	for (std::vector<float>::iterator it = _sqr_distances.begin(); it != _sqr_distances.end(); ++it)
		sum += *it;

	avg_dist = sum / _sqr_distances.size();
	hausdorff = std::sqrt(max_dist_a);
	return containsDefects;
}
bool Comparison::findCircle(PTC::Ptr plane, pcl::ModelCoefficients::Ptr planeCoeffs, PTC::Ptr circle,pcl::ModelCoefficients::Ptr coefficients, double distThresh, double radius, double radiusLowerDelta, double radiusUpperDelta, double epsCenter, Eigen::Vector3f* center) {
	PTC::Ptr projectedCloud(new PTC);
	PTC::Ptr remainingCloud(new PTC);
	Eigen::Quaternionf q;
	Eigen::Vector3f coeffAxis(planeCoeffs->values[0], planeCoeffs->values[1], planeCoeffs->values[2]);
	coeffAxis.normalize();
	q.setFromTwoVectors(coeffAxis, Eigen::Vector3f::UnitZ());
	pcl::RadiusOutlierRemoval<PT> ra;
	ra.setInputCloud(plane);
	ra.setRadiusSearch(0.002);
	ra.setMinNeighborsInRadius(100);
	ra.setNegative(true);
	ra.filter(*projectedCloud);
	pcl::transformPointCloud(*projectedCloud, *projectedCloud, (Eigen::Affine3f)q.toRotationMatrix());
	pcl::transformPoint(*center,*center, (Eigen::Affine3f)q.toRotationMatrix());
	int numberOfTries;
	int maxNumberOfTries = 10;
	for (numberOfTries = 0; numberOfTries < maxNumberOfTries; numberOfTries++) {
		if (numberOfTries <= maxNumberOfTries/2.0)
			pcl::copyPointCloud(*projectedCloud, *remainingCloud);
		pcl::SACSegmentationFromNormals<PT, pcl::Normal> seg;
		seg.setModelType(pcl::SACMODEL_CIRCLE2D);
		if (numberOfTries <= maxNumberOfTries - 1&&radius!=0)
			seg.setRadiusLimits((2.0 * numberOfTries) / maxNumberOfTries * radiusLowerDelta + radius, (2.0 * numberOfTries) / maxNumberOfTries * radiusUpperDelta + radius);
		cout << (2.0 * numberOfTries) / maxNumberOfTries * radiusLowerDelta + radius << (2.0 * numberOfTries) / maxNumberOfTries * radiusUpperDelta + radius;
		if (findFeature(seg, remainingCloud, pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>), distThresh,0,coefficients, circle)) {
			if(abs(coefficients->values[0]- (*center)[0]) <= epsCenter&& abs(coefficients->values[1] - (*center)[1]) <= epsCenter)
				break;
		}
		if (debug == 2) {
			cout << "[DEBUG] No feature within the radius threshold or center threshold have been found.\n";
			Visual Visualizer("[DEBUG] Circle found instead", remainingCloud, circle);
			Visualizer.processOutput();
			cout << "[DEBUG] Relaxing radius thresholds...";
			Visualizer.closeViewer();
		}
	}
	if (numberOfTries == maxNumberOfTries) {
			PCL_WARN("No valid circle found. Measured set to -1. Check the output.");
			return false;
	}
	int numOfPoints = 1000;
	double t = 0;
	for (int i = 0; i < numOfPoints; i++) {
		t += 2 * M_PI / numOfPoints;
		circle->push_back(PT(coefficients->values[0] + radius * cos(t), coefficients->values[1] + radius * sin(t), circle->points[0].z));
	}
	circle->push_back(PT(coefficients->values[0], coefficients->values[1], circle->points[0].z));
	pcl::transformPointCloud(*circle, *circle, (Eigen::Affine3f)q.toRotationMatrix().inverse());
	return true;
}
bool Comparison::findPlane(PTC::Ptr cloud, PTC::Ptr plane, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::Normal>::Ptr normals,double normal_weight, double distThresh, double epsAngle, Eigen::Vector3f* normalAxis, double epsDist, double distanceFromOrigin) {
	PTC::Ptr remainingCloud(new PTC);
	pcl::copyPointCloud(*cloud, *remainingCloud);
	int numberOfTries;
	int maxNumberOfTries = 10;
	pcl::SACSegmentationFromNormals<PT, pcl::Normal> seg;
	seg.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
	if(epsDist!=0)
		seg.setDistanceFromOrigin(distanceFromOrigin);
	if (epsAngle != 0) {
		seg.setAxis(*normalAxis);
		seg.setEpsAngle(epsAngle);
	}
	for (numberOfTries = 0; numberOfTries < maxNumberOfTries; numberOfTries++) {
		if (!findFeature(seg, remainingCloud, normals, distThresh, normal_weight, coefficients, plane)) {
			continue;
		}
		if (epsDist == 0)
			break;
		else {
			if (abs(abs(coefficients->values[3]) - abs(distanceFromOrigin)) <= epsDist)
				break;
		}
		if (debug == 2) {
			cout << "[DEBUG] No feature within the epsDist threshold have been found.\n";
			Visual Visualizer("[DEBUG] Plane found instead", remainingCloud, plane);
			Visualizer.processOutput();
			cout << "[DEBUG] Retrying...";
			Visualizer.closeViewer();
			cout << distanceFromOrigin<< coefficients->values[3];
		}
	}
	if (numberOfTries == maxNumberOfTries) {
		PCL_WARN("No valid plane found. Measured set to -1. Check the output.");
		return false;
	}
	pcl::ProjectInliers<PT> proj;
	proj.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
	proj.setInputCloud(plane);
	proj.setModelCoefficients(coefficients);
	proj.filter(*plane);
	return true;
}
bool Comparison::findFeature(pcl::SACSegmentationFromNormals<PT, pcl::Normal> seg, PTC::Ptr remainingCloud, pcl::PointCloud<pcl::Normal>::Ptr normals, double distThresh, double normalWeight, pcl::ModelCoefficients::Ptr coefficients, PTC::Ptr featureCloud, pcl::PointCloud<pcl::Normal>::Ptr featureNormals) {
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ExtractIndices<PT> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normal;
	seg.setOptimizeCoefficients(true);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight(normalWeight);
	seg.setMaxIterations(200);
	seg.setDistanceThreshold(distThresh);
	seg.setInputCloud(remainingCloud);
	if(normals->width>0)
		seg.setInputNormals(normals);
	else {
		for(int i=0;i<remainingCloud->width;i++)
			normals->push_back(pcl::Normal(0, 0, 0));
		seg.setInputNormals(normals);
	}
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() <= 10)
		return false;
	// Prepare extration
	extract.setInputCloud(remainingCloud);
	extract.setIndices(inliers);
	extract_normal.setIndices(inliers);
	extract_normal.setInputCloud(normals);
	//Extract feature cloud and normals
	extract.setNegative(false);
	extract_normal.setNegative(false);
	extract.filter(*featureCloud);
	extract_normal.filter(*featureNormals);
	// Remove points of the current feature
	extract.setNegative(true);
	extract_normal.setNegative(true);
	extract.filter(*remainingCloud);
	extract_normal.filter(*normals);
	return true;
}
void Comparison::detectFeatures(PTC::Ptr comp,bool showFeatures) {
	//cout << "-> Estimating normals...";
	//pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//pcl::NormalEstimation<PT, pcl::Normal> findNormal;
	//pcl::search::KdTree<PT>::Ptr tree(new pcl::search::KdTree<PT>());
	//findNormal.setSearchMethod(tree);
	//findNormal.setInputCloud(comp);
	//findNormal.setKSearch(5); //200
	//findNormal.compute(*normals);
	//Visual Visualizer("Normals", comp);
	//Visualizer.addNormals(comp, normals);
	//Visualizer.processOutput();
	//PTC::Ptr remainingCloud(new PTC);
	//pcl::copyPointCloud(*comp, *remainingCloud);
	//// Create the segmentation object
	//pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	//PTC::Ptr hullCloud(new PTC);
	//PTC::Ptr featureCloud(new PTC);
	//Visual Hulls;
	//if (showFeatures)
	//	Hulls = Visual("Hulls", remainingCloud, featureCloud);
	//cout << "-> Looking for planes...\n";
	//while (findFeature(pcl::SACMODEL_PLANE, 0.001,0.001,true, remainingCloud, normals, coefficients, hullCloud, featureCloud)) {
	//	CADFeatures.push_back(CADFeature(hullCloud, featureCloud, coefficients, pcl::SACMODEL_PLANE));
	//	if (showFeatures) {
	//		Hulls.updateCloud(remainingCloud, featureCloud);
	//		Hulls.processOutput();


	//		// Create a convex hull around the feature cloud
	//		pcl::ConvexHull<PT> convex_hull;
	//		convex_hull.setInputCloud(featureCloud); //->AUSLAGERN
	//		convex_hull.reconstruct(*hullCloud);
	//	}
	//}
	//cout << "-> Looking for cylinders...\n";
	//while (findFeature(pcl::SACMODEL_CYLINDER, 0.002, 0.001,true, remainingCloud, normals, coefficients, hullCloud, featureCloud)) {
	//	CADFeatures.push_back(CADFeature(hullCloud, featureCloud, coefficients, pcl::SACMODEL_CYLINDER)); 		//add the feature to the CAD feature vector
	//	if (showFeatures){
	//		Hulls.updateCloud(remainingCloud, featureCloud);
	//		Hulls.processOutput();


	//		// Create a convex hull around the feature cloud
	//		pcl::ConvexHull<PT> convex_hull;
	//		convex_hull.setInputCloud(featureCloud); //->AUSLAGERN
	//		convex_hull.reconstruct(*hullCloud);
	//	}
	//}
	//Hulls.closeViewer();
}
void findMinRect(PTC::Ptr cloud, std::vector<int>& indices,double& xDim,double& yDim) {
	Eigen::Affine3f rotation;
	PTC::Ptr rotatedCloud(new PTC);
	double minArea = std::numeric_limits<double>::max();
	for (PTC::iterator point = cloud->begin(); point != cloud->end() - 1; point++) { //Minimum bounding rectangle -> Rotating Calippers
		PT next = *(point + 1);
		double angle = atan2(next.y - (*point).y, next.x - (*point).x);
		rotation = Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ());
		pcl::transformPointCloud(*cloud, *rotatedCloud, rotation);
		PT max, min;
		pcl::getMinMax3D(*rotatedCloud, min, max);
		if ((max.x - min.x) * (max.y - min.y) < minArea) {
			minArea = (max.x - min.x) * (max.y - min.y);
			xDim = (max.x - min.x);
			yDim = (max.y - min.y);
			pcl::KdTreeFLANN<PT> findIndice;
			findIndice.setInputCloud(rotatedCloud);
			std::vector<int> tmp;
			std::vector<float> distance;
			findIndice.nearestKSearch(max, 1, tmp, distance);
			indices.push_back(tmp[0]);
			findIndice.nearestKSearch(min, 1, tmp, distance);
			indices.push_back(tmp[0]);
		}
	}
}
void Comparison::findMeasuredFeatures(PTC::Ptr cam, PTC::Ptr comp) { //das ist anders als bei Erdös
	//for (CADFeature& feature : CADFeatures) {
	//	feature.Measuredhull.reset(new PTC);
	//	feature.MeasuredCloud.reset(new PTC);
	//	feature.Measuredcoeffs.reset(new pcl::ModelCoefficients);
	//	pcl::KdTreeFLANN<PT> findInliers;
	//	pcl::PointIndices::Ptr indices(new pcl::PointIndices);
	//	float epsilon = 0.3;
	//	findInliers.setInputCloud(cam);
	//	for (PTC::iterator point = feature.CADcloud->begin(); point != feature.CADcloud->end() - 1; point++) {
	//		std::vector<int> measuredFeaturePoints;
	//		std::vector<float> distance;
	//		findInliers.radiusSearch(*point, epsilon, measuredFeaturePoints, distance);
	//		indices->indices.insert(indices->indices.end(), measuredFeaturePoints.begin(), measuredFeaturePoints.end());
	//	}
	//	//Delete duplicates
	//	std::sort(indices->indices.begin(), indices->indices.end());
	//	indices->indices.erase(std::unique(indices->indices.begin(), indices->indices.end()), indices->indices.end());
	//	//Get proposed feature points
	//	PTC::Ptr measuredFeatureCloud(new PTC);
	//	pcl::ExtractIndices<PT> extract;
	//	extract.setInputCloud(cam);
	//	extract.setIndices(indices);
	//	extract.filter(*measuredFeatureCloud);
	//	//Find normals
	//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//	pcl::NormalEstimation<PT, pcl::Normal> findNormal;
	//	pcl::search::KdTree<PT>::Ptr tree(new pcl::search::KdTree<PT>());
	//	findNormal.setSearchMethod(tree);
	//	findNormal.setInputCloud(cam);
	//	findNormal.setKSearch(5);
	//	findNormal.compute(*normals);
	//	// Create the segmentation object
	//	if (findFeature(feature.type, 0.001, 0.001,true, measuredFeatureCloud, normals, feature.Measuredcoeffs, feature.Measuredhull, feature.MeasuredCloud))
	//		feature.found = true;
	//	else {
	//		std::cout << "Feature not found";
	//		feature.found = false;
	//	}
	//}
}
void Comparison::parametrizeFeatures() { //gib mir nen feature, ich messe alle features und schreib das in ne datei
	std::vector<std::string> rectParameters;
	std::vector<std::string> cylParameters;
	int index = 0;
	for each(CADFeature feature in CADFeatures) {
		if (feature.type == pcl::SACMODEL_PLANE)
		{
			std::string rectParameter = std::to_string(index) + (std::string)",";
			PTC::Ptr projectedCloud(new PTC);
			Eigen::Quaternionf q;
			Eigen::Vector3f coeffAxis(feature.CADcoeffs->values[0], feature.CADcoeffs->values[1], feature.CADcoeffs->values[2]);
			coeffAxis.normalize();
			q.setFromTwoVectors(coeffAxis, Eigen::Vector3f::UnitZ());
			pcl::transformPointCloud(*feature.CADhull, *projectedCloud, (Eigen::Affine3f)q.toRotationMatrix());
			std::vector<int> indices;
			double xDim, yDim;
			findMinRect(projectedCloud, indices,xDim,yDim);
			PT RectMax, RectMin;
			PTC::Ptr rect(new PTC);
			RectMax = feature.CADhull->points[indices[0]];
			RectMin = feature.CADhull->points[indices[1]];
			rect->push_back(RectMin);
			rect->push_back(RectMax);
			rectParameter += "," + std::to_string(xDim);
			rectParameter += "," + std::to_string(yDim);
			PTC::Ptr rect2(new PTC);
			if (feature.found) {
				coeffAxis = Eigen::Vector3f(feature.Measuredcoeffs->values[0], feature.Measuredcoeffs->values[1], feature.Measuredcoeffs->values[2]);
				coeffAxis.normalize();
				q.setFromTwoVectors(coeffAxis, Eigen::Vector3f::UnitZ());
				pcl::transformPointCloud(*feature.Measuredhull, *projectedCloud, (Eigen::Affine3f)q.toRotationMatrix());
				findMinRect(projectedCloud, indices,xDim,yDim);
				rectParameter += "," + std::to_string(xDim);
				rectParameter += "," + std::to_string(yDim);
				RectMax = feature.Measuredhull->points[indices[0]];
				RectMin = feature.Measuredhull->points[indices[1]];
				rect2->push_back(RectMin);
				rect2->push_back(RectMax);
			}
			rectParameters.push_back(rectParameter);
			/*Visualizer = Visual("Comparison", rect, rect2);
			Visualizer.processOutput();*/
		}
		else if (feature.type == pcl::SACMODEL_CYLINDER) {
			std::string cylParameter = std::to_string(index) + (std::string)",";
			for (int i = 0; i < 7; i++) {
				cylParameter += feature.CADcoeffs->values[i];
				cylParameter +=",";
			}
			for (int i = 0; i < 7; i++) {
				cylParameter += feature.Measuredcoeffs->values[i];
				cylParameter += ",";
			}
			cylParameters.push_back(cylParameter);
		}
		index++;
	}
	std::ofstream file("Rectangles.txt", std::ofstream::out | std::ofstream::trunc);
	if (file.is_open()) {
		for (std::string txt : rectParameters)
			file << txt << "\n";
	}
	file.close();
	file= std::ofstream("Cylinders.txt", std::ofstream::out | std::ofstream::trunc);
	if (file.is_open()) {
		for (std::string txt : cylParameters)
			file << txt << "\n";
	}
	file.close();
}
void Comparison::visualizeFeatures(PTC::Ptr cam, PTC::Ptr comp) {
	std::stringstream coeffs;
	Visual showFeature("Feautures", cam,comp);
	for (CADFeature& feature : CADFeatures) {
		coeffs << feature.CADcoeffs;
		showFeature.addText("Feature: \n" + feature.type + (std::string)"\n" + coeffs.str(), 'r');
		if (feature.found) {
			coeffs << feature.Measuredcoeffs;
			showFeature.addText("Feature found: \n" + coeffs.str(), 'g');
			showFeature.updateCloud(feature.MeasuredCloud, feature.CADcloud);
		}
		else {
			showFeature.addText("Feature not found",'g');
			showFeature.updateCloud(cam, feature.CADcloud);
		}
		showFeature.processOutput();
	}
}
void Comparison::checkTolerances(PTC::Ptr cam ,std::vector<Dimension>& tolerances) {
	pcl::KdTreeFLANN<PT> findCamPoint;
	findCamPoint.setInputCloud(cam);
	for (Dimension& tolerance : tolerances) {
		tolerance.measured = 0;
		if (tolerance.type == 1) { //when dealing with linear measurement
			if (tolerance.references.size() != 2) {
				tolerance.measured = -1;
				continue;
			}
			Eigen::Vector3f* vec1 = new Eigen::Vector3f(tolerance.references[0]->points[0].x- tolerance.references[1]->points[0].x, tolerance.references[0]->points[0].y - tolerance.references[1]->points[0].y, tolerance.references[0]->points[0].z - tolerance.references[1]->points[0].z);
			Eigen::Vector3f* vec2=new Eigen::Vector3f(tolerance.references[0]->points[0].x - tolerance.references[0]->back().x, tolerance.references[0]->points[0].y - tolerance.references[0]->back().y, tolerance.references[0]->points[0].z - tolerance.references[0]->back().z);
			Eigen::Vector3f* normal = new Eigen::Vector3f(vec1->cross(*vec2));
			normal->normalize();
			double d = normal->dot(Eigen::Vector3f(tolerance.references[0]->points[0].x, tolerance.references[0]->points[0].y, tolerance.references[0]->points[0].z));
			pcl::ModelCoefficients::Ptr planeCoeffs(new pcl::ModelCoefficients);
			PTC::Ptr plane(new PTC);
			PTC::Ptr cloud(new PTC(*tolerance.references[0]+*tolerance.references[1]));
			double angleThres = 10 / 180.0 * M_PI;
			PT max, min;
			pcl::getMinMax3D(*cloud, min, max);
			pcl::CropBox<PT> crop;
			crop.setInputCloud(cam);
			crop.setNegative(false);
			crop.setMin(Eigen::Vector4f(min.x-0.02, min.y - 0.02, min.z - 0.02, 1));
			crop.setMax(Eigen::Vector4f(max.x + 0.02, max.y + 0.02, max.z + 0.02, 1));
			crop.filter(*cloud);
			Registration Reg(std::to_string(debug));
			pcl::PointCloud<pcl::Normal>::Ptr normals = Reg.getNormals(cloud, 0.01);
			if (findPlane(cloud, plane, planeCoeffs, normals, 0.0001, 0.001, angleThres, normal, 0.002, d)) {
				if (debug > 0) { Visual Visualizer("[DEBUG] Detected plane", cloud, plane); if (debug == 2)Visualizer.processOutput(); }
				tolerance.measuredReferences.push_back(plane);
				tolerance.measured = planeCoeffs->values[2];
			}
		}
		else if (tolerance.type == 3) { //when dealing with radial measurement
			if (tolerance.references.size() != 1) {
				tolerance.measured = -1;
				continue;
			}
			PTC::Ptr cloud(new PTC);
			Eigen::Vector3f* normal=new Eigen::Vector3f(tolerance.coeffs[4], tolerance.coeffs[5], tolerance.coeffs[6]);
			normal->normalize();
			Eigen::Vector3f center(tolerance.coeffs[0], tolerance.coeffs[1], tolerance.coeffs[2]);
			double radius = tolerance.coeffs[3];
			double d = normal->dot(center);
			pcl::ModelCoefficients::Ptr planeCoeffs(new pcl::ModelCoefficients);
			pcl::ModelCoefficients::Ptr circleCoeffs(new pcl::ModelCoefficients);
			PTC::Ptr plane(new PTC);
			PTC::Ptr circle(new PTC);
			double angleThres = 10 / 180.0 * M_PI;
			pcl::CropBox<PT> crop;
			crop.setInputCloud(cam);
			crop.setTranslation(center);
			crop.setNegative(false);
			crop.setRotation((Eigen::Vector3f::UnitZ() * M_PI / 2).cross(*normal));
			crop.setMin(Eigen::Vector4f(-radius +tolerance.lowerDelta*2- 0.005, -radius + tolerance.lowerDelta*2 - 0.005, -0.005,1));
			crop.setMax(Eigen::Vector4f(radius +tolerance.upperDelta*2+ 0.005, radius + tolerance.upperDelta*2 + 0.005, 0.005, 1));
			crop.filter(*cloud);
			Registration Reg(std::to_string(debug));
			pcl::PointCloud<pcl::Normal>::Ptr normals = Reg.getNormals(cloud, 0.01);
			if (findPlane(cloud,plane,planeCoeffs,normals,0.0001,0.0005,angleThres,normal,0.001,d)) {
				if (debug > 0) { Visual Visualizer("[DEBUG] Detected plane", cloud, plane); if (debug == 2)Visualizer.processOutput(); }
				//crop.setMin(Eigen::Vector4f(-radius + tolerance.lowerDelta*2 - 0.005, -radius + tolerance.lowerDelta*2 - 0.005, -0.02, 1));
				//crop.setMax(Eigen::Vector4f(radius + tolerance.upperDelta*2 + 0.005, radius + tolerance.upperDelta*2 + 0.005, 0.02, 1));
				//crop.filter(*cloud);
				//pcl::PointCloud<pcl::Normal>::Ptr normals = Reg.getNormals(cloud, 0.01);
				if (findCircle(plane, planeCoeffs,circle,circleCoeffs, 0.0005,radius,tolerance.lowerDelta,tolerance.upperDelta,0.001,&center)) {
					if (debug > 0) { Visual Visualizer("[DEBUG] Detected circle", plane, circle); if (debug == 2)Visualizer.processOutput(); }
					cout << center << "\n";
					cout << circleCoeffs->values[0] << circleCoeffs->values[1] << circleCoeffs->values[2] << "\n";
					tolerance.measuredReferences.push_back(circle);
					tolerance.measured = circleCoeffs->values[2];
				}
			}
			else {
				tolerance.measured = -1;
			}
		}
	}
}