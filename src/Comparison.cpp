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
#include "Comparison.h"
Comparison::Comparison(std::string debug) { this->debug = stoi(debug); }

bool Comparison::hausdorff(PTC::Ptr cam, PTC::Ptr comp, float& hausdorff, float& avg_dist) {
	std::vector<int> defects;
	return this->hausdorff(cam, comp, std::vector<double>(), pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>), defects, hausdorff, avg_dist, false, true);
}
bool Comparison::hausdorff(PTC::Ptr cam, PTC::Ptr comp, std::vector<double> thresholds, pcl::PointCloud<pcl::PointXYZRGB>::Ptr comp_hausdorff, std::vector<int>& defects, float& hausdorff, float& avg_dist, bool outlierDeletion, bool noCloudNeeded)
{
	bool containsDefects=false;
	pcl::KdTreeFLANN<PT> tree_comp;
	tree_comp.setInputCloud(comp);
	float max_dist_a = -std::numeric_limits<float>::max();
	float threshold_outlier, threshold_defect,threshold_warning;
	// threshold for deviation such as construction deviation or uncertancies 
	// --> we assume that everyting over this value is a bad spot
	if (thresholds.size() == 3) {
		threshold_outlier = pow(thresholds[0],2);
		threshold_defect = pow(thresholds[1],2);
		threshold_warning = pow(thresholds[2],2);
	}
	else if(!noCloudNeeded) {
		PCL_ERROR("For the function hausdorff, three thresholds have to be given!");
		return false;
	}
	std::vector<int> outliers; //in cam 
	std::vector<float> _sqr_distances;
	// compute max distance
	for (size_t i = 0; i < (*cam).points.size(); ++i)
	{
		std::vector<int> indices(1);
		std::vector<float> sqr_distance(1);

		tree_comp.nearestKSearch((*cam).points[i], 1, indices, sqr_distance);
		if (noCloudNeeded) {
			_sqr_distances.push_back(sqr_distance[0]);
			if (sqr_distance[0] > max_dist_a)
				max_dist_a = sqr_distance[0];
		}
		else {
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
	}
	if (outlierDeletion) {
		cout << "-> Deleting outliers...";
		pcl::ExtractIndices<PT> extract;
		extract.setInputCloud(cam);
		pcl::PointIndices::Ptr outlierPtr(new pcl::PointIndices);
		outlierPtr->indices = outliers;
		extract.setIndices(outlierPtr);
		extract.setNegative(true);
		extract.filter(*cam);
	}
	// compute mean distance
	float sum = 0;
	for (std::vector<float>::iterator it = _sqr_distances.begin(); it != _sqr_distances.end(); ++it)
		sum += *it;

	avg_dist = std::sqrt(sum / _sqr_distances.size());
	hausdorff = std::sqrt(max_dist_a);
	return containsDefects;
}
void Comparison::findEdges(PTC::Ptr cloud, double edge_radius, int maxNeighbors,double leafSize)
{
	Registration Reg(std::to_string(debug));
	Reg.downsample<PT>(cloud, cloud, leafSize); //ensure that cloud has a specified point density for each detection
	if (debug > 0) cout << "[DEBUG] Trying to find edges with edge_radius " << edge_radius << " and maxNeighbors " << maxNeighbors << "\n";
	pcl::RadiusOutlierRemoval<PT> ra;
	ra.setInputCloud(cloud);
	ra.setRadiusSearch(edge_radius);
	ra.setMinNeighborsInRadius(maxNeighbors);
	ra.setNegative(true);
	ra.filter(*cloud);
}
bool Comparison::findCircle(PTC::Ptr plane, pcl::ModelCoefficients::Ptr planeCoeffs, PTC::Ptr circle, pcl::ModelCoefficients::Ptr coefficients, double distThresh, int maxIterations, double minInlierRatio, double radius, double radiusLowerDelta, double radiusUpperDelta, Eigen::Vector3f* center, double epsCenter, double edge_radius, int maxNeighbors,double leafSize) {
	PTC::Ptr planeCut(new PTC);
	Eigen::Quaternionf q;
	Eigen::Vector3f coeffAxis(planeCoeffs->values[0], planeCoeffs->values[1], planeCoeffs->values[2]);
	coeffAxis.normalize();
	q.setFromTwoVectors(coeffAxis, Eigen::Vector3f::UnitZ());
	if (edge_radius > 0) {
		findEdges(plane, edge_radius, maxNeighbors,leafSize);
	}
	pcl::transformPointCloud(*plane, *plane, (Eigen::Affine3f)q.toRotationMatrix());
	pcl::transformPoint(*center,*center, (Eigen::Affine3f)q.toRotationMatrix());
	pcl::CropBox<PT> crop;
	crop.setInputCloud(plane);
	crop.setNegative(false);
	int numberOfTries;
	int refMultiplier;
	int maxNumberOfTries = epsCenter / leafSize;
	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
	for (refMultiplier = 1; refMultiplier <= maxNumberOfTries; refMultiplier++) {
		double eps = (double)refMultiplier / maxNumberOfTries * epsCenter;
		int minInliers = minInlierRatio * 2 * M_PI * (radius-eps) / leafSize;
		crop.setMin(Eigen::Vector4f((*center)[0] - radius- eps, (*center)[1] - radius - eps, (*center)[2]-eps, 1));
		crop.setMax(Eigen::Vector4f((*center)[0] + radius + eps, (*center)[1] + radius + eps, (*center)[2] + eps, 1));
		crop.filter(*planeCut);
		for (numberOfTries = 0; numberOfTries < maxNumberOfTries; numberOfTries++) {
			pcl::SACSegmentationFromNormals<PT, pcl::Normal> seg;
			seg.setModelType(pcl::SACMODEL_CIRCLE2D);
			if (numberOfTries <= maxNumberOfTries - 1 && radius != 0)
				seg.setRadiusLimits((10.0 * numberOfTries) / maxNumberOfTries * radiusLowerDelta + radius, (10.0 * numberOfTries) / maxNumberOfTries * radiusUpperDelta + radius);
			if (findFeature(seg, planeCut, circle, coefficients, pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>), 0, distThresh, maxIterations, minInliers, true, circle)) {
				int minInliersReal= minInlierRatio * 2 * M_PI * coefficients->values[2] / leafSize;
				if (circle->width < minInliersReal)
					continue;
				if (sqrt(pow(coefficients->values[0] - (*center)[0],2)+pow(coefficients->values[1] - (*center)[1],2)) <= eps)
					break;
			}
		}
		if (numberOfTries != maxNumberOfTries)
			break;
		if (debug == 2) {
			cout << "[DEBUG] No feature within the radius threshold or center threshold have been found.\n";
			Visual Visualizer("[DEBUG] Circle found instead", planeCut, circle);
			Visualizer.processOutput();
			cout << "[DEBUG] Relaxing thresholds...\n";
			Visualizer.closeViewer();
		}
	}
	if (numberOfTries == maxNumberOfTries) {
			pcl::console::setVerbosityLevel(pcl::console::L_WARN);
			PCL_WARN("No valid circle found. Measured set to -1. Check the output.");
			return false;
	}
	pcl::console::setVerbosityLevel(pcl::console::L_WARN);
	int numOfPoints = 1000;
	double t = 0;
	for (int i = 0; i < numOfPoints; i++) {
		t += 2 * M_PI / numOfPoints;
		circle->push_back(PT(coefficients->values[0] + coefficients->values[2] * cos(t), coefficients->values[1] + coefficients->values[2] * sin(t), circle->points[0].z));
	}
	circle->push_back(PT(coefficients->values[0], coefficients->values[1], circle->points[0].z));
	pcl::transformPointCloud(*circle, *circle, (Eigen::Affine3f)q.toRotationMatrix().inverse());
	pcl::transformPointCloud(*plane, *plane, (Eigen::Affine3f)q.toRotationMatrix().inverse());
	return true;
}
bool Comparison::findLine(PTC::Ptr plane, PTC::Ptr line, pcl::ModelCoefficients::Ptr coefficients, double distThresh, int maxIterations, double minInlierRatio, Eigen::Vector3f* axis, double epsAxis, PTC::Ptr ref, double epsRef,double edge_radius,int maxNeighbors, double leafSize) {
	PTC::Ptr planeCut(new PTC);
	if (edge_radius > 0) {
		findEdges(plane, edge_radius, maxNeighbors,leafSize);
	}
	pcl::CropBox<PT> crop;
	crop.setInputCloud(plane);
	crop.setNegative(false);
	PT max, min;
	pcl::getMinMax3D(*ref, min, max);
	int minInliers = minInlierRatio*pcl::euclideanDistance(max, min)/leafSize;
	int numberOfTries;
	int refMultiplier;
	int maxNumberOfTries = epsRef/leafSize;
	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
	for (refMultiplier = 1; refMultiplier <= maxNumberOfTries; refMultiplier++) {
		double eps = (double)refMultiplier / maxNumberOfTries * epsRef;
		crop.setMin(Eigen::Vector4f(min.x - eps, min.y - eps, min.z - eps, 1));
		crop.setMax(Eigen::Vector4f(max.x + eps, max.y + eps, max.z + eps, 1));
		crop.filter(*planeCut);
		for (numberOfTries = 0; numberOfTries < maxNumberOfTries; numberOfTries++) {
			pcl::SACSegmentationFromNormals<PT, pcl::Normal> seg;
			seg.setModelType(pcl::SACMODEL_PARALLEL_LINE);
			if (epsAxis != 0) {
				seg.setAxis(*axis);
				seg.setEpsAngle(epsAxis);
			}
			if (findFeature(seg, planeCut, line, coefficients, pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>), 0, distThresh, maxIterations, minInliers, true, line)) {
				float hausdorff, avg_dist;
				this->hausdorff(line, ref, hausdorff, avg_dist);
				if (avg_dist <=eps  || epsRef == 0)
					break;
			}
		}
		if (numberOfTries != maxNumberOfTries)
			break;
		if (debug == 2) {
			cout << "[DEBUG] No line within the reference threshold has been found. Relaxing threshold...\n";
			Visual Visualizer("[DEBUG] Line found instead", planeCut, ref, line);
			Visualizer.processOutput(); Visualizer.closeViewer();
		}
	}
	if (numberOfTries == maxNumberOfTries) {
		pcl::console::setVerbosityLevel(pcl::console::L_WARN);
		PCL_WARN("No valid line found. Measured set to -1. Check the output.");
		return false;
	}
	pcl::console::setVerbosityLevel(pcl::console::L_WARN);
	return true;
}
bool Comparison::findPlane(PTC::Ptr cloud, PTC::Ptr plane, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::Normal>::Ptr normals, double normal_weight, double distThresh, int maxIterations, double minInlierRatio, Eigen::Vector3f* normalAxis, double epsAngle, double distanceFromOrigin, double epsDist,PTC::Ptr projectedPlane) {
	int numberOfTries;
	int maxNumberOfTries = 10;
	pcl::SACSegmentationFromNormals<PT, pcl::Normal> seg;
	seg.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
	seg.setProbability(minInlierRatio);
	if(epsDist!=0)
		seg.setDistanceFromOrigin(distanceFromOrigin);
	if (epsAngle != 0) {
		seg.setAxis(*normalAxis);
		seg.setEpsAngle(epsAngle);
	}
	for (numberOfTries = 0; numberOfTries < maxNumberOfTries; numberOfTries++) {
		if (!findFeature(seg, cloud, plane, coefficients,normals, normal_weight, distThresh, maxIterations,10, true, projectedPlane)) {
			numberOfTries = maxNumberOfTries;
			break;
		}
		if (epsDist == 0)
			break;
		else {
			if (abs(abs(coefficients->values[3]) - abs(distanceFromOrigin)) <= epsDist)
				break;
		}
		if (debug == 2) {
			cout << "[DEBUG] No feature within the epsDist threshold has been found.\n";
			Visual Visualizer("[DEBUG] Plane found instead", cloud, plane);
			Visualizer.processOutput();
			cout << "[DEBUG] Retrying...";
			Visualizer.closeViewer();
		}
	}
	if (numberOfTries == maxNumberOfTries) {
		PCL_WARN("No valid plane found. Measured set to -1. Check the output.");
		return false;
	}
	return true;
}
bool Comparison::findCyl(PTC::Ptr cloud, PTC::Ptr cyl, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::Normal>::Ptr normals, double normal_weight, double distThresh, int maxIterations, double minInlierRatio, Eigen::Vector3f* axis, double epsAxis, double radius, double epsRadius, Eigen::Vector3f* center, double epsCenter, PTC::Ptr projectedCyl) {
	int numberOfTries;
	int maxNumberOfTries = 10;
	pcl::SACSegmentationFromNormals<PT, pcl::Normal> seg;
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	seg.setProbability(minInlierRatio);
	if (epsAxis != 0) {
		seg.setAxis(*axis);
		seg.setEpsAngle(epsAxis);
	}
	if (epsRadius != 0) {
		seg.setRadiusLimits(radius-epsRadius,radius+epsRadius);
	}
	for (numberOfTries = 0; numberOfTries < maxNumberOfTries; numberOfTries++) {
		if (!findFeature(seg, cloud, cyl, coefficients,normals, normal_weight, distThresh, maxIterations,10, true,projectedCyl)) {
			numberOfTries = maxNumberOfTries;
			break;
		}
		if (epsCenter == 0)
			break;
		else {
			Eigen::Vector3f crossVector = axis->cross(Eigen::Vector3f(coefficients->values[3], coefficients->values[4], coefficients->values[5]));
			double distanceBetweenAxes = crossVector.dot(*center - Eigen::Vector3f(coefficients->values[0], coefficients->values[1], coefficients->values[2])) / crossVector.norm();
			if (abs(distanceBetweenAxes) <= epsCenter)
				break;
		}
		if (debug == 2) {
			cout << "[DEBUG] No cylinder within the epsDist threshold has been found.\n";
			Visual Visualizer("[DEBUG] Cylinder found instead", cloud, cyl);
			Visualizer.processOutput();
			cout << "[DEBUG] Retrying...";
			Visualizer.closeViewer();
		}
	}
	if (numberOfTries == maxNumberOfTries) {
		PCL_WARN("No valid Cylinder found. Measured set to -1. Check the output.");
		return false;
	}
	return true;
}
bool Comparison::findFeature(pcl::SACSegmentationFromNormals<PT, pcl::Normal> seg, PTC::Ptr remainingCloud, PTC::Ptr featureCloud, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::Normal>::Ptr normals, double normalWeight, double distThresh,int maxIterations, int minInliers, bool projectInliers, PTC::Ptr projectedCloud, pcl::PointCloud<pcl::Normal>::Ptr featureNormals) {
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ExtractIndices<PT> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normal;
	seg.setOptimizeCoefficients(true);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight(normalWeight);
	seg.setMaxIterations(maxIterations);
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
	if (inliers->indices.size() <= minInliers)
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
	//Remove points of the current feature
	extract.setNegative(true);
	extract_normal.setNegative(true);
	extract.filter(*remainingCloud);
	extract_normal.filter(*normals);
	if (projectInliers) {
		if (seg.getModelType() != pcl::SACMODEL_CYLINDER) {
			pcl::ProjectInliers<PT> proj;
			proj.setModelType(seg.getModelType());
			proj.setInputCloud(featureCloud);
			proj.setModelCoefficients(coefficients);
			proj.filter(*projectedCloud);
		}
		else { //ProjectInlier Filter for cylinders has a bug in PCL 1.9.1 -> thus code was manually implemented -> TODO Delete this code when bug is fixed
			projectedCloud->points.resize(featureCloud->points.size());
			projectedCloud->width = featureCloud->width;
			Eigen::Vector4f line_pt(coefficients->values[0], coefficients->values[1], coefficients->values[2], 0);
			Eigen::Vector4f line_dir(coefficients->values[3], coefficients->values[4], coefficients->values[5], 0);
			float ptdotdir = line_pt.dot(line_dir);
			float dirdotdir = 1.0f / line_dir.dot(line_dir);
			typedef typename pcl::traits::fieldList<PT>::type FieldList;
			// Iterate over each point
			for (size_t i = 0; i < projectedCloud->points.size(); ++i)
				// Iterate over each dimension
				pcl::for_each_type <FieldList>(pcl::NdConcatenateFunctor <PT, PT>(featureCloud->points[i], projectedCloud->points[i]));
			// Iterate through the 3d points and calculate the distances from them to the cylinder
			for (size_t i = 0; i < projectedCloud->points.size(); ++i)
			{
				Eigen::Vector4f p(featureCloud->points[i].x,
					featureCloud->points[i].y,
					featureCloud->points[i].z,
					0);
				float k = (p.dot(line_dir) - ptdotdir) * dirdotdir;
				pcl::Vector4fMap pp = projectedCloud->points[i].getVector4fMap();
				pp.matrix() = line_pt + k * line_dir;
				Eigen::Vector4f dir = p - pp;
				dir.normalize();
				// Calculate the projection of the point onto the cylinder
				pp += dir * coefficients->values[6];
			}
		}
	}
	return true;
}
void Comparison::checkTolerances(PTC::Ptr cam ,std::vector<Dimension>& tolerances, double edge_radius, int maxNeighbors,double leafSize, double minInlierRatio, double maxTolerance,int numberOfIterations_Tol, double tolerance_search_radius, double plane_normal_weight, double cyl_normal_weight, double plane_dist_thresh, double line_dist_thresh, double circle_dist_thresh, double plane_epsDist, double circle_epsCenter, double cyl_epsCenter, double angleThres, double angular_angleThres) {
	for (Dimension& tolerance : tolerances) {
		tolerance.measured = -1;
		tolerance.ok = false;
		pcl::CropBox<PT> crop;
		crop.setInputCloud(cam);
		crop.setNegative(false);
		PTC::Ptr plane(new PTC);
		if (tolerance.type == 1) { //when dealing with linear measurement
			if (tolerance.references.size() != 2) {
				PCL_WARN("Not the right amount of references. Measured set to -1. Check the output.");
				continue;
			}
			Eigen::Vector3f* vec1 = new Eigen::Vector3f(tolerance.references[0]->points[0].x- tolerance.references[1]->points[0].x, tolerance.references[0]->points[0].y - tolerance.references[1]->points[0].y, tolerance.references[0]->points[0].z - tolerance.references[1]->points[0].z);
			Eigen::Vector3f* vec2=new Eigen::Vector3f(tolerance.references[0]->points[0].x - tolerance.references[0]->back().x, tolerance.references[0]->points[0].y - tolerance.references[0]->back().y, tolerance.references[0]->points[0].z - tolerance.references[0]->back().z);
			Eigen::Vector3f* normal = new Eigen::Vector3f(vec1->cross(*vec2));
			normal->normalize(); if (debug > 0) cout << "[DEBUG] Normal: " << *normal << "\n";
			double d = normal->dot(Eigen::Vector3f(tolerance.references[0]->points[0].x, tolerance.references[0]->points[0].y, tolerance.references[0]->points[0].z));
			pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
			PTC::Ptr line1(new PTC);
			PTC::Ptr line2(new PTC);
			PTC::Ptr cloud(new PTC(*tolerance.references[0]+*tolerance.references[1]));
			PT max, min;
			pcl::getMinMax3D(*cloud, min, max); if (debug > 0) cout << "[DEBUG] Min: " << min << ", Max: "<< max<<"\n";
			crop.setMin(Eigen::Vector4f(min.x- maxTolerance, min.y - maxTolerance, min.z - maxTolerance, 1));
			crop.setMax(Eigen::Vector4f(max.x + maxTolerance, max.y + maxTolerance, max.z + maxTolerance, 1));
			crop.filter(*cloud);
			Registration Reg(std::to_string(debug));
			pcl::PointCloud<pcl::Normal>::Ptr normals = Reg.getNormals(cloud, tolerance_search_radius);
			if (findPlane(cloud, plane, coeffs, normals, plane_normal_weight, plane_dist_thresh, numberOfIterations_Tol, minInlierRatio, normal, angleThres , d, plane_epsDist, plane)) {
				if (debug > 0) { Visual Visualizer("[DEBUG] Detected plane", cloud, plane); if (debug == 2)Visualizer.processOutput(); }
				PTC::Ptr refLine1(new PTC); PTC::Ptr refLine2(new PTC);
				pcl::ProjectInliers<PT> projectLines;
				projectLines.setModelType(pcl::SACMODEL_PLANE);
				projectLines.setModelCoefficients(coeffs);
				projectLines.setInputCloud(tolerance.references[0]);
				projectLines.filter(*refLine1);
				projectLines.setInputCloud(tolerance.references[1]);
				projectLines.filter(*refLine2);
				coeffs.reset(new pcl::ModelCoefficients);
				if (findLine(plane, line1, coeffs, line_dist_thresh, numberOfIterations_Tol, minInlierRatio, vec2, angleThres, refLine1,maxTolerance, edge_radius, maxNeighbors, leafSize)) {
					if (debug > 0) { Visual Visualizer("[DEBUG] Detected line", plane, refLine1,line1); if (debug == 2)Visualizer.processOutput(); }
					tolerance.measuredReferences.push_back(line1);
					coeffs.reset(new pcl::ModelCoefficients);
					vec2= new Eigen::Vector3f(tolerance.references[1]->points[0].x - tolerance.references[1]->back().x, tolerance.references[1]->points[0].y - tolerance.references[1]->back().y, tolerance.references[1]->points[0].z - tolerance.references[1]->back().z);
					if (findLine(plane, line2, coeffs, line_dist_thresh, numberOfIterations_Tol, minInlierRatio, vec2, angleThres, refLine2, maxTolerance, edge_radius, maxNeighbors, leafSize)) {
						if (debug > 0) { Visual Visualizer("[DEBUG] Detected line", plane, refLine2,line2); if (debug == 2)Visualizer.processOutput(); }
						tolerance.measuredReferences.push_back(line2);
						float hausdorff, avg_dist;
						this->hausdorff(tolerance.measuredReferences[0], tolerance.measuredReferences[1], hausdorff, avg_dist);
						tolerance.measured = hausdorff;
						tolerance.ok = tolerance.measured <= (tolerance.value + tolerance.upperDelta) && tolerance.measured >= (tolerance.value + tolerance.lowerDelta);
					}
				}
			}
		}
		else if (tolerance.type == 2) { //when dealing with angular measurement
			if (tolerance.references.size() != 2) {
				PCL_WARN("Not the right amount of references. Measured set to -1. Check the output.");
				continue;
			}
			Eigen::Vector3f* vec1 = new Eigen::Vector3f(tolerance.references[0]->points[0].x - tolerance.references[0]->back().x, tolerance.references[0]->points[0].y - tolerance.references[0]->back().y, tolerance.references[0]->points[0].z - tolerance.references[0]->back().z);
			Eigen::Vector3f* vec2 = new Eigen::Vector3f(tolerance.references[1]->points[0].x - tolerance.references[1]->back().x, tolerance.references[1]->points[0].y - tolerance.references[1]->back().y, tolerance.references[1]->points[0].z - tolerance.references[1]->back().z);
			Eigen::Vector3f* normal = new Eigen::Vector3f(vec1->cross(*vec2));
			normal->normalize(); if (debug > 0) cout << "[DEBUG] Normal: " << *normal << "\n";
			double d = normal->dot(Eigen::Vector3f(tolerance.references[0]->points[0].x, tolerance.references[0]->points[0].y, tolerance.references[0]->points[0].z));
			pcl::ModelCoefficients::Ptr coeffs1(new pcl::ModelCoefficients);
			pcl::ModelCoefficients::Ptr coeffs2(new pcl::ModelCoefficients);
			PTC::Ptr plane(new PTC);
			PTC::Ptr line1(new PTC);
			PTC::Ptr line2(new PTC);
			PTC::Ptr cloud(new PTC(*tolerance.references[0] + *tolerance.references[1]));
			PT max, min;
			pcl::getMinMax3D(*cloud, min, max);
			crop.setMin(Eigen::Vector4f(min.x - maxTolerance, min.y - maxTolerance, min.z - maxTolerance, 1));
			crop.setMax(Eigen::Vector4f(max.x + maxTolerance, max.y + maxTolerance, max.z + maxTolerance, 1));
			crop.filter(*cloud);
			Registration Reg(std::to_string(debug));
			pcl::PointCloud<pcl::Normal>::Ptr normals = Reg.getNormals(cloud, tolerance_search_radius);
			if (findPlane(cloud, plane, coeffs1, normals, plane_normal_weight, plane_dist_thresh, numberOfIterations_Tol,minInlierRatio, normal, angleThres, d, plane_epsDist, plane)) {
				if (debug > 0) { Visual Visualizer("[DEBUG] Detected plane", cloud, plane); if (debug == 2)Visualizer.processOutput(); }
				PTC::Ptr refLine1(new PTC); PTC::Ptr refLine2(new PTC);
				pcl::ProjectInliers<PT> projectLines;
				projectLines.setModelType(pcl::SACMODEL_PLANE);
				projectLines.setModelCoefficients(coeffs1);
				projectLines.setInputCloud(tolerance.references[0]);
				projectLines.filter(*refLine1);
				projectLines.setInputCloud(tolerance.references[1]);
				projectLines.filter(*refLine2);
				coeffs1.reset(new pcl::ModelCoefficients);
				if (findLine(plane, line1, coeffs1, line_dist_thresh, numberOfIterations_Tol, minInlierRatio, vec1, angular_angleThres, refLine1, maxTolerance, edge_radius, maxNeighbors,leafSize)) {
					if (debug > 0) { Visual Visualizer("[DEBUG] Detected line", plane, refLine1,line1); if (debug == 2)Visualizer.processOutput(); }
					tolerance.measuredReferences.push_back(line1);
					if (findLine(plane, line2, coeffs2, line_dist_thresh, numberOfIterations_Tol, minInlierRatio, vec2, angular_angleThres, refLine2, maxTolerance, edge_radius, maxNeighbors, leafSize)) {
						if (debug > 0) { Visual Visualizer("[DEBUG] Detected line", plane, refLine2,line2); if (debug == 2)Visualizer.processOutput(); }
						tolerance.measuredReferences.push_back(line2);
						Eigen::Vector3f a(coeffs1->values[3], coeffs1->values[4], coeffs1->values[5]);
						Eigen::Vector3f b(coeffs2->values[3], coeffs2->values[4], coeffs2->values[5]);
						tolerance.measured = acos(a.dot(b)/(a.norm()*b.norm()));
						tolerance.ok = tolerance.measured <= (tolerance.value + tolerance.upperDelta) && tolerance.measured >= (tolerance.value + tolerance.lowerDelta);
					}
				}
			}
		}
		else if (tolerance.type == 3) { //when dealing with radial measurement
			if (tolerance.references.size() != 1) {
				PCL_WARN("Not the right amount of references. Measured set to -1. Check the output.");
				continue;
			}
			PTC::Ptr cloud(new PTC);
			Eigen::Vector3f* normal=new Eigen::Vector3f(tolerance.coeffs[4], tolerance.coeffs[5], tolerance.coeffs[6]);
			normal->normalize(); if (debug > 0) cout << "[DEBUG] Normal: " << *normal << "\n";
			Eigen::Vector3f center(tolerance.coeffs[0], tolerance.coeffs[1], tolerance.coeffs[2]);
			double radius = tolerance.coeffs[3];
			//if the diameter has been tolerated in the CAD file instead of the radius, the tolerances have to be halved to tolerate the radius
			if (tolerance.value != radius) {
				tolerance.value = radius;
				tolerance.upperDelta /= 2;
				tolerance.lowerDelta /= 2;
			}
			double d = normal->dot(center);
			pcl::ModelCoefficients::Ptr planeCoeffs(new pcl::ModelCoefficients);
			pcl::ModelCoefficients::Ptr circleCoeffs(new pcl::ModelCoefficients);
			PTC::Ptr plane(new PTC);
			PTC::Ptr circle(new PTC);
			crop.setTranslation(center);
			crop.setRotation((Eigen::Vector3f::UnitZ() * M_PI / 2).cross(*normal));
			crop.setMin(Eigen::Vector4f(-radius +tolerance.lowerDelta- maxTolerance, -radius + tolerance.lowerDelta - maxTolerance, -0.005,1));
			crop.setMax(Eigen::Vector4f(radius +tolerance.upperDelta+ maxTolerance, radius + tolerance.upperDelta + maxTolerance, 0.005, 1));
			crop.filter(*cloud);
			Registration Reg(std::to_string(debug));
			pcl::PointCloud<pcl::Normal>::Ptr normals = Reg.getNormals(cloud, tolerance_search_radius);
			if (findPlane(cloud,plane,planeCoeffs,normals, plane_normal_weight, plane_dist_thresh, numberOfIterations_Tol, minInlierRatio,normal, angleThres,d, plane_epsDist, plane)) {
				if (debug > 0) { Visual Visualizer("[DEBUG] Detected plane", cloud, plane); if (debug == 2)Visualizer.processOutput(); }
				if (findCircle(plane, planeCoeffs,circle,circleCoeffs, circle_dist_thresh, numberOfIterations_Tol, minInlierRatio, radius,tolerance.lowerDelta,tolerance.upperDelta,&center, circle_epsCenter, edge_radius, maxNeighbors,leafSize)) {
					if (debug > 0) { Visual Visualizer("[DEBUG] Detected circle", plane, tolerance.references[0],circle); if (debug == 2)Visualizer.processOutput(); }
					tolerance.measuredReferences.push_back(circle);
					tolerance.measured = circleCoeffs->values[2];
					tolerance.ok = tolerance.measured <= (tolerance.value+tolerance.upperDelta)&& tolerance.measured >= (tolerance.value + tolerance.lowerDelta);
				}
			}
		}
		else if (tolerance.type == 4) { //when dealing with curve length
			//Not implemented
		}
		else if (tolerance.type == 5|| tolerance.type == 6) { //when dealing with a measure of flatness or circularity
			PTC::Ptr cloud(new PTC);
			Eigen::Vector3f* normal = new Eigen::Vector3f(tolerance.coeffs[3], tolerance.coeffs[4], tolerance.coeffs[5]);
			normal->normalize(); if (debug > 0) cout << "[DEBUG] Normal: " << *normal << "\n";
			Eigen::Vector3f* ref;
			double radius=0, d = 0;
			if (tolerance.type == 6) {
				radius = tolerance.coeffs[6];
				ref = new Eigen::Vector3f((tolerance.coeffs[7] + tolerance.coeffs[10]) / 2.0, (tolerance.coeffs[8] + tolerance.coeffs[11]) / 2.0, (tolerance.coeffs[9] + tolerance.coeffs[12]) / 2.0);
				crop.setTranslation(*ref);
				crop.setRotation((Eigen::Vector3f::UnitZ()* M_PI / 2).cross(*normal));
				double dist = Eigen::Vector3f(tolerance.coeffs[7] - tolerance.coeffs[10], tolerance.coeffs[8] - tolerance.coeffs[11], tolerance.coeffs[9] - tolerance.coeffs[12]).norm();
				crop.setMin(Eigen::Vector4f(-radius - maxTolerance / 2, -radius - maxTolerance / 2, -dist / 2+0.001, 1));
				crop.setMax(Eigen::Vector4f(radius + maxTolerance / 2, radius + maxTolerance / 2, +dist / 2-0.001, 1));
			}
			else {
				ref = new Eigen::Vector3f(tolerance.coeffs[0], tolerance.coeffs[1], tolerance.coeffs[2]);
				d = normal->dot(*ref);
				PTC::Ptr refCloud(new PTC);
				for(PTC::Ptr points:tolerance.references)
					refCloud.reset(new PTC(*refCloud + *points));
				PT max, min;
				pcl::getMinMax3D(*refCloud, min, max);
				crop.setMin(Eigen::Vector4f(min.x+0.002-abs((*normal)[0]) * maxTolerance/2, min.y + 0.002 - abs((*normal)[1]) * maxTolerance/2, min.z + 0.002 - abs((*normal)[2]) * maxTolerance/2, 1));
				crop.setMax(Eigen::Vector4f(max.x-0.002 + abs((*normal)[0]) * maxTolerance/2, max.y - 0.002 + abs((*normal)[1]) * maxTolerance/2, max.z - 0.002 + abs((*normal)[2]) * maxTolerance/2, 1));
			}
			crop.filter(*cloud);
			pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
			PTC::Ptr feature(new PTC);
			PTC::Ptr featureProjected(new PTC);
			Registration Reg(std::to_string(debug));
			pcl::PointCloud<pcl::Normal>::Ptr normals = Reg.getNormals(cloud, tolerance_search_radius);
			if (tolerance.type == 6) {
				if (findCyl(cloud, feature, coeffs, normals, cyl_normal_weight, maxTolerance / 2, numberOfIterations_Tol, minInlierRatio,normal , angleThres, radius, maxTolerance / 2, ref, cyl_epsCenter, featureProjected)) {
					if (debug > 0) { Visual Visualizer("[DEBUG] Detected cylinder", cloud, feature, featureProjected); if (debug == 2)Visualizer.processOutput(); }
					tolerance.measuredReferences.push_back(feature);
					float hausdorff, avg_dist;
					this->hausdorff(featureProjected, feature, hausdorff, avg_dist);
					tolerance.measured = hausdorff;
					tolerance.ok = tolerance.measured <= tolerance.value;
				}
			}
			else
				if (findPlane(cloud, feature, coeffs, normals, plane_normal_weight, maxTolerance/2, numberOfIterations_Tol, minInlierRatio,normal, angleThres, d, plane_epsDist, featureProjected)) {
					if (debug > 0) { Visual Visualizer("[DEBUG] Detected plane", cloud, feature, featureProjected); if (debug == 2)Visualizer.processOutput(); }
					tolerance.measuredReferences.push_back(feature);
					float hausdorff, avg_dist;
					this->hausdorff(featureProjected, feature, hausdorff, avg_dist);
					tolerance.measured = hausdorff;
					tolerance.ok = tolerance.measured <= tolerance.value;
				}
		}
		if (debug > 0) {
			std::vector<Dimension> tol = { tolerance };
			visualizeTolerances(cam, tol, "[DEBUG] Last tolerance");
		}
	}
}
void Comparison::visualizeTolerances(PTC::Ptr cam, std::vector<Dimension> tolerances,std::string name) {
	PTC::Ptr tolerance_points(new PTC);
	PTC::Ptr reference_points(new PTC);
	Visual Tol(name, cam, reference_points, tolerance_points);
	for (Dimension tolerance : tolerances) {
		for (PTC::Ptr points : tolerance.references)
			reference_points.reset(new PTC(*reference_points + *points));
		for (PTC::Ptr points : tolerance.measuredReferences)
			tolerance_points.reset(new PTC(*tolerance_points + *points));
		if (tolerance.ok)
			Tol.addText("Tolerance "+ std::to_string(tolerance.id)+": "+std::to_string(tolerance.value) + " measured:" + std::to_string(tolerance.measured), tolerance.textposition, 0, 255, 0);
		else
			Tol.addText("Tolerance " + std::to_string(tolerance.id) + ": " + std::to_string(tolerance.value) + " measured:" + std::to_string(tolerance.measured), tolerance.textposition, 255, 0, 0);
	}
	Tol.updateCloud(cam, reference_points, tolerance_points);
	Tol.processOutput();
}