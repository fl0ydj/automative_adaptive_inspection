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
#ifndef COMPARISON_H_
#define COMPARISON_H_
#include "Visual.h"
#include "Dimension.hpp"
#include "Registration.h"
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/pca.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/concatenate.h>
typedef pcl::PointXYZ PT;
typedef pcl::PointCloud<PT> PTC;
/**@brief Class for the subproblem of the comparison.

This class allows to perform all kinds of feature detections: edges, circles, lines, planes and cylinders. Furthermore, it also offers a more generic feature detection function.
Moreover, the function for the whole tolerance check is implemented as well.
@author Justin Heinz, Oliver Krumpek et Al.
*/
class Comparison
{
private:
    /**Debug level (0 - none,1 - some,2 - all)*/
    int debug;
public:
    /**Standard constructor
    @param[in] debug -  how many debug info should be displayed (0 - none,1 - some,2 - all)
    */
    Comparison(std::string debug);
    /**Hausdorff comparison without returning a colored hausdorff cloud.  Keep in mind that outlierDeletion is set to false automatically.
    @param[in] cam - measured cloud
    @param[in] comp - cloud of the component
    @param[out] hausdorff - hausdorff distance
    @param[out] avg_dist - average distance of all points.
    @return
    * true if comp contains defects, false otherwise
    * true if comp contains defects, false otherwise
    */
    bool hausdorff(PTC::Ptr cam, PTC::Ptr comp, float& hausdorff, float& avg_dist);
    /**Hausdorff comparison and returning a colored hausdorff cloud.
    Source: Krumpek, O. et al.: RobotScan. Projektbericht, Berlin, 2018.
    @param[in] cam - measured cloud
    @param[in] comp - cloud of the component
    @param[in] thresholds - threshold vector to classify points (threshold_outlier,threshold_defect,threshold_warning)
    @param[out] comp_hausdorff - colored pointcloud to visualize defects
    @param[out] hausdorff - hausdorff distance
    @param[out] avg_dist - average distance of all points
    @param[in] outlierDeletion - If true, delets all points with a distance greater than threshold_outlier to the nearest neighbor in the other cloud. Those points are also not considered for hausdorff und avg_dist calculation.
    @param[in] noCloudNeeded - If true, the part for creating the comp_hausdorff cloud is skipped
    @return
    * true if comp contains defects, false otherwise
    */
    bool hausdorff(PTC::Ptr cam,PTC::Ptr comp, std::vector<double> thresholds, pcl::PointCloud<pcl::PointXYZRGB>::Ptr comp_hausdorff, std::vector<int>& defects, float& hausdorff,float& avg_dist, bool outlierDeletion = false, bool noCloudNeeded=false);
    /**Detection of edges using a radius outlier removal filter. Beforehand the cloud is downsampled to guarantee a predictable point density.
    @param[in,out] cloud - cloud to detect edges in
    @param[in] edge_radius - search radius used for the outlier removal filter
    @param[in] maxNeighbors - The maximum number of neighbors a point is allowed to have within the search radius to be considered as an edge
    @param[out] leafSize - leafSize for the downsampling*/
    void findEdges(PTC::Ptr cloud, double edge_radius, int maxNeighbors,double leafSize);
    /**Detection of circles in a plane.
    @param[in,out] plane - plane used for the circle detection. The edges in the plane are returned.
    @param[in] planeCoeffs - the coefficients of the plane
    @param[out] circle - a pointcloud representing the circle found
    @param[out] coefficients - the coefficients of the circle found
    @param[in]  distThresh - distance Threshold for the feature detection
    @param[in] maxIterations - the number of iterations for the feature detection
    @param[in] minInlierRatio - the minimum number of inliers in relation to the maximum number of possible inliers a feature must have to be considered as valid
    @param[in] radius - the expected radius of the circle
    @param[in] radiusLowerDelta - the maximal allowed negative deviation
    @param[in] radiusUperDelta - the maximal allowed positive deviation
    @param[in] center - expected center point of the circle
    @param[in] epsCenter - allowed deviation with respect to the center
    @param[in] edge_radius - search radius for edge detection
    @param[in] maxNeighbors - The maximum number of neighbors a point is allowed to have within the search radius to be considered as an edge
    @param[in] leafSize - leafSize for the downsampling
    @return
    *true if a circle within the thresholds was found, false otherwise
    */
    bool findCircle(PTC::Ptr plane, pcl::ModelCoefficients::Ptr planeCoeffs, PTC::Ptr circle, pcl::ModelCoefficients::Ptr coefficients, double distThresh, int maxIterations, double minInlierRatio, double radius=0, double radiusLowerDelta=0, double radiusUpperDelta=0, Eigen::Vector3f* center = new Eigen::Vector3f(), double epsCenter = 0, double edge_radius=0, int maxNeighbors=0, double leafSize=0);
    /**Detection of lines.
    @param[in,out] plane - plane used for the line detection. The edges in the plane are returned.
    @param[out] line - a pointcloud representing the line found
    @param[out] coefficients - the coefficients of the line found
    @param[in] distThresh - distance Threshold for the feature detection
    @param[in] maxIterations - the number of iterations for the feature detection
    @param[in] minInlierRatio - the minimum number of inliers in relation to the maximum number of possible inliers a feature must have to be considered as valid
    @param[in] axis - the axis on which the line search is performed
    @param[in] epsAxis - the allowed angular deviation to the given axis in rad
    @param[in] ref - the cloud of the reference line
    @param[in] epsRef - the allowed average distance to the reference line
    @param[in] edge_radius - search radius for edge detection
    @param[in] maxNeighbors - The maximum number of neighbors a point is allowed to have within the search radius to be considered as an edge
    @param[in] leafSize - leafSize for the downsampling
    @return
    *true if a line within the thresholds was found, false otherwise
    */
    bool findLine(PTC::Ptr plane, PTC::Ptr line, pcl::ModelCoefficients::Ptr coefficients, double distThresh, int maxIterations,double minInlierRatio, Eigen::Vector3f* axis = new Eigen::Vector3f(),double epsAxis = 0, PTC::Ptr ref = PTC::Ptr(new PTC),double epsRef = 0, double edge_radius=0, int maxNeighbors=0, double leafSize=0);
    /**Detection of planes.
    @param[in,out] cloud - cloud used for the line detection. The cloud without the plane is returned
    @param[out] plane - a pointcloud representing the plane found
    @param[out] coefficients - the coefficients of the plane found
    @param[in] normals - the surface normals of the cloud
    @param[in] normal_weight - the weight of the deviation in the surface normals in comparison to the deviation in the position
    @param[in] distThresh - distance Threshold for the feature detection
    @param[in] maxIterations - the number of iterations for the feature detection
    @param[in] minInlierRatio - the minimum number of inliers in relation to the maximum number of possible inliers a feature must have to be considered as valid
    @param[in] normalAxis - the expected normal axis of the plane
    @param[in] epsAngle - the allowed angular deviation to the given axis in rad
    @param[in] distanceFromOrigin - the expected distance between the plane and the origin
    @param[in] epsDist - the allowed deviation in distance
    @param[out] projectedPlane - the plane with all inliers projected to the found model
    @return
    *true if a plane within the thresholds was found, false otherwise
    */
    bool findPlane(PTC::Ptr cloud, PTC::Ptr plane, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::Normal>::Ptr normals, double normal_weight, double distThresh, int maxIterations, double minInlierRatio, Eigen::Vector3f* normalAxis = new Eigen::Vector3f(), double epsAngle = 0, double distanceFromOrigin = 0, double epsDist = 0, PTC::Ptr projectedPlane = PTC::Ptr(new PTC));
    /**Detection of cylinders.
    @param[in,out] cloud - cloud used for the line detection. The cloud without the cylinder is returned
    @param[out] cyl - a pointcloud representing the cylinder found
    @param[out] coefficients - the coefficients of the cylinder found
    @param[in] normals - the surface normals of the cloud
    @param[in] normal_weight - the weight of the deviation in the surface normals in comparison to the deviation in the position
    @param[in] distThresh - distance Threshold for the feature detection
    @param[in] maxIterations - the number of iterations for the feature detection
    @param[in] minInlierRatio - the minimum number of inliers in relation to the maximum number of possible inliers a feature must have to be considered as valid
    @param[in] axis - the expected center axis of the cylinder
    @param[in] epsAngle - the allowed angular deviation to the given axis in rad
    @param[in] radius - the expected radius of the cylinder
    @param[in] epsRadius - the allowed deviation in radius
    @param[in] center - expected center point of the cylinder
    @param[in] epsCenter - allowed deviation with respect to the center
    @param[out] projectedCyl - the cylinder with all inliers projected to the found model
    @return
    *true if a cylinder within the thresholds was found, false otherwise
    */
    bool findCyl(PTC::Ptr cloud, PTC::Ptr cyl, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::Normal>::Ptr normals, double normal_weight, double distThresh, int maxIterations, double minInlierRatio, Eigen::Vector3f* axis = new Eigen::Vector3f(), double epsAxis = 0, double radius = 0, double epsRadius = 0, Eigen::Vector3f* center = new Eigen::Vector3f(), double epsCenter = 0,PTC::Ptr projectedCyl = PTC::Ptr(new PTC));
    /**Detection of generic features.
    @attention: The projectInliers Filter contains bugs for cylinder in PCL 1.9.1. Thus the code was reimplemented manually.
    @param[in] seg - the segmentation object, which already has been initialized with the model type and all necessary constraints for the feature detection.
    @param[in,out] remainingCloud - cloud used for the feature detection. The cloud without the feature is returned
    @param[out] featureCloud - a pointcloud representing the feature found
    @param[out] coefficients - the coefficients of the feature found
    @param[in] normals - the surface normals of the cloud
    @param[in] normal_weight - the weight of the deviation in the surface normals in comparison to the deviation in the position
    @param[in] distThresh - distance Threshold for the feature detection
    @param[in] maxIterations - the number of iterations for the feature detection
    @param[in] minInliers - the minimum number of inliers a feature must have to be considered as valid
    @param[in] projectInliers - If true, all inliers are projected to the model
    @param[out] projectedCloud -  the feature with all inliers projected to the found model
    @param[out] featureNormals - the normals of the feature found
    @return
    *true if a cylinder within the thresholds was found, false otherwise
    */
    bool findFeature(pcl::SACSegmentationFromNormals<PT, pcl::Normal> seg, PTC::Ptr remainingCloud, PTC::Ptr featureCloud, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::Normal>::Ptr normals, double normalWeight, double distThresh, int maxIterations,int minInliers,bool projectInliers, PTC::Ptr projectedCloud=PTC::Ptr(new PTC), pcl::PointCloud<pcl::Normal>::Ptr featureNormals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>));
    /**Algorithm for checking all tolerances of a given tolerance definition. Measured is set to -1, when a feature could not be found
    @param[in] cam - the cloud in which the features should be found and measured
    @param[in,out] tolerances - the vector of tolerances used. It is returned with the measured value.
    @param[in] edge_radius - search radius for edge detection
    @param[in] maxNeighbors - The maximum number of neighbors a point is allowed to have within the search radius to be considered as an edge
    @param[in] leafSize - leafSize for the downsampling
    @param[in] minInlierRatio - the minimum number of inliers in relation to the maximum number of possible inliers a feature must have to be considered as valid
    @param[in] maxTolerance - the maximal detectable tolerance during the tolerance check
    @param[in] numberOfIterations_Tol - the number of iterations for the feature detection
    @param[in] plane_normal_weight - the weight of the deviation in the surface normals in comparison to the deviation in the position for the plane detection
    @param[in] cyl_normal_weight - the weight of the deviation in the surface normals in comparison to the deviation in the position for the cylinder detection#
    @param[in] plane_dist_thresh - distance Threshold for the plane detection
    @param[in] line_dist_thresh - distance Threshold for the line detection
    @param[in] circle_dist_thresh - distance Threshold for the circle detection
    @param[in] plane_epsDist - the allowed deviation in distance for the plane detection
    @param[in] circle_epsCenter - allowed deviation with respect to the center for the circle detection
    @param[in] angularThres - the allowed angular deviation to the respetively given axes in rad
    @param[in] angularThres - the allowed angular deviation to the respetively given axes in rad for angular tolerances
    */
    void checkTolerances(PTC::Ptr cam, std::vector<Dimension>& tolerances, double edge_radius, int maxNeighbors, double leafSize, double minInlierRatio, double maxTolerance, int numberOfIterations_Tol, double tolerance_search_radius, double plane_normal_weight, double cyl_normal_weight,double plane_dist_thresh, double line_dist_thresh, double circle_dist_thresh,  double plane_epsDist, double circle_epsCenter, double cyl_epsCenter, double angleThres, double angular_angleThres);
    /**Visualizes all tolerances.
    @param[in] cam - the cloud on which the tolerance definition is based
    @param[in] tolerances - the vector of tolerances used
    @param[in] name - name of the visualizer window
    */
    void visualizeTolerances(PTC::Ptr cam, std::vector<Dimension> tolerances,std::string name="Tolerances");
};
#endif