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

typedef pcl::PointXYZ PT;
typedef pcl::PointCloud<PT> PTC;
struct CADFeature {
    PTC::Ptr CADhull;
    PTC::Ptr CADcloud;
    pcl::ModelCoefficients::Ptr CADcoeffs;
    PTC::Ptr Measuredhull;
    PTC::Ptr MeasuredCloud;
    pcl::ModelCoefficients::Ptr Measuredcoeffs;
    pcl::SacModel type;
    bool found;
    CADFeature(PTC::Ptr CADhull, PTC::Ptr CADcloud, pcl::ModelCoefficients::Ptr coeffs,pcl::SacModel type) { this->CADhull = CADhull; this->CADcloud = CADcloud;  this->CADcoeffs = coeffs; this->type = type; }
};
class Comparison
{
private:
    int debug;
    double scale;
public:
    std::vector<PTC::Ptr> compFeatures;
    std::vector<PTC::Ptr> camFeatures;
    std::vector<CADFeature> CADFeatures;
    Comparison(std::string debug);
    bool hausdorff(PTC::Ptr cam,PTC::Ptr comp, pcl::PointCloud<pcl::PointXYZRGB>::Ptr comp_hausdorff, std::vector<int>& defects, float& hausdorff,float& avg_dist, bool outlierDeletion = true);
    void detectFeatures(PTC::Ptr comp,bool showFeatures);
    void parametrizeFeatures();
    bool findCircle(PTC::Ptr plane, pcl::ModelCoefficients::Ptr planeCoeffs, PTC::Ptr circle, pcl::ModelCoefficients::Ptr coefficients, double distThresh, double radius=0, double radiusLowerDelta=0, double radiusUpperDelta=0, double epsCenter = 0, Eigen::Vector3f* center = new Eigen::Vector3f());
    bool findLine(PTC::Ptr plane, pcl::ModelCoefficients::Ptr planeCoeffs, PTC::Ptr line, pcl::ModelCoefficients::Ptr coefficients, double distThresh, double epsAxis = 0, Eigen::Vector3f* axis = new Eigen::Vector3f(), double epsRef = 0, PTC::Ptr ref = PTC::Ptr(new PTC));
    bool findPlane(PTC::Ptr cloud, PTC::Ptr plane, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::Normal>::Ptr normals, double normal_weight, double distThresh, double epsAngle=0, Eigen::Vector3f* normalAxis= new Eigen::Vector3f(), double epsDist=0, double distanceFromOrigin=0);
    bool findFeature(pcl::SACSegmentationFromNormals<PT, pcl::Normal> seg, PTC::Ptr remainingCloud, pcl::PointCloud<pcl::Normal>::Ptr normals, double distThresh, double normalWeight, pcl::ModelCoefficients::Ptr coefficients, PTC::Ptr featureCloud, pcl::PointCloud<pcl::Normal>::Ptr featureNormals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>));
    void findMeasuredFeatures(PTC::Ptr cam, PTC::Ptr comp);
    void visualizeFeatures(PTC::Ptr cam, PTC::Ptr comp);
    void checkTolerances(PTC::Ptr cam,std::vector<Dimension>& tolerances);
};
#endif