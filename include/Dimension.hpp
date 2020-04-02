#ifndef DIMENSION_HPP_
#define DIMENSION_HPP_
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
typedef pcl::PointXYZ PT;
typedef pcl::PointCloud<PT> PTC;
struct Dimension {
    std::vector<PTC::Ptr> references;
    std::vector<double> coeffs;
    int type; //1 - linear 2 - angular 3 - Radial 4 - curve length
    PT textposition;
    double value, upperDelta, lowerDelta;
    double measured;
    std::vector<PTC::Ptr> measuredReferences;
};
#endif