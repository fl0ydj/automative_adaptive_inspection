#ifndef FILEIO_H_
#define FILEIO_H_
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <pcl/PolygonMesh.h>
#include <JtTk/JtkCADImporter.h>
#include <JtTk/JtkEntityFactory.h>
#include <JtTk/JtkPMI.h>
#include <JtTk/JtkXTBody.h>
#include <JtTk/JtkXTRegion.h>
#include <JtTk/JtkXTShell.h>
#include <JtTk/JtkXTFacade.h>
#include <JtTk/JtkXTFace.h>
#include <JtTk/JtkXTLoop.h>
#include <JtTk/JtkXTFin.h>
#include <JtTk/JtkXTEdge.h>
#include <JtTk/JtkXTSpoke.h>
#include <JtTk/JtkXTVertex.h>

#include <JtTk/JtkXTSurface.h>
#include <JtTk/JtkXTBSurf.h>
#include <JtTk/JtkXTPlane.h>
#include <JtTk/JtkXTCylinder.h>
#include <JtTk/JtkXTCone.h>
#include <JtTk/JtkXTSphere.h>
#include <JtTk/JtkXTTorus.h>
#include <JtTk/JtkXTOffset.h>
#include <JtTk/JtkXTSwept.h>
#include <JtTk/JtkXTSpun.h>
#include <JtTk/JtkXTMesh.h>

#include <JtTk/JtkXTCurve.h>
#include <JtTk/JtkXTBCurve.h>
#include <JtTk/JtkXTLine.h>
#include <JtTk/JtkXTCircle.h>
#include <JtTk/JtkXTEllipse.h>
#include <JtTk/JtkXTSPCurve.h>
#include <JtTk/JtkXTPolyline.h>

#include <JtTk/JtkXTPoint.h>
#include "Dimension.hpp"
typedef pcl::PointXYZ PT;
typedef pcl::PointCloud<PT> PTC;
class FileIO
{
private:
public:
    std::vector<Dimension> tolerances;
    FileIO() {};
    void loadParameters(std::map<std::string, std::string>& parameters);
    void saveParameters(std::map<std::string, std::string>& parameters);
    bool loadCloud(std::string text, PTC::Ptr cloud, std::string def="", bool scalingRequired=false);
    bool loadTolerances();
    bool queryScaling(std::string path,PTC::Ptr cloud=PTC::Ptr(new PTC));
};
#endif