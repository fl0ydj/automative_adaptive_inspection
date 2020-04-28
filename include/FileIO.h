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
/**@brief Class for file input and output for Clouds, Tolerance Definitions, Matrices and the Results format.
@author Justin Heinz, Siemens PLM Software
*/
class FileIO
{
private:
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**Vector of type Dimension to store the tolerances.*/
    std::vector<Dimension> tolerances;
    /**Default constructor.*/
    FileIO() {};
    /**Reads in parameters.txt. All parameters are stored using the structure "Key:Value\n".
    If the file does not exist, saveparameters(parameters) is called.
    @param[in,out] parameters - the map of parameters
    */
    void loadParameters(std::map<std::string, std::string>& parameters);
    /**Writes parameters to parameters.txt. All parameters are stored using the structure "Key:Value\n".
    @param[in] parameters - the map of parameters
    */
    void saveParameters(std::map<std::string, std::string>& parameters);
    /**Query a user for a path to a pointcloud and load it. Supported file formats: .pcd, .stl, .ply, .txt, .jt
    For .txt files, loadResults(path) is called. For .jt files, loadTolerances(path) is called.
    @param[in] text - Text to show to the user to ask for a path.
    @param[out] cloud - the cloud loaded as a response
    @param[in] def - When set, it is the default path name to use, when the user types "default".
    @param[in] scalingRequired - When set, queryScaling is called.
    @return
    * true when the cloud was loaded successfully, false when an error occured
    */
    template<typename T>
    bool loadCloud(std::string text, typename pcl::PointCloud<T>::Ptr cloud, std::string def="", bool scalingRequired=false);
    /**Save a pointcloud in the .pcd format.
    @param[in] path - Path to the location where the cloud should be saved.
    @param[in] cloud - the cloud to be saved.
    */
    template<typename T>
    void saveCloud(std::string path, typename pcl::PointCloud<T>::Ptr cloud);
    /**Load tolerances for the tolerance comparison.
    At the end queryScaling() is called.
    Source: Parts of this procedure have been copied from an example by Siemens PLM.
    @param[in] path - Path to load the tolerance definition from. When empty, the user is queried to enter a path.
    @return
    * true when the tolerance definition was loaded successfully, false when an error occured
    */
    bool loadTolerances(std::string path="");
    /**Query the user to enter the unit which is used for the cloud. The cloud is scaled accordingly. Supported: mikro,mm,cm,dm,m.
    @param[in] path - The path to the cloud.
    @param[in,out] cloud - The pointer to the scaled cloud. If empty, tolerance is scaled instead.
    @return
    * true when successful, false when an error occured or an invalid unit was entered.
    */
    template<typename T>
    bool queryScaling(std::string path, typename pcl::PointCloud<T>::Ptr cloud= pcl::PointCloud<T>::Ptr(new pcl::PointCloud<T>));
    /**Save the tolerances in Results.txt. Format: id:textposition:value:measured:ok
    */
    void saveResults();
    /**Load in the results file.
    @param[in] path - The path to the results file.
    @return
    * true when successful, false when an error occured or if the results file is in an invalid format
    */
    bool loadResults(std::string path);
    /**Save an Eigen Matrix with a specified format in a text file.
    @param[in] path - Where to save the text file
    @param[in] mat - Matrix to save
    */
    template<typename scalar,int rows, int cols>
    void saveMatrix(std::string path, typename Eigen::Matrix<scalar,rows,cols>& mat) {
        cout << "Saving matrix to " << path << "...\n";
        std::ofstream file(path, std::ofstream::out | std::ofstream::trunc);
        if (file.is_open()) {
            file << mat;
        }
        file.close();
    }
    /**Load an Eigen Matrix from a text file.
    @param[in] path - Where to load the matrix from
    @param[out] mat - Loaded matrix
    @return
    * true when successful, false when an error occured
    */
    template<typename scalar,int rows, int cols>
    bool loadMatrix(std::string path, Eigen::Matrix<scalar,rows,cols>& mat) {
        cout << "Loading matrix from " << path << "...\n";
        ifstream infile;
        try {
            infile.open(path);
            int rows = 0;
            while (!infile.eof())
            {
                std::string line;
                getline(infile, line);
                int cols = 0;
                std::stringstream stream(line);
                while (!stream.eof())
                    stream >> mat.coeffRef(rows, cols++);
                rows++;
            }
        }
        catch (std::exception e) {
            PCL_ERROR("Could not load textfile!\n");
            infile.close();
            return false;
        }
        infile.close();
        return true;
    }
};
#endif