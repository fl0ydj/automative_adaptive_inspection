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
/** @mainpage Documentation
This is the code documentation for the Bachelor's Thesis "Adaptive Analysis of Geometric Tolerances with Low-Cost Sensors".
How to setup the programming environment
===========================================
In the following it is described briefly how to setup all necessary libraries used in this project.

The references are set mostly using variables in CMakeLists.txt which have to be updated accordingly. Alternatively, environment variables could be set.
Visual Studio and Cmake
-----------------------
%Visual Studio 2019 was used and can be downloaded here: https://visualstudio.microsoft.com/vs/.
%Visual Studio Cmake Tools have to be installed.

Furthermore, Cmake itself has to be installed. The minimum version is 3.6: https://cmake.org/download/.
Turntable HW-Control
--------------------
The files have to be downloaded from the github directory and the source code has to be built.

Afterwards, in CMakeLists TURNTABLE_DIR has to be adjusted and DLL_TURNTABLE has to be checked.
Realsense SDK
-------------
The realsense sdk can be downloaded here: https://github.com/IntelRealSense/librealsense/releases.

Version 2.33.1 was used for this project.
Adjust REALSENSE_DIR in CmakeLists accordingly.
Point Cloud Library 
-------------------
PCL can be downloaded here: https://github.com/PointCloudLibrary/pcl/releases.
The Version used for this project was 1.9.1.

Using the windows installer, an environment variable should be set automatically. If not, add it manually: PCL_ROOT (to the root directory of PCL).

Furthermore add %PCL_ROOT%\bin to your PATH variable.
Open CV
-------
Download OpenCV and compile it: https://github.com/opencv/opencv.

OpenCV contrib has to be downloaded and built as well (some modules are used from that): https://github.com/opencv/opencv_contrib.

Adjust OPENCV_DIR accordingly. 
JT Open Library
---------------
JT Open Libary can be downloaded here: https://www.plm.automation.siemens.com/store/en-us/trial/jt2-open.html.

After registration, a 60 days trial version begins.

In CMakeLists the variables JTTK_LIB and JTTK_INCLUDE have to be adjusted accordingly.
 */
#ifndef MENU_H_
#define MENU_H_
#include "Depthcam.h"
#include "Comparison.h"
#include "FileIO.h"
#include "HW_control.h"
#include "Serial.h"
#include "HW_control_Export.h"
typedef pcl::PointXYZ PT;
typedef pcl::PointCloud<PT> PTC;
/**@brief Core Class for all functions the user can execute.

This class is used to perform the Image Recording using the Depthcam class, the Registration and the Comparison. The user can invoke the member functions. Furthermore, all parameters used in this program are defined here.
@author Justin Heinz
*/
class Menu {
private:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	/**An instance of FileIO.*/
	FileIO io;
	/**Vector to store the Depthcam instances.*/
	std::vector<Depthcam> Cams;
	/**Index of the master camera in the Cams vector.*/
	int MasterCamIndex = 0;
	/**Number of cameras.*/
	int camcount = 0;
	/** Map of strings and according variables to store the parameters. The values defined in these variables are the default ones. The actual values are loaded from parameters.txt at the program start.*/
	std::map<std::string, std::string>  parameters;
	/**General Debug Level: 0 - no Debug Output, 1 - some Debug Output, 2 - all Debug Output.*/
	std::string debug = "0";
	/**ID of the master camera*/
	std::string master_cam = "934222070559";
	/**The minimal leaf size used to downsample the clouds.*/
	std::string minLeafSize="0.0005";
	//1. Image Recording
	/**The preset json file used for the realsense cameras.*/
	std::string preset = "HighAccuracy.json";
	/**The disparity shift.*/
	std::string disparity_shift = "225";
	/**The value for the laser power*/
	std::string laser_power = "30";
	/**The alpha parameter for the spatial filter*/
	std::string spat_alpha="0.95";
	/**The delta parameter for the spatial filter*/
	std::string spat_delta="5";
	/**The magnitude parameter for the spatial filter*/
	std::string spat_magnitude="1";
	/**The alpha parameter for the temporal filter*/
	std::string temp_alpha="0.1";
	/**The delta parameter for the temporal filter*/
	std::string temp_delta="5";
	/**The persistency parameter for the temporal filter*/
	std::string temp_presistency="7";
	/**Enables(1)/disables(0) the error correction using ICP.*/
	std::string errorCorrection = "0";
	/**The number of points used in the Statistical Outlier Removal filter for calculating the mean.*/
	std::string postprocess_MeanK = "400";
	/**The allowed deviation of the point with respect to the mean in the Statistical Outlier removal filter as a multiple of the standard deviation.*/
	std::string postprocess_thresh = "0.5";
	//2. Registration
	/**Used to calculate the leafsize: The maximum distance between two points in the component cloud divided by leafPoints*/
	std::string leafPoints = "100";
	/**The search radius used for the normals estimation as a multiple of the leaf size used for downsampling.*/
	std::string normalsSearchRadius = "10";
	/**The search radius used for fast point feature historgram calculation as a multiple of the leaf size used for downsampling.*/
	std::string FPFHSearchRadius = "20";
	/**Used to calculate the minimum sample distance: The maximum distance between two points in the component cloud divided by samplesInObjectSize*/
	std::string samplesInObjectSize = "1.2";
	/**The maximum correspondence distance used for the SAC initial alignment.*/
	std::string maxCorrespondenceDistance_SAC = "0.15";
	/**The number of iterations used for the SAC inital alignment.*/
	std::string numberOfIterations_SAC = "1000";
	/**The maximum correspondence distance used for ICP.*/
	std::string maxCorrespondenceDistance_ICP = "0.05";
	/**The maximum allowable fitness score to consider ICP to have converged.*/
	std::string maxFitnessScore_ICP = "0.00000001";
	/**The maximum number of iterations used for ICP.*/
	std::string numberOfIterations_ICP = "5000";
	//3. Comparison
	/**Deivations greater than this parameter are deleted as outliers*/
	std::string threshold_outlier ="0.02";
	/**Deviations greater than this parameter are marked as defects*/
	std::string threshold_defect ="0.0015";
	/**Deviations greater than this parameter are marked as warnings.*/
	std::string threshold_warning ="0.0005";
	/**The search radius used for the radius outlier removal filter in the edge detection.*/
	std::string edge_radius = "0.002";
	/**The maximum number of neighbors a point is allowed to have within the search radius to be considered as an edge in the edge detection.*/
	std::string maxNeighbors = "35";
	/**The minimum number of inliers in relation to the maximum number of possible inliers a feature must have to be considered as valid.*/
	std::string minInlierRatio = "0.5";
	/**The maximal detectable tolerance during the tolerance check.*/
	std::string maxTolerance ="0.02";
	/**The maximum number of iterations used for the SAC segmentation during the tolerance check.*/
	std::string numberOfIterations_Tol= "1000";
	/**The search radius used for normal estimation during the tolerance check.*/
	std::string tolerance_search_radius ="0.0025";
	/**The allowed angular deviation for the axis of features in radiants*/
	std::string angleThres = "0.2617994"; //15
	/**The allowed angular deviation used for angular tolerances.*/
	std::string angular_angleThres = "1.570796"; //90
	/**The weight of the deviation in the surface normals in comparison to the deviation in the position during plane detection.*/
	std::string plane_normal_weight ="0";
	/**The weight of the deviation in the surface normals in comparison to the deviation in the position during cylinder detection.*/
	std::string cyl_normal_weight ="0";
	/**The distance threshold for plane detection*/
	std::string plane_dist_thresh ="0.0005";
	/**The distance threshold for line detection*/
	std::string line_dist_thresh ="0.0005";
	/**The distance threshold for circle detection*/
	std::string circle_dist_thresh ="0.0005";
	/**The allowed deviation in the position of the plane.*/
	std::string plane_epsDist ="0.005";
	/**The allowed deviation in the position of the center of the cirlce.*/
	std::string circle_epsCenter ="0.005";
	/**The allowed deviation in the position of the center of the cylinder,*/
	std::string cyl_epsCenter ="0.005";
public:
	/**Standard constructor*/
	Menu();
	/**Clear the Cams vector and try to connect to the cameras again.*/
	void reconnect();
	/**Display the help text.*/
	void displayIntro();
	/**Perform the extrinsic calibration.*/
	void extrinsics();
	/**Perform the RGB intrinsic calibration*/
	void intrinsics();
	/**Query the user how many scans should be recorded and how many degree the turntable should be rotated after each scan. Then start the recording doing the following in a loop: Extrinsic calibration of the master cam, deduce the extrinsic calibration of the slave cameras using the FromMasterCam Transformation, record point clouds, remove the background, save the clouds and rotate the turntable. Afterwards, the whole cloud is postprocessed and downsampled and then saved. */
	void record();
	/**Query the user for the measured cloud and the cloud of the component, calculate the maximum distance between two points in the component cloud and use that information to downsample both clouds. Afterwards, the normals of both clouds are estimated and they are used for the initial alignment and then for ICP. Save the cloud after registration.*/
	void registration();
	/**Merge two clouds.*/
	void merge();
	/**Query the user for the cloud after registration and the cloud of the component. Downsample both with the minimum leaf size and perform the hausdorff comparison.*/
	void hausdorff();
	/**Display all parameters and their current values. Then allow the user to type a parameter and its new value.*/
	void settings();
	/**View a point cloud, tolerance definition or the results file.*/
	void view();
	/**Record a point cloud from all cameras and show the output. Save the recorded cloud.*/
	void snap();
	/**Query the user for a tolerance definition (.jt) and a point cloud. Perform the tolerance comparison and visualize the tolerances.*/
	void tolerances();
};
#endif