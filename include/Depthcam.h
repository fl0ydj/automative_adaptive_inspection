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
#ifndef DEPTHCAM_H_
#define DEPTHCAM_H_
#include "Visual.h"
#include "Registration.h"
#include <string>
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <pcl/PointIndices.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core/eigen.hpp>
#include <pcl/filters/passthrough.h>
#include <fstream>

typedef pcl::PointXYZ PT;
typedef pcl::PointCloud<PT> PTC;
/**@brief Real Sense Wrapper class.

This class allows to create pointclouds, remove the background in a pointclouds, perform the RGB intrinsic and extrinsic calibration.
Sources: https://docs.opencv.org/3.4.9/df/d4a/tutorial_charuco_detection.html, https://github.com/IntelRealSense/librealsense/issues/1601, https://github.com/IntelRealSense/librealsense/issues/1021
@author Justin Heinz
*/
class Depthcam
{
private:
	/**Debug level (0 - none,1 - some,2 - all)*/
	int debug;
	/***Pipeline for the communcation to the camera.*/
    rs2::pipeline pipe;
	/**Length of one marker of the charuco board.*/
	double markerLength;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	/**Last recorded color matrix.*/
	cv::Mat color_mat;
	/**Last recorded point cloud*/
	PTC::Ptr cloud;
	/**Last recorded colored point cloud*/
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB;
	/**Reference to the realsense device*/
	rs2::device dev;
	/**Extrinsic Matrix between the color stream and the depth stream.*/
	Eigen::Matrix4f ColorToDepth;
	/**Extrinsic Matrix between the color stream and the world coordinate*/
	Eigen::Matrix4f ExtrinsicMatrix;
	/**Intrinsic Matrix of the color stream*/
	Eigen::Matrix3d IntrinsicMatrix; 
	/**Distortion coefficients of the color stream*/
	Eigen::Matrix<double,5,1> DistortionCoeff;
	/**Transformation between the color stream and the color stream of the master camera*/
	Eigen::Matrix4f FromMasterCam;
	/**Point cloud of the background (charuco board)*/
    PTC::Ptr background;
	/**Dictionary used for the charuco board*/
	cv::Ptr<cv::aruco::Dictionary> dictionary = new cv::aruco::Dictionary;
	/**The charuco board used*/
	cv::Ptr<cv::aruco::CharucoBoard> board = new cv::aruco::CharucoBoard;
	//Error Assessment
	/**The translation vectors from the color stream to the individual corners of the charuco board*/
	std::vector<cv::Vec3d> corner_tvecs;
	/**The 2D points of the corners detected*/
	std::vector<std::vector<cv::Point2f>> corners;
	/**The IDs of the corners detected*/
	std::vector<int> ids;
	/**Vector of reprojection errors*/
	std::vector<double> ReprojectionErrors;
	/**Vector of numbers of detected markers*/
	std::vector<int> DetectedMarkers;
	/** Create the depthcam object, which provides the functions to interact with the realsense library.
	@param[in] ctx - context of the real sense camera
	@param[in] dev - device of the real sense camera
	@param[in] preset - preset which should be used for recording
	@param[in] disparity_shift - the disparity shift to use
	@param[in] laser_power - the value for the laser power
	@param[in] debug - Debug level(0 - none,1 - some,2 - all)
	*/
    Depthcam(rs2::context ctx, rs2::device& dev,std::string preset, int disparity_shift, int laser_power,std::string debug);
	/**Stops the pipeline.*/
	void stop();
	/**
	Apply a preset to the real sense camera
	@param[in] path - path to the preset json file
	@param[in] advanced_mode_dev - pointer to the device in advanced mode
	*/
	void ApplyPreset(std::string path, rs400::advanced_mode& advanced_mode_dev);
	/**
	Setup the dictionary of the charuco board.
	@param[in] width - number of markers in horizontal direction
	@param[in] height - number of markers in vertical direction
	@param[in] squareLenght - length of one square
	@param[in] markerLenght - length of one marker
	@param[in] dict_name - type of dictionary to use
	*/
	void SetupDictionary(int width, int height, double squareLength, double markerLength, cv::aruco::PREDEFINED_DICTIONARY_NAME dict_name);
	/**Wait for frames of the camera, than create the librealsense point cloud and convert it to the PCL format.
	@param[in] spat_alpha - alpha parameter for the spatial filter
	@param[in] spat_delta - delta parameter for the spatial filter
	@param[in] spat_magnitude - magnitude parameter for the spatial filter
	@param[in] temp_alpha - alpha parameter for the temporal filter
	@param[in] temp_delta - delta parameter for the temporal filter
	@param[in] temp_persistency - persistency parameter for the temporal filter
	*/
	void createPCLPointCloud(double spat_alpha, double spat_delta, double spat_magnitude, double temp_alpha, double temp_delta, int temp_persistency);
	/**
	Remove everything which is not on the charuco board.
	@param[in,out] cloud - the cloud whose background should be removed
	@param[in] RemoveUnderground - When true, also remove the points of the charuco board itself
	*/
	void removeBackground(PTC::Ptr cloud, bool RemoveUnderground);
	/**Detect charuco corners. Source: https://docs.opencv.org/3.4.9/df/d4a/tutorial_charuco_detection.html
	@param[out] charucoCorners - the 2D positions of the detected corners
	@param[out] charucoIds - the IDs of the detected corners
	@param[in] Intrinsics - When true, uses the intrinsic matrix to derive the translation vectors to the individual corners
	@return 
	* true when some corners could be detected, false if not enough corners have been found
	*/
	bool detectChArUcoCorners(std::vector<cv::Point2f>& charucoCorners, std::vector<int>& charucoIds,bool Intrinsics);
	/**Uses a set of charuco corners to derive the extrinsic matrix. Source: https://docs.opencv.org/3.4.9/df/d4a/tutorial_charuco_detection.html
	@param[out] charucoIds - the IDs of the detected corners
	@param[out] charucoCorners - the 2D positions of the detected corners
	@return
	* true when the extrinsic matrix could be determined, false when not enough markers could be used
	*/
	bool estimateExtrinsics(std::vector<int> charucoIds, std::vector<cv::Point2f> charucoCorners);
	/**Performs the RGB intrinsic calibration using 10 pictures of a charuco board.
	@return 
	* true when successful, false otherwise
	*/
	bool calibrateIntrinsics();
	/**Performs the extrinsic calibration between the color stream and the world coordinate system using a charuco board.
	@return
	* true when successful, false otherwise
	*/
    bool calibrateExtrinsics();
	/** Determine the cam-to-cam transformation based on its own extrinsic matrix.
	@param[in] MasterCam_ExtrinsicMatrix - extrinsic matrix of the master camera
	*/
	void determineFromMasterCam(Eigen::Matrix4f& MasterCam_ExtrinsicMatrix);
	/** Determine the extrinsic matrix based on the cam-to-cam transformation.
	@param[in] MasterCam_ExtrinsicMatrix - extrinsic matrix of the master camera
	*/
	void calibrateStatic(Eigen::Matrix4f& MasterCam_ExtrinsicMatrix);
	/** Refines the extrinsic calibration.
	@param[in] error - matrix representing the calibration error
	*/
	void refineCalibration(Eigen::Matrix4f& error);
	/** Returns the device info representing the realsense camera.
	@return
	* device number
	*/
    std::string getDeviceInfo();
	/**Calculates the error of the cam-to-cam calibration.
	@param[in] master_tvecs - translational vector of all corners detected by the master cam
	@param[in] master_ids - ids of all corners detected by the master cam
	@param[in] master_extrinsics - extrinsic matrix of the master camera
	*/
	void transformationError(std::vector<cv::Vec3d> master_tvecs, std::vector<int> master_ids, Eigen::Matrix4f& master_extrinsics);
};
#endif