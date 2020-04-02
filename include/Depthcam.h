#ifndef DEPTHCAM_H_
#define DEPTHCAM_H_
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/PointIndices.h>
#include <pcl/common/common.h>
#include <vtkPLYReader.h>
#include <pcl/filters/extract_indices.h>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core/eigen.hpp>
#include <pcl/filters/passthrough.h>
#include <fstream>

typedef pcl::PointXYZ PT;
typedef pcl::PointCloud<PT> PTC;
/*Realsense Wrapper class*/
class Depthcam
{
private:
	int debug;
    rs2::context ctx;
    rs2::config cfg;
    rs2::pipeline pipe;
    rs2::pipeline_profile profile;
	cv::Mat color_mat;
	double markerLength;
public:
	PTC::Ptr cloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB;
    double scale;
	rs2::device dev;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW Eigen::Matrix4f ColorToDepth;
	Eigen::Matrix4f ExtrinsicMatrix;
	cv::Mat IntrinsicMatrix;
	cv::Vec<double, 5> DistortionCoeff;
	Eigen::Matrix4f FromMasterCam;
    PTC::Ptr background;
	cv::Ptr<cv::aruco::Dictionary> dictionary = new cv::aruco::Dictionary;
	cv::Ptr<cv::aruco::CharucoBoard> board = new cv::aruco::CharucoBoard;
	//Error Assessment
	std::vector<cv::Vec3d> corner_tvecs;
	std::vector<std::vector<cv::Point2f>> corners;
	std::vector<int> ids;
	std::vector<double> ReprojectionErrors;
	/*
	* Create the depthcam object, which provides the functions to interact with the realsense library
	* @param ctx: context the device is located in [input]
	* @param dev: pointer to the intel Realsense camera [input]
	* @return void
	*/
    Depthcam(rs2::context ctx, rs2::device& dev,std::string preset, int disparity_shift,std::string debug);
	void stop();
	/*
* Uses filters to remove outliers.
* @param cloud: the cloud to be filtered. [output]
* @return void
*/
	void postprocess(PTC::Ptr cloud);
	void ApplyPreset(std::string path, rs400::advanced_mode& advanced_mode_dev);
	void SetupDictionary(int width, int height, double squareLength, double markerLength, cv::aruco::PREDEFINED_DICTIONARY_NAME dict_name);
    void createAccuratePCLPointCloud();
	/*
	* Wait for frames of the camera, than create the librealsense point cloud and convert it to the PCL format.
	* @param cloud: pointer to the generated cloud [output]
	* @return void
	*/
   // void createPCLPointCloud(PTC::Ptr cloud);
	/*
	* Wait for frames of the camera, than create the librealsense point cloud and convert it to the PCL format.
	* @param cloud: pointer to the generated cloud [output]
	* @return void
	*/
	void createPCLPointCloud();
	/*
	* Sets the background pointer of the instance to cloud and saves the background as a .pcd file.
	* @param cloud: the cloud to be used as a background [input]
	* @return void
	*/
    void saveBackground();
	/*
	* Loads the background .pcd file and assigns it to the background cloud pointer.
	* @return void
	*/
    void loadBackground();
	/*
	* Detects changes between the background and cloud and only keeps the new voxels in cloud.
	* @param cloud: the cloud to checked for changes in comparison with the background [output]
	*TODO
	* @return void
	*/
    //void removeBackground(PTC::Ptr match_cloud, PTC::Ptr match_cloud_out, PTC::Ptr cloud);
	void removeBackground(PTC::Ptr cloud, bool RemoveUnderground);
	bool detectChArUcoCorners(std::vector<cv::Point2f>& charucoCorners, std::vector<int>& charucoIds,bool Intrinsics);
	bool estimateExtrinsics(std::vector<int> charucoIds, std::vector<cv::Point2f> charucoCorners);
	bool calibrateIntrinsics();
	/*
	* Performs the extrinsic camera calibration usign a CharUco board and saves the extrinsic matrix in a file.
	* @return bool: flag whether calibration was successful
	*/
    bool calibrateExtrinsics();
	/*
	* Loads the file, containing the transformation to the main cam.
	* @return void
	*/
    void loadFromMasterCam();
	void loadIntrinsics();
	/*
	* Performs the extrinsic camera calibration usign the extrinsic matrix of the main camera.
	* @return bool: flag whether calibration was successful
	*/
	void determineFromMasterCam(Eigen::Matrix4f& MasterCam_ExtrinsicMatrix, Eigen::Matrix4f& MasterCam_ColorToDepth);
	/*
	* Performs the extrinsic camera calibration usign the extrinsic matrix of the main camera.
	* @return bool: flag whether calibration was successful
	*/
	void calibrateStatic(Eigen::Matrix4f& MasterCam_ExtrinsicMatrix, Eigen::Matrix4f& MasterCam_ColorToDepth);
	/*
	
	*/
	void refineCalibration(Eigen::Matrix4f& error);
	/*
	* Returns the device info representing the realsense camera.
	* @return std::string: Device number
	*/
    std::string getDeviceInfo();
	cv::Mat getMat();
	void calibrationError(std::vector<cv::Vec3d> master_tvecs, std::vector<int> master_ids, Eigen::Matrix4f& master_extrinsics);
};
#endif