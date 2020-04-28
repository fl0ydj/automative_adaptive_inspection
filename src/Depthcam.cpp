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
#include "Depthcam.h"

// Helper function
// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.
//Source: https://github.com/IntelRealSense/librealsense/issues/1601
std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)
{
	const int w = texture.get_width(), h = texture.get_height();
	int x = std::min(std::max(int(texcoords.u * w + .5f), 0), w - 1);
	int y = std::min(std::max(int(texcoords.v * h + .5f), 0), h - 1);
	int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
	const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
	return std::tuple<uint8_t, uint8_t, uint8_t>(
		texture_data[idx], texture_data[idx + 1], texture_data[idx + 2]);
}
// Helper function
// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.
//Source: https://github.com/IntelRealSense/librealsense/blob/master/wrappers/opencv/cv-helpers.hpp
cv::Mat frame_to_mat(const rs2::frame& f)
{
	auto vf = f.as<rs2::video_frame>();
	const int w = vf.get_width();
	const int h = vf.get_height();

	if (f.get_profile().format() == RS2_FORMAT_BGR8)
	{
		return cv::Mat(cv::Size(w, h), CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP);
	}
	else if (f.get_profile().format() == RS2_FORMAT_RGB8)
	{
		auto r = cv::Mat(cv::Size(w, h), CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP);
		cv::cvtColor(r, r, cv::COLOR_RGB2BGR);
		return r;
	}
	else if (f.get_profile().format() == RS2_FORMAT_Z16)
	{
		return cv::Mat(cv::Size(w, h), CV_16UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
	}
	else if (f.get_profile().format() == RS2_FORMAT_Y8)
	{
		return cv::Mat(cv::Size(w, h), CV_8UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
	}
	else if (f.get_profile().format() == RS2_FORMAT_Y16)
	{
		return cv::Mat(cv::Size(w, h), CV_16UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
	}
	else if (f.get_profile().format() == RS2_FORMAT_YUYV)
	{
		return cv::Mat(cv::Size(w, h), CV_8UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
	}
	else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
	{
		return cv::Mat(cv::Size(w, h), CV_32FC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
	}
	throw std::runtime_error("Frame format is not supported yet!");
}


Depthcam::Depthcam(rs2::context ctx, rs2::device& dev,std::string preset,int disparity_shift,int laser_power,std::string debug) {
	this->debug = stoi(debug);
	this->dev = dev;
	pipe = rs2::pipeline(ctx);
	rs2::config cfg;
	rs2::pipeline_profile profile;
	cfg.enable_device(getDeviceInfo());
	cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 15);
	cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 15);
	profile = pipe.start(cfg);
	rs2::depth_sensor depth_sensor = dev.first<rs2::depth_sensor>();
	auto advanced_mode_dev = this->dev.as<rs400::advanced_mode>();
	ApplyPreset(preset,advanced_mode_dev);
	depth_sensor.set_option(RS2_OPTION_LASER_POWER, laser_power);
	depth_sensor.set_option(RS2_OPTION_DEPTH_UNITS, 0.0001); //set length of one depth unit to 0.1 mm
	depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
	depth_sensor.set_option(RS2_OPTION_EMITTER_ON_OFF,0);
	auto advance_mode_dev_group = advanced_mode_dev.get_depth_table();
	advance_mode_dev_group.disparityShift = disparity_shift;
	advanced_mode_dev.set_depth_table(advance_mode_dev_group);
	background.reset(new PTC);
	rs2_extrinsics extrin;
	rs2_error* err;
	auto stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
	auto stream_color = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
	rs2_get_extrinsics(stream, stream_color ,&extrin, &err);
	ColorToDepth = Eigen::Matrix4f::Identity();
	ColorToDepth.block(0, 0, 3, 3) << extrin.rotation[0], extrin.rotation[3], extrin.rotation[6],
									extrin.rotation[1], extrin.rotation[4], extrin.rotation[7],
										extrin.rotation[2], extrin.rotation[5], extrin.rotation[8];
	ColorToDepth.block(0, 3, 3, 1) << extrin.translation[0], extrin.translation[1], extrin.translation[2];
	auto intrinsics = stream_color.get_intrinsics();
	double discoeffs[5];
	for (int i = 0; i < 5; i++)
		discoeffs[i] = (double)intrinsics.coeffs[i];
	IntrinsicMatrix <<intrinsics.fx, 0, intrinsics.ppx, 0, intrinsics.fy, intrinsics.ppy, 0, 0, 1;
	DistortionCoeff = Eigen::Matrix<double, 5, 1>(discoeffs);
	markerLength = 0.014308 *2.0/3.0;
	SetupDictionary(14,14, 0.014308, markerLength,cv::aruco::DICT_4X4_100);
}
void Depthcam::stop() {
	pipe.stop();
}
void Depthcam::SetupDictionary(int width, int height, double squareLength, double markerLength, cv::aruco::PREDEFINED_DICTIONARY_NAME dict_name) {
	dictionary=cv::Ptr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(dict_name));
	board=cv::aruco::CharucoBoard::create(width, height, squareLength, markerLength, dictionary);
}
void Depthcam::ApplyPreset(std::string path,rs400::advanced_mode& advanced_mode_dev) {
	std::ifstream file(path, std::ifstream::binary);
	std::string preset_json((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>()); //https://github.com/IntelRealSense/librealsense/issues/1021
	advanced_mode_dev.load_json(preset_json);
}
void Depthcam::createPCLPointCloud(double spat_alpha,double spat_delta,double spat_magnitude,double temp_alpha, double temp_delta,int temp_persistency) {
	cloud.reset(new PTC);
	cloudRGB.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	rs2::frameset fs = pipe.wait_for_frames();  //waits for frames to automatically delay the loop and not return instantly (see also poll_for_frames)
	rs2::align align_to_depth(RS2_STREAM_DEPTH);
	fs = align_to_depth.process(fs);
	rs2::pointcloud realsense_cloud;
	rs2::points points;
	rs2::depth_frame depth = fs.get_depth_frame();
	rs2::disparity_transform depth2disparity;
	depth = depth2disparity.process(depth);
	rs2::spatial_filter spat_filter(spat_alpha, spat_delta,spat_magnitude,0);
	depth = spat_filter.process(depth);
	rs2::temporal_filter temp_filter(temp_alpha, temp_delta,temp_persistency); // reduce temporal noise -> but smoothes!
	depth = temp_filter.process(depth);
	rs2::disparity_transform disparity2depth(false);
	depth = disparity2depth.process(depth);
	auto color = fs.first(RS2_STREAM_COLOR);
	realsense_cloud.map_to(color);
	points = realsense_cloud.calculate(depth);
	auto vertices = points.get_vertices();              // get vertices 
	auto tex_coords = points.get_texture_coordinates(); // and texture coordinates 
	PT point;
	pcl::PointXYZRGB pointRGB;
	for (int i = 0; i < points.size(); i++)
	{
		if (vertices[i].z)
		{
			// upload the point and texture coordinates only for points we have depth data for 
			point.x = vertices[i].x;
			point.y = vertices[i].y;
			point.z = vertices[i].z;
			pointRGB.x = vertices[i].x;
			pointRGB.y = vertices[i].y;
			pointRGB.z = vertices[i].z;
			std::tuple<uint8_t, uint8_t, uint8_t> current_color;
			current_color = get_texcolor(color, tex_coords[i]);
			pointRGB.r = std::get<0>(current_color);
			pointRGB.g = std::get<1>(current_color);
			pointRGB.b = std::get<2>(current_color);
			cloud->push_back(point);
			cloudRGB->push_back(pointRGB);
		}
	}
	pcl::transformPointCloud(*cloud, *cloud, ExtrinsicMatrix.inverse()*ColorToDepth);
	pcl::transformPointCloud(*cloudRGB, *cloudRGB, ExtrinsicMatrix.inverse() * ColorToDepth);
}
void Depthcam::determineFromMasterCam(Eigen::Matrix4f& MasterCam_ExtrinsicMatrix) {
	FromMasterCam = MasterCam_ExtrinsicMatrix * ExtrinsicMatrix.inverse();
}
void Depthcam::calibrateStatic(Eigen::Matrix4f& MasterCam_ExtrinsicMatrix) {
	ExtrinsicMatrix = FromMasterCam.inverse()* MasterCam_ExtrinsicMatrix;
}
void Depthcam::refineCalibration(Eigen::Matrix4f& error) {
	ExtrinsicMatrix = ExtrinsicMatrix* error;
}
std::string Depthcam::getDeviceInfo() {
	return dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
}
void Depthcam::removeBackground(PTC::Ptr cloud,bool RemoveUnderground) {
	pcl::PassThrough<PT> pass;
	pass.setInputCloud(cloud);
	if (RemoveUnderground) {
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.002, 0.200);
		pass.filter(*cloud);
	}
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-0, 0.200);
	pass.filter(*cloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-0, 0.200);
	pass.filter(*cloud);
}
bool Depthcam::detectChArUcoCorners(std::vector<cv::Point2f>& charucoCorners, std::vector<int>& charucoIds, bool Intrinsics)
{
	cv::Mat Intrin;
	cv::Vec<double, 5> Dist;
	cv::eigen2cv(IntrinsicMatrix, Intrin);
	cv::eigen2cv(DistortionCoeff, Dist);
	cv::Ptr<cv::aruco::DetectorParameters> params(new cv::aruco::DetectorParameters);
	params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
	cv::aruco::detectMarkers(color_mat, dictionary, corners, ids,params);
	if (ids.size() > 0) {
		if (Intrinsics) {
			std::vector<cv::Vec3d> corner_rvecs;
			cv::aruco::estimatePoseSingleMarkers(corners, markerLength, Intrin, Dist, corner_rvecs, corner_tvecs); //Estimate their pose for error assessment
			// Refine them
			cv::aruco::interpolateCornersCharuco(corners, ids, color_mat, board, charucoCorners, charucoIds, Intrin, Dist);
		}else
			cv::aruco::interpolateCornersCharuco(corners, ids, color_mat, board, charucoCorners, charucoIds);
		if (charucoIds.size() <=4) {
			return false;
		}
		return true;
	}
	return false;
}
bool Depthcam::estimateExtrinsics(std::vector<int> charucoIds, std::vector<cv::Point2f> charucoCorners) {
	cv::Vec3f rvec, tvec;
	cv::Mat Intrin;
	cv::Vec<double, 5> Dist;
	cv::eigen2cv(IntrinsicMatrix, Intrin);
	cv::eigen2cv(DistortionCoeff, Dist);
	bool valid= cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, Intrin, Dist, rvec, tvec);
	if(valid){
		cv::Mat RotationMatrix;
		cv::Rodrigues(rvec, RotationMatrix);
		Eigen::Matrix3f eigen_RotationMatrix;
		Eigen::Vector3f eigen_Translation;
		cv::cv2eigen(tvec, eigen_Translation);
		cv::cv2eigen(RotationMatrix, eigen_RotationMatrix);
		ExtrinsicMatrix = Eigen::Matrix4f::Identity();
		ExtrinsicMatrix.block(0, 0, 3, 3) = eigen_RotationMatrix;
		ExtrinsicMatrix.block(0, 3, 3, 1) = eigen_Translation;
		cv::aruco::drawAxis(color_mat, Intrin, Dist, rvec, tvec, 0.1);
		cv::aruco::drawDetectedCornersCharuco(color_mat, charucoCorners, charucoIds);
		//Error Assessment
		double mean_x = 0, mean_y = 0;
		std::vector<cv::Point2f> points;
		cv::projectPoints(board->chessboardCorners, rvec, tvec, Intrin, Dist, points);
		for(cv::Point2f point:points)
			cv::circle(color_mat, point, 5, cv::Scalar(0, 0, 255));
		int index = 0;
		for (cv::Point2f charucoCorner : charucoCorners) {
			mean_x += pow(charucoCorner.x - points.at(charucoIds.at(index)).x, 2);
			mean_y += pow(charucoCorner.y - points.at(charucoIds.at(index)).y, 2);
			index++;
		}
		double rms = sqrt(1.0 / index * (mean_x + mean_y));
		ReprojectionErrors.push_back(rms);
		DetectedMarkers.push_back(charucoIds.size());
		cout<< "For "<<getDeviceInfo() <<" the number of detect markers is "<<charucoIds.size()<<" and their reprojection error is " << rms<<" pixels.\n";
		if(debug>0)
			cv::imshow("[DEBUG] Calibration of "+getDeviceInfo(), color_mat);
		while (true&&debug==2) {
			char key = (char)cv::waitKey(100);
			if (key == 32)
				break;
		}
		return true;
	}
	else {
		return false;
	}
}
bool Depthcam::calibrateIntrinsics() {
	std::vector<std::vector<cv::Point2f>> Corners;
	std::vector<std::vector<int>> Ids;
	std::vector<cv::Vec3f> rvecs, tvecs;
	cout<<"Starting intrinsic calibration.\n";
	for (int i = 0; i < 10; i++) {
		rs2::frameset fs;
		cout<<"Find a new position of the charuco board. Press SPACE to continue.\n";
		while (true) {
			fs = pipe.wait_for_frames();
			color_mat = frame_to_mat(fs.get_color_frame()).clone();
			cv::imshow("Intrinsic: RGB", color_mat);
			char key = (char)cv::waitKey(100);
			if (key == 32)
				break;
		}
		double minval, maxval;
		cv::minMaxLoc(color_mat, &minval, &maxval, NULL, NULL);
		color_mat.convertTo(color_mat, CV_8UC1, 255.0 / maxval);
		std::vector<cv::Point2f> charucoCorners;
		std::vector<int> charucoIds;
		if (detectChArUcoCorners(charucoCorners, charucoIds,false)) {
			Corners.push_back(charucoCorners);
			Ids.push_back(charucoIds);
		}
		else {
			PCL_WARN("Position not accepted. Not enough charuco corners have been found. Try again.");
			i--;
			continue;
		}
	}
	cv::Mat intrin;
	cv::Vec<double, 5> dist;
	double rms = cv::aruco::calibrateCameraCharuco(Corners, Ids, board, color_mat.size(), intrin, dist);
	cv::cv2eigen(dist, DistortionCoeff);
	cv::cv2eigen(intrin, IntrinsicMatrix);
	cout << "The intrinsic calibration lead to an error of " <<rms<<"pixels.\n";
	return true;
}
bool Depthcam::calibrateExtrinsics() {
	rs2::frameset fs;
	fs = pipe.wait_for_frames();
	color_mat = frame_to_mat(fs.get_color_frame()).clone();
	std::vector<cv::Point2f> charucoCorners;
	std::vector<int> charucoIds;
	if (detectChArUcoCorners(charucoCorners, charucoIds,true)) {
		if (estimateExtrinsics(charucoIds, charucoCorners)) {
			return true; 
		}
	}
	return false; //skip this turntable position for this camera
}
void Depthcam::transformationError(std::vector<cv::Vec3d> master_tvecs,std::vector<int> master_ids,Eigen::Matrix4f& master_extrinsics) {
	int count = 0;
	int markersInBothPictures = 0;
	double mean_x = 0;
	double mean_y=0;
	double mean_z=0;
	PTC::Ptr cloud_master(new PTC);
	PTC::Ptr cloud_slave(new PTC);
	for (int master_id : master_ids) {
		Eigen::Vector4d corner = Eigen::Vector4d::Ones();
		Eigen::Vector3d tmp;
		cv::cv2eigen(master_tvecs.at(count), tmp);
		corner.head(3) = tmp;
		Eigen::Vector4f master_projected = master_extrinsics.inverse() * corner.cast<float>();
		Eigen::Vector4f slave_projected;
		count++;
		auto location = std::find(ids.begin(), ids.end(), master_id);
		if (location != ids.end()) {
			cv::cv2eigen(corner_tvecs.at(location - ids.begin()), tmp);
			corner.head(3) = tmp;
			slave_projected = ExtrinsicMatrix.inverse() * corner.cast<float>();
			mean_x += pow(slave_projected(0) - master_projected(0), 2);
			mean_y += pow(slave_projected(1) - master_projected(1), 2);
			mean_z += pow(slave_projected(2) - master_projected(2), 2);
			cloud_master->push_back(PT(master_projected(0), master_projected(1), master_projected(2)));
			cloud_slave->push_back(PT(slave_projected(0), slave_projected(1), slave_projected(2)));
			markersInBothPictures++;
		}
	}
	if (debug > 0) { Visual Visualizer("[DEBUG] Calibration Error of "+getDeviceInfo(), cloud_slave, cloud_master); if (debug == 2)	Visualizer.processOutput(); }
	double rms = sqrt(1.0 / markersInBothPictures *(mean_x + mean_y+mean_z));
	cout << "The transformation error of the markers of "<<getDeviceInfo()<< " to the master cam is " << rms << "m.\n";
}
