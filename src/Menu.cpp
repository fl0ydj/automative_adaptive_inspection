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
#include "Menu.h"
Menu::Menu() {
	parameters = std::map<std::string, std::string>({
		{ "debug", debug},
		{ "master_cam", master_cam},
		{"minLeafSize",minLeafSize},
		{ "preset", preset},
		{ "disparity_shift", disparity_shift},
		{ "laser_power", laser_power},
		{ "spat_alpha", spat_alpha},
		{ "spat_delta", spat_delta},
		{ "spat_magnitude", spat_magnitude},
		{ "temp_alpha", temp_alpha},
		{ "temp_delta", temp_delta},
		{ "temp_presistency", temp_presistency},
		{ "errorCorrection", errorCorrection},
		{ "postprocess_MeanK", postprocess_MeanK},
		{ "postprocess_thresh", postprocess_thresh},
		{ "leafPoints", leafPoints},
		{"normalsSearchRadius",normalsSearchRadius},
		{"FPFHSearchRadius",FPFHSearchRadius},
		{ "samplesInObjectSize",  samplesInObjectSize},
		{"maxCorrespondenceDistance_SAC",maxCorrespondenceDistance_SAC},
		{"numberOfIterations_SAC",numberOfIterations_SAC},
		{"maxCorrespondenceDistance_ICP", maxCorrespondenceDistance_ICP},
		{"maxFitnessScore_ICP", maxFitnessScore_ICP},
		{"numberOfIterations_ICP",  numberOfIterations_ICP},
		{"threshold_outlier", threshold_outlier},
		{"threshold_defect", threshold_defect}, 
		{"threshold_warning", threshold_warning},
		{"edge_radius", edge_radius},
		{"maxNeighbors", maxNeighbors},
		{"minInlierRatio", minInlierRatio},
		{"maxTolerance", maxTolerance},
		{"numberOfIterations_Tol",numberOfIterations_Tol},
		{"tolerance_search_radius", tolerance_search_radius},
		{"plane_normal_weight", plane_normal_weight},
		{"cyl_normal_weight", cyl_normal_weight},
		{"plane_dist_thresh", plane_dist_thresh},
		{"line_dist_thresh", line_dist_thresh},
		{"circle_dist_thresh", circle_dist_thresh},
		{"plane_epsDist", plane_epsDist},
		{"circle_epsCenter", circle_epsCenter},
		{"cyl_epsCenter", cyl_epsCenter},
		{"angleThres", angleThres},
		{"angular_angleThres", angular_angleThres},
	});
	io.loadParameters(parameters);
	displayIntro();
	reconnect();
}
void Menu::displayIntro() {
	std::cout << "\n+---------------------------------------------------------------------------+\n";
	std::cout << "|                                                                           |\n";
	std::cout << "|      Adaptive Analysis of Geometric Tolerances with Low-Cost Sensors      |\n";
	std::cout << "|                                                                           |\n";
	std::cout << "+---------------------------------------------------------------------------+\n";
	std::cout << "| Available commands:                                                       |\n";
	std::cout << "|                                                                           |\n";
	std::cout << "|     reconnect -> Restart the camera setup.                                |\n";
	std::cout << "|    intrinsics -> Perform intrinsic calibration using a CharUco board.     |\n";
	std::cout << "|    extrinsics -> Estimate the extrinsics using a CharUco Board.           |\n";
	std::cout << "|        record -> Start the recording process.                             |\n";
	std::cout << "|  registration -> Perform the registration of two clouds.                  |\n";
	std::cout << "|         merge -> Merge two clouds.                                        |\n";
	std::cout << "|     hausdorff -> Use the hausdorff distance to do the comparison.         |\n";
	std::cout << "|          view -> Visualize a point cloud file.                            |\n";
	std::cout << "|          snap -> Take a single shot and display the result.               |\n";
	std::cout << "|      settings -> Change the settings used in this project.                |\n";
	std::cout << "|          help -> Display this text again.                                 |\n";
	std::cout << "|          quit -> Quit the program.                                        |\n";
	std::cout << "|                                                                           |\n";
	std::cout << "+---------------------------------------------------------------------------+\n";
	std::cout << "Hint: To confirm PCL Visualizer windows or cv::imshow windows, press SPACE.\n";
}
void Menu::settings() {
	std::string response;
	std::cout << "\n---Current settings---\n";
	std::cout << "General:\n";
	std::cout << "-> debug:" << parameters.at("debug") << "\n";
	std::cout << "-> master_cam:" << parameters.at("master_cam") << "\n";
	std::cout << "-> minLeafSize:" << parameters.at("minLeafSize") << "\n";
	std::cout << "Image Recording:\n";
	std::cout << "-> preset:" << parameters.at("preset")<<"\n";
	std::cout << "-> disparityShift:" << parameters.at("disparity_shift")<< "\n";
	std::cout << "-> laser_power:" << parameters.at("laser_power") << "\n";
	std::cout << "-> spat_alpha:" << parameters.at("spat_alpha") << "\n";
	std::cout << "-> spat_delta:" << parameters.at("spat_delta") << "\n";
	std::cout << "-> spat_magnitude:" << parameters.at("spat_magnitude") << "\n";
	std::cout << "-> temp_alpha:" << parameters.at("temp_alpha") << "\n";
	std::cout << "-> temp_delta:" << parameters.at("temp_delta") << "\n";
	std::cout << "-> temp_presistency:" << parameters.at("temp_presistency") << "\n";
	std::cout << "-> errorCorrection:" << parameters.at("errorCorrection") << "\n";
	std::cout << "-> postprocess_MeanK:" << parameters.at("postprocess_MeanK") << "\n";
	std::cout << "-> postprocess_thresh:" << parameters.at("postprocess_thresh") << "\n";
	std::cout << "Registration:\n";
	std::cout << "-> leafPoints:" << parameters.at("leafPoints") << "\n";
	std::cout << "-> normalsSearchRadius:" << parameters.at("normalsSearchRadius") << "\n";
	std::cout << "-> FPFHSearchRadius:" << parameters.at("FPFHSearchRadius") << "\n";
	std::cout << "-> samplesInObjectSize:" << parameters.at("samplesInObjectSize") << "\n";
	std::cout << "-> maxCorrespondenceDistance_SAC:" << parameters.at("maxCorrespondenceDistance_SAC") << "\n";
	std::cout << "-> numberOfIterations_SAC:" << parameters.at("numberOfIterations_SAC") << "\n";
	std::cout << "-> maxCorrespondenceDistance_ICP:" << parameters.at("maxCorrespondenceDistance_ICP") << "\n";
	std::cout << "-> numberOfIterations_ICP:" << parameters.at("numberOfIterations_ICP") << "\n";
	std::cout << "-> maxFitnessScore_ICP:" << parameters.at("maxFitnessScore_ICP") << "\n";
	std::cout << "Comparison:\n";
	std::cout << "-> threshold_outlier:" << parameters.at("threshold_outlier") << "\n";
	std::cout << "-> threshold_defect:" << parameters.at("threshold_defect") << "\n";
	std::cout << "-> threshold_warning:" << parameters.at("threshold_warning") << "\n";
	std::cout << "-> edge_radius:" << parameters.at("edge_radius") << "\n";
	std::cout << "-> maxNeighbors:" << parameters.at("maxNeighbors") << "\n";
	std::cout << "-> minInlierRatio:" << parameters.at("minInlierRatio") << "\n";
	std::cout << "-> maxTolerance:" << parameters.at("maxTolerance") << "\n";
	std::cout << "-> numberOfIterations_Tol:" << parameters.at("numberOfIterations_Tol") << "\n";
	std::cout << "-> tolerance_search_radius:" << parameters.at("tolerance_search_radius") << "\n";
	std::cout << "-> angleThres:" << parameters.at("angleThres") << "\n";
	std::cout << "-> angular_angleThres:" << parameters.at("angular_angleThres") << "\n";
	std::cout << "-> plane_normal_weight:" << parameters.at("plane_normal_weight") << "\n";
	std::cout << "-> cyl_normal_weight:" << parameters.at("cyl_normal_weight") << "\n";
	std::cout << "-> plane_dist_thresh:" << parameters.at("plane_dist_thresh") << "\n";
	std::cout << "-> line_dist_thresh:" << parameters.at("line_dist_thresh") << "\n";
	std::cout << "-> circle_dist_thresh:" << parameters.at("circle_dist_thresh") << "\n";
	std::cout << "-> plane_epsDist:" << parameters.at("plane_epsDist") << "\n";
	std::cout << "-> circle_epsCenter:" << parameters.at("circle_epsCenter") << "\n";
	std::cout << "-> cyl_epsCenter:" << parameters.at("cyl_epsCenter") << "\n";
	std::cout << "\n\n";
	std::cout << "Type in the setting to change, \":\" and than the new value (e.g. Preset:xx.json). Type \":save\" to save (\":q\" to discard) the changes and to go back to the main menu.\n";
	std::string key, value;
	while (getline(cin, key, ':') && getline(cin, value)) {
		if (value == "save"|| value == "q") break;
		if (key.substr(0, 1) == "\n")key = key.substr(1);
		parameters.at(key) = value;
		std::cout << "\nParameter "<<key<<" changed to "<<value<<". Beware that no consistency checks are done. \n";
	}
	if (value == "save")
		io.saveParameters(parameters);
	std::cout << "\nGoing back to main menu...\n";
	io.loadParameters(parameters);
	reconnect();
}
void Menu::reconnect() {
	camcount = 0;
	Cams.clear();
	cout << "Connecting to the Intel Real Sense Cams...\n";
	rs2::context ctx;

	for (auto&& dev : ctx.query_devices())
	{
		Cams.emplace_back(Depthcam(ctx, dev, preset, stoi(parameters.at("disparity_shift")),stoi(parameters.at("laser_power")), parameters.at("debug")));
		if ((string)dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) == parameters.at("master_cam"));
		MasterCamIndex = camcount;
		camcount++;
	}
	cout << camcount << " Intel Real Sense Cameras found.\n";
}
void Menu::view() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	PTC::Ptr cloud2(new PTC);
	if (io.loadCloud<pcl::PointXYZRGB>("What is the path of the cloud to be visualized? (supported: .stl,.pcd,.ply)\n", cloud,"",true)) {
		if (io.loadCloud<PT>("What is the path of the second cloud/tolerance definition (.jt)/results file (.txt)? (type \"default\" to load no second cloud)\n", cloud2, "q", true)) {
			PTC::Ptr cloud1(new PTC);
			pcl::copyPointCloud(*cloud, *cloud1);
			if (cloud2->width != 0) {
				Visual Visualizer("CloudViewer", cloud1, cloud2);
				Visualizer.processOutput();
				Visualizer.closeViewer();
			}
			else {
				Comparison Comp(parameters.at("debug"));
				Comp.visualizeTolerances(cloud1, io.tolerances);
			}
		}
		else {
			Visual Visualizer("CloudViewer", cloud);
			Visualizer.processOutput();
			Visualizer.closeViewer();
		}
	}
}
void Menu::snap() {
	for (Depthcam& Cam : Cams) {
		io.loadMatrix<float, 4, 4>("FromMasterCam_" + Cam.getDeviceInfo() + ".txt", Cam.FromMasterCam);
		Eigen::Matrix4f tmp(Eigen::Matrix4f::Identity());
		Cam.calibrateStatic(tmp);
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr VisCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (Depthcam& Cam : Cams) {
		Cam.createPCLPointCloud(stod(parameters.at("spat_alpha")), stod(parameters.at("spat_delta")), stod(parameters.at("spat_magnitude")), stod(parameters.at("temp_alpha")), stod(parameters.at("temp_delta")), stoi(parameters.at("temp_presistency")));
		Registration Reg(parameters.at("debug"));
		Reg.postprocess<pcl::PointXYZRGB>(Cam.cloudRGB,stoi(parameters.at("postprocess_MeanK")), stod(parameters.at("postprocess_thresh")));
		VisCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>(*VisCloud + *Cam.cloudRGB));
	}
	io.saveCloud<pcl::PointXYZRGB>("snap.pcd", VisCloud);
	Visual Visualizer("Snap Cloud", VisCloud);
	Visualizer.processOutput();
	Visualizer.closeViewer();
}
void Menu::intrinsics() {
	if (camcount == 0) {
		PCL_ERROR("No Intel Real Sense has been found.");
		return;
	}
	for (Depthcam& Cam : Cams) {
		std::cout<<"Calibration is started for "<<Cam.getDeviceInfo()<<"...\n";
		Cam.calibrateIntrinsics();
		io.saveMatrix<double, 3, 3>("IntrinsicMatrix_" + Cam.getDeviceInfo() + ".txt", Cam.IntrinsicMatrix);
		io.saveMatrix<double, 5, 1>("DistortionCoeffs_" + Cam.getDeviceInfo() + ".txt", Cam.DistortionCoeff);
	}
}
void Menu::extrinsics() {
	if (camcount == 0) {
		PCL_ERROR("No Intel Real Sense has been found.");
		return;
	}
	PCL_INFO("Calibration is started.\n");
	time_t tstart, tend;
	tstart = time(0);
	bool calibrationFault = true;
	while (calibrationFault) {
		calibrationFault = false;
		for (Depthcam& Cam : Cams) {
			if (!Cam.calibrateExtrinsics()) {
				std::cout << "Calibration could not be performed. Try changing the position of the charuco board.\n";
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
				calibrationFault = true;
				break;
			}//tries to find CharUco board which can be seen by all cameras
		}
	}
	if (stoi(parameters.at("errorCorrection"))) {
		for (Depthcam& Cam : Cams) {
			Cam.createPCLPointCloud(stod(parameters.at("spat_alpha")), stod(parameters.at("spat_delta")), stod(parameters.at("spat_magnitude")), stod(parameters.at("temp_alpha")), stod(parameters.at("temp_delta")), stoi(parameters.at("temp_presistency")));
			Cam.removeBackground(Cam.cloud, false);
			if (Cams[MasterCamIndex].getDeviceInfo() == Cam.getDeviceInfo()) {
				Cam.background = Cam.cloud;
				if (Cam.cloud->size() != 0)
					io.saveCloud<PT>("background_" + Cam.getDeviceInfo() + ".pcd",Cam.cloud);
			}
			else {
				cout << "Before error correction: ";
				Cam.transformationError(Cams[MasterCamIndex].corner_tvecs, Cams[MasterCamIndex].ids, Cams[MasterCamIndex].ExtrinsicMatrix); //calibration error before error correction
			}
		}
	}
	std::vector<cv::Mat> slaveMats;
	std::vector<Eigen::Matrix4f> fromMasterCams;
	for (Depthcam& Cam : Cams) {
		if (Cams[MasterCamIndex].getDeviceInfo() != Cam.getDeviceInfo() && stoi(parameters.at("errorCorrection"))) {
			Registration Reg(parameters.at("debug"));
			//ERROR CORRECTION
			pcl::PointCloud<pcl::Normal>::Ptr slave_normals = Reg.getNormals(Cam.cloud, stod(parameters.at("minLeafSize")) * stod(parameters.at("normalsSearchRadius")));
			pcl::PointCloud<pcl::Normal>::Ptr master_normals = Reg.getNormals(Cams[MasterCamIndex].background, stod(parameters.at("minLeafSize")) * stod(parameters.at("normalsSearchRadius")));
			Eigen::Matrix4f error;
			double score;
			if (!Reg.cloudRegistration(Cam.cloud, slave_normals, Cams[MasterCamIndex].background, master_normals, score, error, 0.005, 3))
				PCL_ERROR("Calibration erronous. Do both camera record a pointcloud in this position? Consider doing the Calibration again. Maybe change the position of the charuco board. \n");
			else {
				cout << "The Error correction using ICP lead to a fitness score of " << score << "\n";
				Eigen::Matrix4f errorInv = error.inverse();
				Cam.refineCalibration(errorInv);
			}
		}
		Cam.determineFromMasterCam(Cams[MasterCamIndex].ExtrinsicMatrix);
		io.saveMatrix<float, 4, 4>("FromMasterCam_" + Cam.getDeviceInfo() + ".txt", Cam.FromMasterCam);
		if (Cams[MasterCamIndex].getDeviceInfo() != Cam.getDeviceInfo()) {
			if (stoi(parameters.at("errorCorrection")))
				cout << "After error correction: ";
			Cam.transformationError(Cams[MasterCamIndex].corner_tvecs, Cams[MasterCamIndex].ids, Cams[MasterCamIndex].ExtrinsicMatrix); //calibration fault after error correction
		}
	}
	tend = time(0);
	cout << "Calibration is finished. It took " << difftime(tend, tstart) << " seconds.\n";
}
void Menu::record() {
	//Arduino turntable("\\\\.\\COM5"); //UNCOMMENT THIS, IF TURNTABLE IS USED
	PTC::Ptr camCloud(new PTC);
	if (camcount == 0) {
		PCL_ERROR("No Intel Real Sense has been found.");
		return;
	}
	int NoS, degrees;
	std::cout << "How many scans should be made?\n";
	std::cin >> NoS;
	std::cout << "How many degrees should the turntable be rotated after each scan?\n";
	std::cin >> degrees;
	time_t tstart, tend,recStart,recEnd;
	recStart = time(0);
	tstart = time(0);
	for (Depthcam& Cam : Cams) {
		if (Cams[MasterCamIndex].getDeviceInfo() == Cam.getDeviceInfo() && stoi(parameters.at("errorCorrection")))
			io.loadCloud<PT>("background_" + Cam.getDeviceInfo() + ".pcd", Cam.background);
		io.loadMatrix<float,4,4>("FromMasterCam_" + Cam.getDeviceInfo() + ".txt", Cam.FromMasterCam);
		Cam.ReprojectionErrors.clear();
		Cam.DetectedMarkers.clear();
	}
	int i = 0;
	for (i = 0; i < NoS; i++) {
		tend = time(0);
		std::this_thread::sleep_for(std::chrono::milliseconds(5000));
		// INITIAL CALIBRATION OF MAIN CAM
		while (!Cams[MasterCamIndex].calibrateExtrinsics()) {
			std::cout << "Calibration could not be performed. The turntable is rotated.\n";
			//turntable.rotatePlateWithoutFeedback(10); //UNCOMMENT THIS, IF TURNTABLE IS USED
			std::this_thread::sleep_for(std::chrono::milliseconds(5000));
		}
		// Error Correction OF Master CAM
		if (stoi(parameters.at("errorCorrection"))) {
			PTC::Ptr cloud(new PTC);
			PTC::Ptr cloud_sparse(new PTC);
			PTC::Ptr background(new PTC);
			Cams[MasterCamIndex].createPCLPointCloud(stod(parameters.at("spat_alpha")), stod(parameters.at("spat_delta")), stod(parameters.at("spat_magnitude")), stod(parameters.at("temp_alpha")), stod(parameters.at("temp_delta")), stoi(parameters.at("temp_presistency")));
			Cams[MasterCamIndex].removeBackground(Cams[MasterCamIndex].cloud, false);
			Registration Reg(parameters.at("debug"));
			pcl::PointCloud<pcl::Normal>::Ptr old_normals = Reg.getNormals(background, stod(parameters.at("minLeafSize")) * stod(parameters.at("normalsSearchRadius")));
			pcl::PointCloud<pcl::Normal>::Ptr master_normals = Reg.getNormals(Cams[MasterCamIndex].cloud, stod(parameters.at("minLeafSize")) * stod(parameters.at("normalsSearchRadius")));
			Eigen::Matrix4f error;
			double score;
			if (!Reg.cloudRegistration(Cams[MasterCamIndex].cloud, master_normals,background, old_normals, score, error, 0.005, 3)) {
				std::cout << "PointCloud was errornous. Turntable is rotated.\n";
				//turntable.rotatePlateWithoutFeedback(10); //UNCOMMENT THIS, IF TURNTABLE IS USED
				tstart = time(0);
				i--;
				continue;
			}
			else {
				cout << "The Error correction using ICP lead to a fitness score of " << score << "\n";
				Eigen::Matrix4f errorInv = error.inverse();
				Cams[MasterCamIndex].refineCalibration(errorInv);
			}
		}
		// POINT CLOUD RECORDING & SAVING
		for (Depthcam& Cam : Cams) {
			if (Cams[MasterCamIndex].getDeviceInfo() != Cam.getDeviceInfo())
				Cam.calibrateStatic(Cams[MasterCamIndex].ExtrinsicMatrix);
			Cam.createPCLPointCloud(stod(parameters.at("spat_alpha")), stod(parameters.at("spat_delta")), stod(parameters.at("spat_magnitude")), stod(parameters.at("temp_alpha")), stod(parameters.at("temp_delta")), stoi(parameters.at("temp_presistency")));
		}
		// ROTATE TURNTABLE
		//turntable.rotatePlateWithoutFeedback(degrees); //UNCOMMENT THIS, IF TURNTABLE IS USED
		cout << i+1 << "/" << NoS << "\n";
		system("PAUSE");
		tstart = time(0);
 		for (Depthcam& Cam : Cams) {
			Cam.removeBackground(Cam.cloud,true);
			camCloud.reset(new PTC(*camCloud + *Cam.cloud));
		}
	}
	Visual Visualizer("MeasuredCloud", camCloud);
	Registration Reg(parameters.at("debug"));
	Reg.postprocess<PT>(camCloud, stoi(parameters.at("postprocess_MeanK")), stod(parameters.at("postprocess_thresh")));
	double leafSize = stod(parameters.at("minLeafSize"));
	Reg.downsample<PT>(camCloud, camCloud, leafSize);
	recEnd = time(0);
	cout << "Recording finished. It took " << difftime(recEnd, recStart) << " seconds.\n";
	double avgReprojection = std::accumulate(Cams[MasterCamIndex].ReprojectionErrors.begin(), Cams[MasterCamIndex].ReprojectionErrors.end(), 0.0) / (double)Cams[MasterCamIndex].ReprojectionErrors.size();
	double avgMarkers = std::accumulate(Cams[MasterCamIndex].DetectedMarkers.begin(), Cams[MasterCamIndex].DetectedMarkers.end(), 0) / (double)Cams[MasterCamIndex].DetectedMarkers.size();
	cout << "For " << Cams[MasterCamIndex].getDeviceInfo() << " an average of " << avgMarkers << " markers were detected and led to a reprojection error of " << avgReprojection << ".\n";
	Visualizer.updateCloud(camCloud);
	Visualizer.processOutput();
	io.saveCloud<PT>("MeasuredCloud.pcd", camCloud);
}
void Menu::registration() {
	PTC::Ptr cam(new PTC);
	PTC::Ptr comp(new PTC);
	PTC::Ptr comp_sparse(new PTC);
	PTC::Ptr cam_sparse(new PTC);
	Registration Reg(parameters.at("debug"));
	if (io.loadCloud<PT>("What is the path of the component file? (type \"default\" for: testobjekt-fixed.ply)\n", comp, "testobjekt-fixed.ply",true) &&
		io.loadCloud<PT>("What is the path of the measured point cloud? (type \"default\" for: MeasuredCloud.pcd)\n", cam, "MeasuredCloud.pcd",true)) {
		Visual Visualizer("Registration", cam, comp);
		if (stoi(parameters.at("debug")) >0) {
			cout << "[DEBUG] Clouds before Registration\n";  Visualizer.processOutput(); }
		time_t tstart, tend,regStart,regEnd;
		regStart = time(0);
		Reg.demean(cam, cam);
		Reg.demean(comp, cam, true);
		PT max, min;
		pcl::getMinMax3D(*comp, min, max);
		double objSize=pcl::euclideanDistance(min, max);
		double leafSize= std::max(stod(parameters.at("minLeafSize")), objSize / stod(parameters.at("leafPoints")));
		Reg.downsample<PT>(cam, cam_sparse, leafSize,100000);
		Reg.downsample<PT>(comp, comp_sparse, leafSize,100000);
		Eigen::Matrix4f transformation;
		double score;
		PCL_INFO("Registration is started... This may take up to 10 min.\n");
		pcl::PointCloud<pcl::Normal>::Ptr cam_normals = Reg.getNormals(cam_sparse, leafSize*stod(parameters.at("normalsSearchRadius")));
		pcl::PointCloud<pcl::Normal>::Ptr comp_normals = Reg.getNormals(comp_sparse, leafSize* stod(parameters.at("normalsSearchRadius")));
		tstart = time(0);
		if (Reg.findInitialGuess(cam_sparse, cam_normals,comp_sparse,comp_normals, score,transformation, objSize/stod(parameters.at("samplesInObjectSize")), pow(stod(maxCorrespondenceDistance_SAC),2), stoi(parameters.at("numberOfIterations_SAC")), leafSize*stod(parameters.at("FPFHSearchRadius")))) {
			pcl::transformPointCloud(*cam, *cam, transformation);
			tend = time(0);
			cout << "The Initial Registration using Sample Consensus Initial Alignment took " << difftime(tend, tstart) << " seconds and lead to a fitness score of: " << score << ".\n";
			if (stoi(parameters.at("debug")) ==2) { Visualizer.updateCloud(cam); Visualizer.processOutput(); }
			tstart = time(0);
			if (Reg.cloudRegistration(cam_sparse, cam_normals, comp_sparse, comp_normals, score,transformation, pow(stod(parameters.at("maxCorrespondenceDistance_ICP")),2), stoi(parameters.at("numberOfIterations_ICP")), stod(parameters.at("maxFitnessScore_ICP")))) {
				pcl::transformPointCloud(*cam, *cam, transformation);
				tend = time(0);
				cout << "The Fine registration using ICP took "<< difftime(tend,tstart)<<" seconds and lead to a fitness score of " << score << ".\n";
				cout<<"Please ensure, that the clouds are aligned properly.\n";
			}
		}
		else {
			PCL_WARN("Registration failed. Check alignment manually! Consider doing a new recording.");
		}
		regEnd = time(0);
		cout << "Registration finished. The whole registration took "<< difftime(regEnd, regStart) << "seconds.\n";
		Visualizer.updateCloud(cam,comp);
		Visualizer.processOutput();
		pcl::io::savePCDFileASCII("lol.pcd", *cam);
		io.saveCloud<PT>("MeasuredCloudAfterRegistration.pcd", cam);
	}
}
void Menu::merge() {
	PTC::Ptr cam1(new PTC);
	PTC::Ptr cam2(new PTC);
	if (io.loadCloud<PT>("What is the path of the first cloud?\n", cam1) &&
		io.loadCloud<PT>("What is the path of the second cloud?\n", cam2)) {
		PTC::Ptr merged(new PTC(*cam1 + *cam2));
		io.saveCloud<PT>("Merged.pcd", merged);
	}
}
void Menu::hausdorff() {
	Comparison Compare(parameters.at("debug"));
	PTC::Ptr comp(new PTC);
	PTC::Ptr cam(new PTC);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr comp_hausdorff(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (io.loadCloud<PT>("What is the path of the component file? (type \"default\" for: testobjekt-fixed.ply)\n", comp, "testobjekt-fixed.ply",true) &&
		io.loadCloud<PT>("What is the path of the measured point cloud? type \"default\" for: MeasuredCloudAfterRegistration.pcd)\n", cam, "MeasuredCloudAfterRegistration.pcd")) {
		Registration Reg(parameters.at("debug"));
		double leafSize = stod(parameters.at("minLeafSize"));
		Reg.downsample<PT>(comp, comp, leafSize);
		Reg.downsample<PT>(cam, cam, leafSize);
		cout << "Starting Hausdorff comparison...\n";
		time_t tstart, tend;
		tstart = time(0);
		float hausdorff;
		float avg_dist;
		std::vector<int> defects;
		std::vector<double> thresh = { stod(parameters.at("threshold_outlier")),stod(parameters.at("threshold_defect")) ,stod(parameters.at("threshold_warning"))};
		if (Compare.hausdorff(cam, comp,thresh, comp_hausdorff,defects, hausdorff, avg_dist,true)) {
			cout << "The component contains defects with a deviation of greater than " << parameters.at("threshold_defect") << "m!";
			cout << "There are " << defects.size() << " defective points from a total of " << comp_hausdorff->width << " points.";
		}
		else {
			cout << "All deviations (except the detected outliers) are within " << parameters.at("threshold_defect") << "m. However, check the orange marked points, which show deviations of more than "<< parameters.at("threshold_warning") <<"m\n";
		}
		tend = time(0);
		cout << "Hausdorff Comparison finished. It took "<<difftime(tend,tstart) <<" seconds.\n";
		cout << "The hausdorff distance from cam to comp is: " << hausdorff << "\n";
		cout << "The average distance from cam to comp is: " << avg_dist << "\n";
		Visual clouds("Component and Measured Cloud", cam,comp);
		Visual colored("Hausdorff cloud", comp_hausdorff);
		io.saveCloud<pcl::PointXYZRGB>("HausdorffCloud.pcd", comp_hausdorff);
		io.saveCloud<PT>("MeasuredCloudAfterHausdorff.pcd", cam);
		clouds.processOutput();
	}
}
void Menu::tolerances() {
	PTC::Ptr cam(new PTC);
	if (io.loadTolerances()&&io.loadCloud<PT>("What is the path of the measured point cloud? (type \"default\" for: MeasuredCloudAfterHausdorff.pcd)\n", cam, "MeasuredCloudAfterHausdorff.pcd")) {
		time_t tstart, tend;
		tstart = time(0);
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		Registration Reg(parameters.at("debug"));
		double leafSize = stod(parameters.at("minLeafSize"));
		Reg.downsample<PT>(cam, cam, leafSize);
		Comparison Compare(parameters.at("debug"));
		Compare.checkTolerances(cam,io.tolerances, stod(parameters.at("edge_radius")), stoi(parameters.at("maxNeighbors")),stod(parameters.at("minLeafSize")), stod(parameters.at("minInlierRatio")),stod(parameters.at("maxTolerance")),stoi(parameters.at("numberOfIterations_Tol")), stod(parameters.at("tolerance_search_radius")), stod(parameters.at("plane_normal_weight")), stod(parameters.at("cyl_normal_weight")), stod(parameters.at("plane_dist_thresh")), stod(parameters.at("line_dist_thresh")), stod(parameters.at("circle_dist_thresh")), stod(parameters.at("plane_epsDist")), stod(parameters.at("circle_epsCenter")), stod(parameters.at("cyl_epsCenter")), stod(parameters.at("angleThres")), stod(parameters.at("angular_angleThres")));
		tend = time(0);
		cout << "Tolerance Comparison finished. It took " << difftime(tend, tstart) << " seconds.\n";
		Compare.visualizeTolerances(cam, io.tolerances);
		io.saveResults();
	}
	io.tolerances.clear();
}