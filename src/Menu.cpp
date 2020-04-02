#include "Menu.h"
Menu::Menu() {
	parameters = std::map<std::string, std::string>({
		{ "debug", debug},
		{ "preset", preset},
		{ "disparity_shift", disparity_shift},
		{ "leafPoints", leafPoints},
		{"normalsSearchRadius",normalsSearchRadius},
		{"FPFHSearchRadius",FPFHSearchRadius},
		{ "samplesInObject",  samplesInObject},
		{"maxCorrespondenceDistance_SAC",maxCorrespondenceDistance_SAC},
		{"numberOfIterations_SAC",numberOfIterations_SAC},
		{"maxCorrespondenceDistance_ICP", maxCorrespondenceDistance_ICP},
		{"numberOfIterations_ICP",  numberOfIterations_ICP},
	});
	io.loadParameters(parameters);
	displayIntro();
	reconnect();
}
void Menu::displayIntro() {
	std::cout << "\n+---------------------------------------------------------------------------+\n";
	std::cout << "|                                                                           |\n";
	std::cout << "|                         Tolerance Checking                                |\n";
	std::cout << "|                                                                           |\n";
	std::cout << "+---------------------------------------------------------------------------+\n";
	std::cout << "| Available commands:                                                       |\n";
	std::cout << "|                                                                           |\n";
	std::cout << "|     reconnect -> Restart the camera setup.                                |\n";
	std::cout << "|    intrinsics -> Perform intrinsic calibration using a CharUco board.     |\n";
	std::cout << "|    extrinsics -> Estimate the extrinsics using a CharUco Board.           |\n";
	std::cout << "|        record -> Start the recording process.                             |\n";
	std::cout << "|  registration -> Perform the registration of two clouds.                  |\n";
	std::cout << "|     hausdorff -> Use the hausdorff distance to do the comparison.         |\n";
	std::cout << "|      features -> Detect features and parametrizes them.                   |\n";
	std::cout << "|          view -> Visualize a point cloud file.                            |\n";
	std::cout << "|          snap -> Take a single shot and display the result.               |\n";
	std::cout << "|      settings -> Change the settings used in this project.                |\n";
	std::cout << "|          help -> Display this text again.                                 |\n";
	std::cout << "|          quit -> Quit the program.                                        |\n";
	std::cout << "|                                                                           |\n";
	std::cout << "+---------------------------------------------------------------------------+\n";
}
void Menu::settings() {
	std::string response;
	std::cout << "\n---Current settings---\n";
	std::cout << "General:\n";
	std::cout << "-> debug:" << parameters.at("debug") << "\n";
	std::cout << "Image Recording:\n";
	std::cout << "-> Preset:" << parameters.at("Preset")<<"\n";
	std::cout << "-> DisparityShift:" << parameters.at("disparity_shift")<< "\n";
	std::cout << "Registration:\n";
	std::cout << "-> leafPoints:" << parameters.at("leafPoints") << "\n";
	std::cout << "-> normalsSearchRadius:" << parameters.at("normalsSearchRadius") << "\n";
	std::cout << "-> FPFHSearchRadius:" << parameters.at("FPFHSearchRadius") << "\n";
	std::cout << "-> samplesInObject:" << parameters.at("samplesInObject") << "\n";
	std::cout << "-> maxCorrespondenceDistance_SAC:" << parameters.at("maxCorrespondenceDistance_SAC") << "\n";
	std::cout << "-> numberOfIterations_SAC:" << parameters.at("numberOfIterations_SAC") << "\n";
	std::cout << "-> maxCorrespondenceDistance_ICP:" << parameters.at("maxCorrespondenceDistance_ICP") << "\n";
	std::cout << "-> numberOfIterations_ICP:" << parameters.at("numberOfIterations_ICP") << "\n";
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
		Cams.emplace_back(Depthcam(ctx, dev, preset, stoi(parameters.at("disparity_shift")), parameters.at("debug")));
		if ((string)dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) == "934222070559");// "939622072326")//")//"817612071411")
		MasterCamIndex = camcount;
		camcount++;
	}
	cout << camcount << " Intel Real Sense Cameras found.\n";
}
void Menu::view() {
	PTC::Ptr cloud(new PTC);
	PTC::Ptr cloud2(new PTC);
	if (io.loadCloud("What is the path of the cloud to be visualized? (supported: .stl,.pcd,.ply)\n", cloud)) {
		Visual Visualizer;
		if (io.loadCloud("What is the path of the second cloud? (type \"default\" to load no second cloud)\n", cloud2, "q",true)) 
			Visualizer = Visual("CloudViewer", cloud,cloud2);
		else
			Visualizer= Visual("CloudViewer", cloud);
		Visualizer.processOutput();
		Visualizer.closeViewer();
	}
}
void Menu::snap() {
	for (Depthcam& Cam : Cams) {
		Cam.loadFromMasterCam();
		Eigen::Matrix4f tmp(Eigen::Matrix4f::Identity());
		Cam.calibrateStatic(tmp, Cams[MasterCamIndex].ColorToDepth);
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr VisCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (Depthcam& Cam : Cams) {
		Cam.createPCLPointCloud();
		VisCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>(*VisCloud + *Cam.cloudRGB));
	}
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
		if(Cams[MasterCamIndex].getDeviceInfo()!=Cam.getDeviceInfo())
			Cam.calibrateIntrinsics();
	}
}
void Menu::extrinsics() {
	if (camcount == 0) {
		PCL_ERROR("No Intel Real Sense has been found.");
		return;
	}
	PCL_INFO("Calibration is started.\n");
	bool calibrationFault = true;
	while (calibrationFault) {
		calibrationFault = false;
		for (Depthcam& Cam : Cams) {
			//Cam.loadIntrinsics();
			if (!Cam.calibrateExtrinsics()) {
				std::cout << "Calibration could not be performed. Try changing the position of the charuco board.\n";
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
				calibrationFault = true;
				break;
			}//tries to find CharUco board which can be seen by all cameras
		}
	}
	for (Depthcam& Cam : Cams) {
		Cam.createPCLPointCloud();
		Cam.removeBackground(Cam.cloud,false);
		if (Cams[MasterCamIndex].getDeviceInfo() == Cam.getDeviceInfo())
			Cam.saveBackground();
		else {
			//How is it before error correction
			Cam.calibrationError(Cams[MasterCamIndex].corner_tvecs, Cams[MasterCamIndex].ids, Cams[MasterCamIndex].ExtrinsicMatrix);
		}
	}
	std::vector<cv::Mat> slaveMats;
	std::vector<Eigen::Matrix4f> fromMasterCams;
	for (Depthcam& Cam : Cams) {
		//if (Cams[MasterCamIndex].getDeviceInfo() != Cam.getDeviceInfo()) {
		//	Registration Reg(parameters.at("debug"));
		//	PTC::Ptr cloud_sparse(new PTC);
		//	PTC::Ptr background(new PTC);
		//	//ERROR CORRECTION
		//	Reg.downsample(Cam.cloud, cloud_sparse, 0.001);
		//	Reg.downsample(Cams[MasterCamIndex].background, background, 0.001);
		//	Eigen::Matrix4f error;
		//	double score;
		//	if (!Reg.cloudRegistration(background,cloud_sparse,score, error, 0.002, 1))
		//		PCL_ERROR("Calibration erronous. Do both camera record a pointcloud in this position? Consider doing the Calibration again. Maybe change the position of the charuco board. \n");
		//	else
		//		pcl::transformPointCloud(*Cam.cloud, *Cam.cloud, error.inverse());
		//	cout << "The Error correction using ICP lead to a fitness score of " << score << "\n";
		//	Cam.refineCalibration(error);
		//}
		Cam.determineFromMasterCam(Cams[MasterCamIndex].ExtrinsicMatrix, Cams[MasterCamIndex].ColorToDepth);
		if (Cams[MasterCamIndex].getDeviceInfo() != Cam.getDeviceInfo()) {
			//After
			Cam.calibrationError(Cams[MasterCamIndex].corner_tvecs, Cams[MasterCamIndex].ids,Cams[MasterCamIndex].ExtrinsicMatrix);
		}
	}
	PCL_INFO("Calibration is finished.\n");
	//		Comp.Visualizer.processOutput();
}
void Menu::record() {
	//Arduino turntable("\\\\.\\COM5");
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
	time_t tstart, tend;
	tstart = time(0);
	for (Depthcam& Cam : Cams) {
		if (Cams[MasterCamIndex].getDeviceInfo() == Cam.getDeviceInfo())
			Cam.loadBackground();
		Cam.loadFromMasterCam();
	}
	int i = 0;
	for (i = 0; i < NoS; i++) {
		tend = time(0);
		std::this_thread::sleep_for(std::chrono::milliseconds(5000 - (int)difftime(tend, tstart) * 1000));
		// INITIAL CALIBRATION OF MAIN CAM
		while (!Cams[MasterCamIndex].calibrateExtrinsics()) {
			std::cout << "Calibration could not be performed. The turntable is rotated.\n";
			//turntable.rotatePlateWithoutFeedback(10);
			std::this_thread::sleep_for(std::chrono::milliseconds(5000));
		}
		// FINE CALIBRATION OF MAIN CAM
		/*PTC::Ptr cloud(new PTC);
		PTC::Ptr cloud_sparse(new PTC);
		PTC::Ptr background(new PTC);
		Cams[MasterCamIndex].createPCLPointCloud();
		Cams[MasterCamIndex].getCloud(cloud);
		Cams[MasterCamIndex].removeBackground(cloud,false);
		Reg.downsample(cloud, cloud_sparse, 0.001);
		Reg.downsample(Cams[MasterCamIndex].background, background, 0.001);
		Eigen::Matrix4f error;
		if(!Reg.cloudRegistration(background,cloud_sparse, error, 0.002, 1)) {
			std::cout << "PointCloud was errornous. Turntable is rotated.\n";
			turntable.rotatePlateWithoutFeedback(10);
			tstart = time(0);
			i--;
			continue;
		}
		Cams[MasterCamIndex].refineCalibration(error);*/
		// POINT CLOUD RECORDING & SAVING
		for (Depthcam& Cam : Cams) { //manchmal riesen offset zwischen scans; edge uncertainty -> edges fehlen; weitere features
			if (Cams[MasterCamIndex].getDeviceInfo() != Cam.getDeviceInfo())
				Cam.calibrateStatic(Cams[MasterCamIndex].ExtrinsicMatrix, Cams[MasterCamIndex].ColorToDepth);
			Cam.createPCLPointCloud();
		}
		// ROTATE TURNTABLE
		//turntable.rotatePlateWithoutFeedback(degrees);
		tstart = time(0);
		// REGISTRATION PIPELINE
 		for (Depthcam& Cam : Cams) {
			Cam.removeBackground(Cam.cloud,true);
			//Cam.postprocess(Cam.cloud);
			camCloud.reset(new PTC(*camCloud + *Cam.cloud));
#ifdef DEBUG
			//Reg.Visualizer.updateCloud(Cam.match_cloud_with_background);
			/*Reg.Visualizer.updateCloud(Reg.cam);
			Reg.Visualizer.processOutput();*/
			//Reg.Visualizer.updateCloud(Reg.cam_sparse);
#endif
		}
	}
	Registration Reg(parameters.at("debug"));
	double leafSize = 0.0005;
	Reg.downsample(camCloud, camCloud, leafSize);
	Visual Visualizer("MeasuredCloud", camCloud);
	Visualizer.processOutput();
	PCL_INFO("Recording finished. Saving point cloud MeasuredCloud.pcd...\n");
	pcl::io::savePCDFileASCII("MeasuredCloud.pcd", *camCloud);
	for (Depthcam& Cam : Cams) {
		cout << "Reprojection Errors: " << Cam.getDeviceInfo() << "\n";
		for (double error : Cam.ReprojectionErrors)
			cout << error << "\n";
		cout << "\n";
	}
}
void Menu::registration() {
	PTC::Ptr cam(new PTC);
	PTC::Ptr comp(new PTC);
	PTC::Ptr comp_sparse(new PTC);
	PTC::Ptr cam_sparse(new PTC);
	Registration Reg(parameters.at("debug"));// Comp.cloud_sparse, Cams[MasterCamIndex].scale);
	if (io.loadCloud("What is the path of the component file? (type \"default\" for: testobjekt-fixed.ply)\n", comp, "testobjekt-fixed.ply",true) &&
		io.loadCloud("What is the path of the measured point cloud? (type \"default\" for: MeasuredCloud.pcd)\n", cam, "MeasuredCloud.pcd")) {
		/*Cams[MasterCamIndex].postprocess(cam);
		Visual Visualizer3("MeasuredCloud", cam);
		Visualizer3.processOutput();
		PCL_INFO("[RECORD] Recording finished. Saving point cloud MeasuredCloud.pcd...\n");
		pcl::io::savePCDFileASCII("MeasuredCloud.pcd", *cam);*/
		Reg.demean(cam,cam);
		Reg.demean(comp, cam,true);
		Visual Visualizer("Registration", cam, comp);
		if (stoi(parameters.at("debug")) > 0) {
			cout << "[DEBUG] ";  Visualizer.processOutput(); }
		PT max, min;
		pcl::getMinMax3D(*comp, min, max);
		double objSize=pcl::euclideanDistance(min, max);
		double leafSize= std::max(0.0005, objSize / stod(parameters.at("leafPoints"))); //-> 0.5mm ist Unsicherheit, daher ist die minimale leaf size
		Reg.downsample(cam, cam_sparse, leafSize,100000);
		Reg.downsample(comp, comp_sparse, leafSize,100000);
		Eigen::Matrix4f transformation;
		double score;
		PCL_INFO("Registration is started... This may take up to 10 min.\n");
		pcl::PointCloud<pcl::Normal>::Ptr cam_normals = Reg.getNormals(cam_sparse, leafSize*stod(parameters.at("normalsSearchRadius")));
		pcl::PointCloud<pcl::Normal>::Ptr comp_normals = Reg.getNormals(comp_sparse, leafSize* stod(parameters.at("normalsSearchRadius")));
		if (Reg.findInitialGuess(cam_sparse, cam_normals,comp_sparse,comp_normals, score,transformation, objSize/stod(parameters.at("samplesInObject")), stod(maxCorrespondenceDistance_SAC), stoi(parameters.at("numberOfIterations_SAC")), leafSize*stod(parameters.at("FPFHSearchRadius")))) {
			pcl::transformPointCloud(*cam, *cam, transformation);
			cout << "The Initial Registration using Sample Consensus Initial Alignment lead to a fitness score of: " << score << ".\n";
			if (stoi(parameters.at("debug")) ==2) { Visualizer.updateCloud(cam); Visualizer.processOutput(); }
			if (Reg.cloudRegistration(cam_sparse, cam_normals, comp_sparse, comp_normals, score,transformation, stod(parameters.at("maxCorrespondenceDistance_ICP")), stoi(parameters.at("numberOfIterations_ICP")))) {
				pcl::transformPointCloud(*cam, *cam, transformation);
				cout << "The Fine registration using ICP lead to a fitness score of: " << score << ".\n";
				PCL_INFO("Please ensure, that the clouds are aligned properly.\n");
			}
		}
		else {
			PCL_WARN("Registration failed. Check alignment manually! Consider doing a new recording.");
		}
		Visualizer.updateCloud(cam);
		Visualizer.processOutput();
		Visualizer.updateCloud(cam_sparse,comp_sparse);
		Visualizer.processOutput();
		PCL_INFO("Registration finished. Saving point cloud MeasuredCloudAfterRegistration.pcd...\n");
		pcl::io::savePCDFileASCII("MeasuredCloudAfterRegistration.pcd", *cam);
	}
}
void Menu::merge() {
	PTC::Ptr cam1(new PTC);
	PTC::Ptr cam2(new PTC);
	if (io.loadCloud("What is the path of the first cloud?\n", cam1) &&
		io.loadCloud("What is the path of the second cloud?\n", cam2)) {
		PTC::Ptr merged(new PTC(*cam1 + *cam2));
		PCL_INFO("Saving point cloud Merged.pcd...\n");
		pcl::io::savePCDFileASCII("Merged.pcd", *merged);
	}
}
void Menu::hausdorff() {
	Comparison Compare(parameters.at("debug"));
	PTC::Ptr comp(new PTC);
	PTC::Ptr cam(new PTC);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr comp_hausdorff(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (io.loadCloud("What is the path of the component file? (type \"default\" for: testobjekt-fixed.ply)\n", comp, "testobjekt-fixed.ply",true) &&
		io.loadCloud("What is the path of the measured point cloud? type \"default\" for: MeasuredCloudAfterRegistration.pcd)\n", cam, "MeasuredCloudAfterRegistration.pcd")) {
		Registration Reg(parameters.at("debug"));
		double leafSize = 0.0005;
		Reg.downsample(comp, comp, leafSize);
		Reg.downsample(cam, cam, leafSize);
		cout << "Starting Hausdorff comparison...\n";
		float hausdorff;
		float avg_dist;
		std::vector<int> defects;
		if (Compare.hausdorff(cam, comp, comp_hausdorff,defects, hausdorff, avg_dist)) {
			cout << "The component contains defects with a deviation of greater than 2mm!\n";
		}
		else {
			cout << "All deviations (except the detected outliers) are within 2mm. However, check the orange marked points, which show deviations of more than 1mm.\n";
		}
		cout << "The hausdorff distance from cam to comp is: " << hausdorff << "\n";
		cout << "The average distance from cam to comp is: " << avg_dist << "\n";
		Visual clouds("Component and Measured Cloud", cam,comp);
		Visual colored("Hausdorff cloud", comp_hausdorff);
		PCL_INFO("Hausdorff Comparison finished. Saving point cloud HausdorffCloud.pcd...\n");
		pcl::io::savePCDFileASCII("HausdorffCloud.pcd", *comp_hausdorff);
		clouds.processOutput();
	}
}
void Menu::features() {
	Comparison Compare(parameters.at("debug"));
	PTC::Ptr comp(new PTC);
	PTC::Ptr cam(new PTC);
	if (io.loadCloud("What is the path of the component file? (type \"default\" for: testobjekt-fixed.ply)\n", comp, "testobjekt-fixed.ply", true) &&
		io.loadCloud("What is the path of the measured point cloud? (type \"default\" for: MeasuredCloudAfterRegistration.pcd)\n", cam, "MeasuredCloudAfterRegistration.pcd")) {
		Registration Reg(parameters.at("debug"));
		double leafSize = 0.0005;
		Reg.downsample(comp, comp, leafSize);
		Reg.downsample(cam, cam, leafSize);
		cout << "Detecting features in the component cloud...\n";
		Compare.detectFeatures(comp,true); //detect features in Component
		Compare.findMeasuredFeatures(cam,comp); //find features in measured cloud
		Compare.parametrizeFeatures(); //parametrize everything and save it for later use
		Compare.visualizeFeatures(cam,comp);
	}
}
void Menu::tolerances() {
	PTC::Ptr cam(new PTC);
	if (io.loadTolerances()&&io.loadCloud("What is the path of the measured point cloud? (type \"default\" for: MeasuredCloudAfterRegistration.pcd)\n", cam, "MeasuredCloudAfterRegistration.pcd")) {
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		Registration Reg(parameters.at("debug"));
		double leafSize = 0.0005;
		Reg.downsample(cam, cam, leafSize);
		Comparison Compare(parameters.at("debug"));
		Compare.checkTolerances(cam,io.tolerances);
		PTC::Ptr tolerance_points(new PTC);
		for (Dimension tolerance : io.tolerances) {
			for (PTC::Ptr points : tolerance.references)
				tolerance_points.reset(new PTC(*tolerance_points + *points));
			for (PTC::Ptr points : tolerance.measuredReferences)
				tolerance_points.reset(new PTC(*tolerance_points + *points));
		}
		Visual Tol("Tolerances",cam, tolerance_points);
		Tol.processOutput();
	/*	for (Dimension tolerance : io.tolerances) {
			Tol.addText(std::to_string(tolerance.value) + " tolerance: " + std::to_string(tolerance.upperDelta) + " " + std::to_string(tolerance.lowerDelta)+"\nmeasured:"+ std::to_string(tolerance.measured), PT(((*tolerance.points)[0].x + (*tolerance.points)[1].x) / 2, ((*tolerance.points)[0].y + (*tolerance.points)[1].y) / 2, ((*tolerance.points)[0].z + (*tolerance.points)[1].z) / 2));
		}*/
		Tol.processOutput();
		Tol.closeViewer();
	}
}