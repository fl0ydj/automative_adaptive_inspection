#ifndef MENU_H_
#define MENU_H_
#include "Depthcam.h"
#include "Registration.h"
#include "Comparison.h"
#include "FileIO.h"
#include "HW_control.h"
#include "Serial.h"
#include "HW_control_Export.h"
typedef pcl::PointXYZ PT;
typedef pcl::PointCloud<PT> PTC;
class Menu {
private:
	FileIO io;
	std::vector<Depthcam> Cams;
	int MasterCamIndex = 0;
	int camcount = 0;
	//PARAMETERS - DEFAULT VALUES, actual values are loaded from parameters.txt
	std::map<std::string, std::string>  parameters;
	//GENRAL DEBUG LEVEL: 0 - no Debug Output, 1 - some Debug Output, 2 - all Debug Output
	std::string debug = "2";
	//1. Image Recording
	std::string preset = "HighAccuracy.json";
	std::string disparity_shift = "175";
	//2. Registration
	std::string leafPoints = "100";
	std::string normalsSearchRadius = "5";
	std::string FPFHSearchRadius = "5";
	std::string samplesInObject = "2";
	std::string maxCorrespondenceDistance_SAC = "0.15";
	std::string numberOfIterations_SAC = "5000";
	std::string maxCorrespondenceDistance_ICP = "0.05";
	std::string numberOfIterations_ICP = "1000";
public:
	Menu();
	void reconnect();
	void displayIntro();
	void extrinsics();
	void intrinsics();
	void record();
	void registration();
	void merge();
	void hausdorff();
	void features();
	void settings();
	void view();
	void snap();
	void tolerances();
};
#endif