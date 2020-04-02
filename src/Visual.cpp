#include "Visual.h"
bool next = false;
void
keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
	void* nothing)
{
	if (event.getKeySym() == "space" &&event.keyDown())
		next = true;
}
void Visual::closeViewer() {
	viewer->close();
}
Visual::Visual() {}
Visual::Visual(std::string name, PTC::Ptr cloud) {
	viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Init"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_cloud(cloud, 0, 255, 0);
	viewer->addPointCloud(cloud, color_cloud,"cloud");
	setOptions(viewer, name);
}
Visual::Visual(std::string name, PTC::Ptr cloud, PTC::Ptr component) {
	viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Init"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_cloud(cloud, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_component(component, 255, 0, 0);
	viewer->addPointCloud(cloud, color_cloud,"cloud");
	viewer->addPointCloud(component, color_component,"component");
	setOptions(viewer, name);
}
Visual::Visual(std::string name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Init"));
	viewer->addPointCloud(cloud, "cloud");
	setOptions(viewer, name);
}
void Visual::setOptions(pcl::visualization::PCLVisualizer::Ptr viewer, std::string name) {
	viewer->setWindowName(name);
	viewer->setBackgroundColor(0.0, 0.0, 0.0);
	viewer->addCoordinateSystem(0.01);
	viewer->initCameraParameters();
	viewer->registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);
	viewer->spinOnce();
}
void Visual::updateCloud(PTC::Ptr cloud) {
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_cloud(cloud, 0, 255, 0);
	viewer->updatePointCloud(cloud, color_cloud,"cloud");
	viewer->spinOnce();
}
void Visual::updateCloud(PTC::Ptr cloud,PTC::Ptr component) {
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_component(component, 255, 0, 0);
	viewer->updatePointCloud(component, color_component,"component");
	updateCloud(cloud);
}
void Visual::updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud) {
	viewer->updatePointCloud(colorCloud, "cloud");
	processOutput();
}
void Visual::addText(std::string text,char color) {
	if(color=='r')
		viewer->addText(text, 0, 0,255,150,150, "Text");
	else if(color=='g')
		viewer->addText(text, 0, 500, 150, 255, 150, "Textu");
	else
		viewer->addText(text, 0, 0, "Text");
}
void Visual::addText(std::string text, PT position) {
	viewer->addText3D(text, position);
}
void Visual::addNormals(PTC::Ptr cloud,pcl::PointCloud<pcl::Normal>::Ptr normals) {
	viewer->addPointCloudNormals<PT, pcl::Normal>(cloud, normals, 100, 0.005, "normals");
}
void Visual::processOutput() {
	next = false;
	do {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	} while (!next); //repeat until space pressed
}