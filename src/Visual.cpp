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
Visual::Visual(std::string name, PTC::Ptr cloudA, PTC::Ptr cloudB, PTC::Ptr cloudC) {
	viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Init"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_cloud(cloudA, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_component(cloudB, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_cloudC(cloudC, 0, 0, 255);
	viewer->addPointCloud(cloudA, color_cloud, "cloud");
	viewer->addPointCloud(cloudB, color_component, "component");
	viewer->addPointCloud(cloudC, color_cloudC, "cloudC");
	setOptions(viewer, name);
}
Visual::Visual(std::string name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Init"));
	if (cloud->points[0].r == 0 && cloud->points[0].g == 0 && cloud->points[0].b == 0) {
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_cloud(cloud, 0, 255, 0);
		viewer->addPointCloud(cloud, color_cloud, "cloud");
	}
	else 
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
void Visual::updateCloud(PTC::Ptr cloudA, PTC::Ptr cloudB,PTC::Ptr cloudC) {
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_cloudC(cloudC, 0, 0, 255);
	viewer->updatePointCloud(cloudC, color_cloudC, "cloudC");
	updateCloud(cloudA,cloudB);
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
void Visual::addText(std::string text, PT position,int r,int g, int b) {
	viewer->addText3D(text, position, 0.001, r, g, b);
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