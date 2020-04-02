#ifndef VISUAL_H_
#define VISUAL_H_
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
typedef pcl::PointXYZ PT;
typedef pcl::PointCloud<PT> PTC;
class Visual
{
private:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    void setOptions(pcl::visualization::PCLVisualizer::Ptr viewer, std::string name);
public:
	/*
	* Standard constructor
	*/
    Visual();
	/*
	* Constructor to show the camera cloud in a new PCLVisualizer window with a specific name.
	* @param name: the name of the window [input]
	* @param cloud: the cloud to be visualized [input]
	* @return void
	*/
    Visual(std::string name, PTC::Ptr cloud);
	/*
	* Constructor to show the camera cloud as well as the component cloud in a new PCLVisualizer window with a specific name. 
	* @param name: the name of the window [input]
	* @param cloud: the camera cloud to be visualized [input]
	* @param cloud: the component cloud to be visualized [input]
	* @return void
	*/
    Visual(std::string name, PTC::Ptr cloud, PTC::Ptr component);
	/*
	* Constructor to show the a colored cloud in a new PCLVisualizer window with a specific name. 
	* @param name: the name of the window [input]
	* @param cloud: the cloud to be visualized [input]
	* @return void
	*/
    Visual(std::string name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	/*
	* Updates the cloud visualized.
	* @param cloud: the cloud to be updated [input]
	* @return void
	*/
    void updateCloud(PTC::Ptr cloud);
	/*
	* Updates the camera cloud visualized and the component cloud.
	* @param cloud: the camera cloud to be updated [input]
	* @param component: the component cloud to be updated [input]
	* @return void
	*/
    void updateCloud(PTC::Ptr cloud, PTC::Ptr component);
	/*
	* Updates the colored cloud visualized.
	* @param cloud: the cloud to be updated [input]
	* @return void
	*/
    void updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud);
	/*
	* Updates the window. If DEBUG flag is set, waits for the user to press SPACE.
	* @return void
	*/
    void processOutput();
	void addText(std::string text,char color='0');
	void addText(std::string text, PT position);
	void addNormals(PTC::Ptr cloud,pcl::PointCloud<pcl::Normal>::Ptr normals);
	void closeViewer();
};
#endif