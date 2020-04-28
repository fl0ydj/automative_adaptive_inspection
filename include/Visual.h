#ifndef VISUAL_H_
#define VISUAL_H_
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
typedef pcl::PointXYZ PT;
typedef pcl::PointCloud<PT> PTC;
/**@brief Class for Visualization of Point Clouds.
@author Justin Heinz
*/
class Visual
{
private:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    void setOptions(pcl::visualization::PCLVisualizer::Ptr viewer, std::string name);
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	/**
	 Standard constructor
	*/
    Visual();
	/**
	Constructor to show the camera cloud in a new PCLVisualizer window with a specific name.
	 @param[in] name - the name of the window
	 @param[in] cloud - the cloud to be visualized (green)
	*/
    Visual(std::string name, PTC::Ptr cloud);
	/**
	Constructor to show the camera cloud as well as the component cloud in a new PCLVisualizer window with a specific name.
	 @param[in] name - the name of the window
	 @param[in] cloud - the camera cloud to be visualized (green)
	 @param[in] component - the component cloud to be visualized (red)
	*/
    Visual(std::string name, PTC::Ptr cloud, PTC::Ptr component);
	/**
	Constructor to show three clouds in a new PCLVisualizer window with a specific name.
	 @param[in] name - the name of the window
	 @param[in] cloudA - the first cloud to be visualized (green)
	 @param[in] cloudB - the second cloud to be visualized (red)
	 @param[in] cloudC - the third cloud to be visualized (blue)
	*/
	Visual(std::string name, PTC::Ptr cloudA, PTC::Ptr cloudB,PTC::Ptr cloudC);
	/** Constructor to show the a colored cloud in a new PCLVisualizer window with a specific name. 
	@param[in] name - the name of the window
	@param[in] cloud - the RGB cloud to be visualized
	*/
    Visual(std::string name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	/**
	 Updates the cloud visualized.
	 @param[in] cloud - the cloud to be updated
	*/
    void updateCloud(PTC::Ptr cloud);
	/**
	 Updates the clouds visualized.
	 @param[in] cloud - the cloud to be updated
	 @param[in] component - the component cloud to be updated
	*/
    void updateCloud(PTC::Ptr cloud, PTC::Ptr component);
	/**
	 Updates the clouds visualized.
	 @param[in] cloudA - the first cloud to be updated
	 @param[in] cloudB - the second cloud to be updated
	  @param[in] cloudC - the third cloud to be updated
	*/
	void updateCloud(PTC::Ptr cloudA, PTC::Ptr cloudB,PTC::Ptr cloudC);
	/**
	 Updates the colored cloud visualized.
	 @param[in] cloud - the cloud to be updated
	*/
    void updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud);
	/** Updates the window. Waits for the user to press SPACE.
	*/
    void processOutput();
	/**
	Adds a 2D Text to the visualizer.
	@param[in] text - the text to display
	@param[in] color - For "r", the text is displayed in red with a slight vertical offset in the upper left hand corner. For "g", the text is displayed in green in the upper left hand corner. Else, the text is displayed in white in the upper left hand corner.
	*/
	void addText(std::string text,char color='0');
	/**
	Adds a 3D Text to the visualizer.
	@param[in] text - the text to display
	@param[in] position - Sets the position of the upper left hand corner of the text.
	@param[in] r - Sets the red value
	@param[in] g - Sets the green value
	@param[in] b - Sets the blue value
	*/
	void addText(std::string text, PT position,int r=255,int g=255, int b=255);
	/**
	Adds normals to a cloud in the visualizer.
	@param[in] cloud - the cloud to attach normals to
	@param[in] normals - pointer to the normals to use
	*/
	void addNormals(PTC::Ptr cloud,pcl::PointCloud<pcl::Normal>::Ptr normals);
	/**
	Closes the viewer window.
	*/
	void closeViewer();
};
#endif