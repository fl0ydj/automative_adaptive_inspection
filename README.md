This is the git repository for the Bachelor's Thesis "Adaptive Analysis of Geometric Tolerances with Low-Cost Sensors".

How to setup the programming environment
===========================================
In the following it is described briefly how to setup all necessary libraries used in this project.

The references are set mostly using variables in CMakeLists.txt which have to be updated accordingly. Alternatively, environment variables could be set.
Visual Studio and Cmake
-----------------------
Visual Studio 2019 was used and can be downloaded here: https://visualstudio.microsoft.com/vs/.
Visual Studio Cmake Tools have to be installed.

Furthermore, Cmake itself has to be installed. The minimum version is 3.6: https://cmake.org/download/.
Turntable HW-Control
--------------------
The files have to be downloaded from the github directory and the source code has to be built.

Afterwards, in CMakeLists TURNTABLE_DIR has to be adjusted and DLL_TURNTABLE has to be checked.
Realsense SDK
-------------
The realsense sdk can be downloaded here: https://github.com/IntelRealSense/librealsense/releases.

Version 2.33.1 was used for this project.
Adjust REALSENSE_DIR in CmakeLists accordingly.
Point Cloud Library 
-------------------
PCL can be downloaded here: https://github.com/PointCloudLibrary/pcl/releases.
The Version used for this project was 1.9.1.

Using the windows installer, an environment variable should be set automatically. If not, add it manually: PCL_ROOT (to the root directory of PCL).

Furthermore add %PCL_ROOT%\bin to your PATH variable.
Open CV
-------
Download OpenCV and compile it: https://github.com/opencv/opencv.

OpenCV contrib has to be downloaded and built as well (some modules are used from that): https://github.com/opencv/opencv_contrib.

Adjust OPENCV_DIR accordingly. 
JT Open Library
---------------
JT Open Libary can be downloaded here: https://www.plm.automation.siemens.com/store/en-us/trial/jt2-open.html.

After registration, a 60 days trial version begins.

In CMakeLists the variables JTTK_LIB and JTTK_INCLUDE have to be adjusted accordingly.