cmake_minimum_required(VERSION 3.6 FATAL_ERROR)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
if(POLICY CMP0074)
  # TODO:
  # 1. Find*.cmake modules need to be individually verified.
  # 2. PCLConfig.cmake needs to be changed.
  cmake_policy(SET CMP0074 OLD)
endif()
project(BACHELORARBEIT)
# Make project require C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()
set(CALIBRATE_DIR "C:/CalibrationToolAPI/2.6.8.0")
set (TURNTABLE_DIR "D:/HW_control") # TODO: Update this variable to correct path - folder where HW_Control folder is found
set (REALSENSE_DIR "C:/Program Files (x86)/Intel RealSense SDK 2.0") # TODO: Update this variable to correct path - folder where realsense2.lib is found
set (JTTK_LIB "D:/Program Files (x86)/Siemens/JTOpenToolkit/10_5/dev/lib/win_64_VS2017") # TODO: Update this variable to correct path - folder where realsense2.lib is found
set(JTTK_INCLUDE "D:/Program Files (x86)/Siemens/JTOpenToolkit/10_5/dev/include") # TODO: Update this variable to correct path - folder where realsense2.lib is found
set(OPENCV_DIR D:/opencv/build/x64/vc14/bin) #"D:/opencv/build/bin/Release")  # TODO: Update this variable to correct path - folder where opencv_world420d.dll is found
# Simple non robust way to find the librealsense library
if(WIN32)
    if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "8")
        set(LIBRARY_DIR ${REALSENSE_DIR}/lib/x64)          
        set(DLL_DIR ${REALSENSE_DIR}/bin/x64)             
        set(DLL_TURNTABLE ${TURNTABLE_DIR}/out/build/x64-Debug)                                # TODO: Check if this path is correct
    else()
        set(LIBRARY_DIR ${REALSENSE_DIR}/lib/x86)
        set(DLL_DIR ${REALSENSE_DIR}/bin/x86)
        set(DLL_TURNTABLE ${TURNTABLE_DIR}/out/build/x64-Debug)  # TODO: Check if this path is correct
    endif()
    set(DLL_CALIBRATE ${CALIBRATE_DIR}/bin)
    set(LIB_CALIBRATE ${CALIBRATE_DIR}/lib)
    set(PROJECT_BINARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}")
    set(ADDITIONAL_INCLUDE_DIRS ${REALSENSE_DIR}/include "D:/OpenCV/opencv/build/include")
endif()

find_library(REALSENSE2_FOUND realsense2 HINTS ${LIBRARY_DIR} REQUIRED)
if(NOT REALSENSE2_FOUND)
    SET(REALSENSE2_FOUND "realsense2")
    message(WARN "Failed to find_library(realsense2)")
endif()
find_library(HW_CONTROL_FOUND HW_control HINTS ${DLL_TURNTABLE} REQUIRED)
if(NOT HW_CONTROL_FOUND)
    SET(HW_CONTROL_FOUND "HW_control")
    message(WARN "Failed to find_library(HW_control)")
endif()
find_library(JTTK_FOUND JtTk105 HINTS ${JTTK_LIB} REQUIRED)
if(NOT JTTK_FOUND)
    SET(JTTK_FOUND "JtTk105")
    message(WARN "Failed to find_library(JtTk105)")
endif()
find_package(OpenCV REQUIRED)
find_package(PCL 1.3 REQUIRED)

add_executable(main
src/main.cpp
src/Depthcam.cpp
src/Visual.cpp
src/Registration.cpp
src/Comparison.cpp
src/Menu.cpp
src/FileIO.cpp
include/Depthcam.h
include/Visual.h
include/Registration.h
include/Comparison.h
include/Menu.h
include/FileIO.h
include/Dimension.hpp
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(main ${CMAKE_CURRENT_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS} ${ADDITIONAL_INCLUDE_DIRS} ${TURNTABLE_DIR}/include  ${DLL_TURNTABLE} ${JTTK_INCLUDE})
target_link_libraries(main ${REALSENSE2_FOUND} ${JTTK_FOUND} ${PCL_LIBRARIES} ${HW_CONTROL_FOUND} ${OpenCV_LIBS}) #DLL has to be created manually when a new file location is wanted

# Post Build script to copy realsense2.dll and HW_control.dll
if(WIN32)
message(STATUS "Adding Post build script to copy realsense2.dll and HW_control.dll and DSDynamicCalibrationAPI.dll and opencv_world420.dll to project's binary folder")
message(STATUS "Will try to copy from ${DLL_DIR} and ${DLL_TURNTABLE} and ${OPENCV_DIR} to ${PROJECT_BINARY_OUTPUT_PATH}")
add_custom_command(TARGET main POST_BUILD        # Adds a post-build event
    COMMAND ${CMAKE_COMMAND} -E copy_if_different              # which executes "cmake - E copy_if_different..."
        "${DLL_DIR}/realsense2.dll"                            # <--this is in-file
        ${PROJECT_BINARY_OUTPUT_PATH})                 # <--this is out-file path
add_custom_command(TARGET main POST_BUILD        # Adds a post-build event
    COMMAND ${CMAKE_COMMAND} -E copy_if_different              # which executes "cmake - E copy_if_different..."
        "${DLL_TURNTABLE}/HW_control.dll"                            # <--this is in-file
        ${PROJECT_BINARY_OUTPUT_PATH})                 # <--this is out-file path
add_custom_command(TARGET main POST_BUILD        # Adds a post-build event
    COMMAND ${CMAKE_COMMAND} -E copy_if_different              # which executes "cmake - E copy_if_different..."
        "${OPENCV_DIR}/opencv_world420.dll"                            # <--this is in-file
        ${PROJECT_BINARY_OUTPUT_PATH})   
add_custom_command(TARGET main POST_BUILD        # Adds a post-build event
    COMMAND ${CMAKE_COMMAND} -E copy_if_different              # which executes "cmake - E copy_if_different..."
        "${JTTK_LIB}/JtTk105.dll"                            # <--this is in-file
        ${PROJECT_BINARY_OUTPUT_PATH}) 
endif()

install(
    TARGETS

    main

    RUNTIME DESTINATION
    ${CMAKE_INSTALL_PREFIX}/bin
)