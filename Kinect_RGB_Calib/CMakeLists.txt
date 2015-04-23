CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set(name "video")

set(OpenCV_DIR /home/urmish/Documents/OpenCVLib/share/OpenCV)
set(CMAKE_FIND_LIBRARY_SUFFIXES ".so")
set(BUILD_SHARED_LIBRARIES ON)

include_directories(/home/urmish/Documents/OpenCVLib/)
#---------------------------------
# packages
#---------------------------------
find_package(OpenCV REQUIRED) # http://opencv.willowgarage.com

set(CMAKE_VERBOSE_MAKEFILE on)

set(CMAKE_BUILD_TYPE "RELEASE")

add_executable(camera_caliberation camera_calibration.cpp)
target_link_libraries(camera_caliberation opencv_core opencv_imgproc opencv_highgui opencv_objdetect opencv_imgproc opencv_calib3d opencv_features2d)

