CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set(name "video")

set(OpenCV_DIR /home/urmish/Documents/OpenCVLib/share/OpenCV)
set(CMAKE_FIND_LIBRARY_SUFFIXES ".so")
set(BUILD_SHARED_LIBRARIES ON)

include_directories(/home/urmish/Documents/OpenCVLib/)
include_directories(/usr/local/include/libfreenect/)
include_directories(/usr/include/)
#---------------------------------
# packages
#---------------------------------
find_package(OpenCV REQUIRED) # http://opencv.willowgarage.com

set(CMAKE_VERBOSE_MAKEFILE on)

set(CMAKE_BUILD_TYPE "RELEASE")

add_executable(test-kinect main.cpp)
target_link_libraries(test-kinect opencv_core opencv_imgproc opencv_highgui opencv_objdetect opencv_imgproc freenect freenect_cv freenect_sync -lusb-1.0 -lpthread)

