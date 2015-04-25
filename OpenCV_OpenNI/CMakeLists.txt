CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set(name "video")
#set (CMAKE_PREFIX_PATH /home/urmish/Documents/OpenCVLib_Openni)
#set(CMAKE_LIBRARY_PATH  /home/urmish/Documents/OpenCVLib_Openni/lib)
set(OpenCV_DIR /home/urmish/Documents/OpenCVLib_Openni/)
set(CMAKE_FIND_LIBRARY_SUFFIXES ".so")
set(BUILD_SHARED_LIBRARIES ON)

include_directories(/home/urmish/Documents/OpenCVLib_Openni/include/)
#---------------------------------
# packages
#---------------------------------
find_package(OpenCV REQUIRED) # http://opencv.willowgarage.com

set(CMAKE_VERBOSE_MAKEFILE on)

set(CMAKE_BUILD_TYPE "RELEASE")

add_executable(cvni openni_capture.cpp)
#target_link_libraries(pcl opencv_core opencv_imgproc opencv_highgui opencv_objdetect opencv_imgproc opencv_calib3d opencv_features2d ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} )
target_link_libraries(cvni opencv_core opencv_imgproc opencv_highgui opencv_objdetect opencv_imgproc opencv_calib3d opencv_features2d OpenNI XnCore)

