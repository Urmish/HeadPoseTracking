CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set(name "video")

find_package(PCL 1.7 REQUIRED COMPONENTS common io visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

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

include_directories(/home/urmish/Documents/OpenCVLib_Openni/include/)

find_path(EIGEN_INCLUDE_DIR Eigen/Core
        /usr/local/include/eigen3
        /usr/include/eigen3
)
include_directories(${EIGEN_INCLUDE_DIR})
find_package(libpointmatcher 1.1.0 REQUIRED)
include_directories("${libpointmatcher_INCLUDE_DIRS}")
message(STATUS "Using libpointmatcher version ${libpointmatcher_VERSION}")

add_executable(withPclViewer ppt_generator.cpp)
#target_link_libraries(pcl opencv_core opencv_imgproc opencv_highgui opencv_objdetect opencv_imgproc opencv_calib3d opencv_features2d ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} )
target_link_libraries(withPclViewer opencv_core opencv_imgproc opencv_highgui opencv_objdetect opencv_imgproc opencv_calib3d opencv_features2d ${PCL_LIBRARIES} pcl_filters OpenNI XnCore ${libpointmatcher_LIBRARIES})

