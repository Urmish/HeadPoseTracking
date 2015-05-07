#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <string>
#include <stdio.h>
#include <pcl/common/transforms.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}
int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
//  for (size_t i = 0; i < cloud->points.size (); ++i)
//    std::cout << "    " << cloud->points[i].x
//              << " "    << cloud->points[i].y
//              << " "    << cloud->points[i].z << std::endl;
//
 
 //...
  //pcl::PointCloud<pcl::PointXYZRGB> cloud;
   //... populate cloud
  std::vector<int> indices; 
  pcl::removeNaNFromPointCloud(*cloud,*cloud_filtered, indices);	
  //pcl_visualization::CloudViewer viewer("Simple Cloud Viewer");
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

  // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
  float theta = M_PI/4; // The angle of rotation in radians
  transform_1 (0,0) = cos (theta);
  transform_1 (0,1) = -sin(theta);
  transform_1 (1,0) = sin (theta);
  transform_1 (1,1) = cos (theta);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // You can either apply transform_1 or transform_2; they are the same
  viewer = simpleVis(cloud_filtered);
  while (!viewer->wasStopped ())
  {
  	viewer->spinOnce (100);
  	boost::this_thread::sleep (boost::posix_time::microseconds (100));
        viewer->removePointCloud("sample cloud");
  	pcl::transformPointCloud (*cloud_filtered, *transformed_cloud, transform_1);
	cloud_filtered = transformed_cloud;
  	viewer->addPointCloud<pcl::PointXYZ> (cloud_filtered, "sample cloud");
  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  }
  
  return (0);
}
