#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::cout << "Enter filename.\n";
  std::string filename;
  std::cin >> filename;
  pcl::io::loadPCDFile (filename, *cloud_in);
  std::cout << "Enter another filename.\n";
  std::cin >> filename;
  pcl::io::loadPCDFile (filename, *cloud_out);
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  icp.setInputCloud(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final (new pcl::PointCloud<pcl::PointXYZRGB>);
  icp.align(*Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb1 (cloud_in, 255, 0, 0); //This will display the point cloud in red (R,G,B)
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb2 (cloud_out, 0, 255, 0); //This will display the point cloud in green (R,G,B)
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb3 (Final, 0, 0, 255); //This will display the point cloud in green (R,G,B)
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud_in, rgb1, "sample cloud");
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud_out, rgb2, "sample cloud2");
  viewer->addPointCloud<pcl::PointXYZRGB> (Final, rgb3, "Final cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

 return (0);
}
