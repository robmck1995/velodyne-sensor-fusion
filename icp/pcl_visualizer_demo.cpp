#include <iostream>
#include <string>
#include <sstream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

// --------------
// -----Help-----
// --------------
void
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-f           Choose custom filename of sample\n"
            << "-n           Normals visualisation example\n"
            << "-d           Choose filenames of multiple samples\n"
            << "\n\n";
}


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

boost::shared_ptr<pcl::visualization::PCLVisualizer> doubleVis (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  int colors[9][3] = {{255,0,0},{255,255,0},{0,255,0},{0,0,255},{255,0,255},{255,255,255},{255,128,0},{0,255,255},{255,0,128}};
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  for(int i = 0; i < clouds.size(); i++) {
    int r = colors[i%9][0];
    int g = colors[i%9][1];
    int b = colors[i%9][2];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = clouds.at(i);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb (cloud, r, g, b); //This will display the point cloud in red (R,G,B)
    std::ostringstream id;
    id << "cloud " << i;
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, id.str());
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id.str());
  }
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 1, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}
// --------------
// -----Main-----
// --------------
int main (int argc, char** argv)
{
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }
  bool custom_d(false), custom_s(false), normals(false);
  pcl::PointCloud<pcl::PointXYZ>::Ptr single_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr normal_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
  if (pcl::console::find_argument (argc, argv, "-d") >= 0)
  {
    std::string dir;
    std::cout << "Enter cloud directory." << std::endl;
    std::cin >> dir;
    std::string file = "0";
    while(file != "-1") {
      std::cout << "Enter filename or -1 to finish input.\n";
      std::cin >> file;
      if(file == "-1") {
        std::cout << "End" << std::endl;
        break;
      }
      std::ostringstream filename;
      filename << dir << "/" << file;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::io::loadPCDFile (filename.str(), *cloud_ptr);
      clouds.push_back(cloud_ptr);
    }
    custom_d = true;
  }

  else if (pcl::console::find_argument (argc, argv, "-f") >= 0)
  {
    std::string fname;
    pcl::console::parse_argument(argc, argv, "-f", fname);
    pcl::io::loadPCDFile (fname, *single_cloud_ptr);
    custom_s = true;
  }
  else if (pcl::console::find_argument (argc, argv, "-n") >= 0)
  {
    normals = true;
    std::string fname;
    pcl::console::parse_argument(argc, argv, "-n", fname);
    pcl::io::loadPCDFile (fname, *normal_cloud_ptr);
    std::cout << "Normals visualisation example\n";
  }
  else
  {
    printUsage (argv[0]);
    return 0;
  }

  // ----------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.05-----
  // ----------------------------------------------------------------
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (normal_cloud_ptr);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.05);
  ne.compute (*cloud_normals1);

  // ---------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.1-----
  // ---------------------------------------------------------------
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (1);
  ne.compute (*cloud_normals2);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  if (custom_s)
  {
    viewer = simpleVis(single_cloud_ptr);
  }
  else if (custom_d)
  {
    viewer = doubleVis(clouds);
  }
  else if (normals)
  {
    viewer = normalsVis(normal_cloud_ptr, cloud_normals2);
  }

  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
