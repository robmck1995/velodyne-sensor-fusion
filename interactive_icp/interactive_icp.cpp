#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/console/parse.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;

void
print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void
keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
                       void* nothing)
{
  if (event.getKeySym () == "space" && event.keyDown ())
    next_iteration = true;
}

int
main (int argc,
      char* argv[])
{
  // The point clouds we will be using
  PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
  PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud
  PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud

  // Checking program arguments

  int xy_degrees = 0;
  int xz_degrees = 0;
  int yz_degrees = 0;

  double x_trans = 0.0;
  double y_trans = 0.0;
  double z_trans = 0.0;

  int iterations = 0;
  pcl::console::TicToc time;
  time.tic ();
  std::string ply_extension = ".ply";
  std::string pcd_extension = ".pcd";
  std::string fname;
  pcl::console::parse_argument(argc, argv, "-file", fname);
  pcl::console::parse_argument(argc, argv, "-x", x_trans);
  pcl::console::parse_argument(argc, argv, "-y", y_trans);
  pcl::console::parse_argument(argc, argv, "-z", z_trans);
  pcl::console::parse_argument(argc, argv, "-xy_deg", xy_degrees);
  pcl::console::parse_argument(argc, argv, "-xz_deg", xz_degrees);
  pcl::console::parse_argument(argc, argv, "-yz_deg", yz_degrees);
  pcl::console::parse_argument(argc, argv, "-iter", iterations);
  if (fname.compare(fname.size() - ply_extension.size(), ply_extension.size(), ply_extension) == 0 && pcl::io::loadPLYFile (fname, *cloud_in) < 0)
  {
    PCL_ERROR ("Error loading cloud %s.\n", fname);
    return (-1);
  }

  if (fname.compare(fname.size() - pcd_extension.size(), pcd_extension.size(), pcd_extension) == 0 && pcl::io::loadPCDFile (fname, *cloud_in) < 0)
  {
    PCL_ERROR ("Error loading cloud %s.\n", fname);
    return (-1);
  }

  std::cout << "\nLoaded file " << fname << " (" << cloud_in->size () << " points) in " << time.toc () << " ms\n" << std::endl;

  // Defining a rotation matrix and translation vector
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

  // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
  double theta = xy_degrees*(M_PI / 180);  // The angle of rotation in radians
  transformation_matrix (0, 0) = cos (theta);
  transformation_matrix (0, 1) = -sin (theta);
  transformation_matrix (1, 0) = sin (theta);
  transformation_matrix (1, 1) = cos (theta);

  // A translation on Z axis
  transformation_matrix (2, 3) = z_trans;

  // Display in terminal the transformation matrix
  std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
  print4x4Matrix (transformation_matrix);
  pcl::transformPointCloud (*cloud_in, *cloud_icp, transformation_matrix);

  transformation_matrix = Eigen::Matrix4d::Identity ();
  theta = xz_degrees*(M_PI / 180);  // The angle of rotation in radians
  transformation_matrix (0, 0) = cos (theta);
  transformation_matrix (0, 2) = -sin (theta);
  transformation_matrix (2, 0) = sin (theta);
  transformation_matrix (2, 2) = cos (theta);

  // A translation on Y axis
  transformation_matrix (1, 3) = y_trans;

  // Display in terminal the transformation matrix
  std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
  print4x4Matrix (transformation_matrix);

  pcl::transformPointCloud (*cloud_icp, *cloud_icp, transformation_matrix);

  transformation_matrix = Eigen::Matrix4d::Identity ();
  theta = yz_degrees*(M_PI / 180);  // The angle of rotation in radians
  transformation_matrix (1, 1) = cos (theta);
  transformation_matrix (1, 2) = -sin (theta);
  transformation_matrix (2, 1) = sin (theta);
  transformation_matrix (2, 2) = cos (theta);

  // A translation on X axis
  transformation_matrix (0, 3) = x_trans;

  // Display in terminal the transformation matrix
  std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
  print4x4Matrix (transformation_matrix);

  pcl::transformPointCloud (*cloud_icp, *cloud_icp, transformation_matrix);
  *cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use

  // The Iterative Closest Point algorithm
  time.tic ();
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations (iterations);
  icp.setInputSource (cloud_icp);
  icp.setInputTarget (cloud_in);
  if(iterations > 0) {
    icp.align (*cloud_icp);
  }
  icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
  std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

  // Visualization
  pcl::visualization::PCLVisualizer viewer ("ICP demo");
  // Create two verticaly separated viewports
  int v1 (0);
  int v2 (1);
  viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

  // The color we will be using
  float bckgr_gray_level = 0.0;  // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  // Original point cloud is white
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                             (int) 255 * txt_gray_lvl);
  viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
  viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

  // Transformed point cloud is green
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_tr, 20, 180, 20);
  viewer.addPointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

  // ICP aligned point cloud is red
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
  viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

  // Adding text descriptions in each viewport
  viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
  viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

  std::stringstream ss;
  ss << iterations;
  std::string iterations_cnt = "ICP iterations = " + ss.str ();
  viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

  // Set background color
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

  // Set camera position and orientation
  viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
  viewer.setSize (1280, 1024);  // Visualiser window size

  // Register keyboard callback :
  viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);

  // Display the visualiser
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();

    // The user pressed "space" :
    if (next_iteration)
    {
      // The Iterative Closest Point algorithm
      time.tic ();
      icp.align (*cloud_icp);
      std::cout << "Applied 1 ICP iteration in " << time.toc () << " ms" << std::endl;

      if (icp.hasConverged ())
      {
        printf ("\033[11A");  // Go up 11 lines in terminal output.
        printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
        std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
        transformation_matrix *= icp.getFinalTransformation ().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
        print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose

        ss.str ("");
        ss << iterations;
        std::string iterations_cnt = "ICP iterations = " + ss.str ();
        viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
        viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
      }
      else
      {
        PCL_ERROR ("\nICP has not converged.\n");
        return (-1);
      }
    }
    next_iteration = false;
  }
  return (0);
}
