#include "simple_grabber.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

using namespace std;
using namespace pcl::console;
using namespace pcl::visualization;

int main (int argc, char ** argv)
{
  std::string hdlCalibration, pcapFile;

  int frameSep = 10;
  parse_argument (argc, argv, "-calibrationFile", hdlCalibration);
  parse_argument (argc, argv, "-pcapFile", pcapFile);
  parse_argument (argc, argv, "-sep", frameSep);
  pcl::HDLGrabber grabber (hdlCalibration, pcapFile);

  SimpleHDLViewer v (grabber, frameSep);
  v.run ();
  return (0);
}
