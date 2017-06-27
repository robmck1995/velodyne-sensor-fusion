#ifndef _SIMPLE_GRABBER
#define _SIMPLE_GRABBER

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

using namespace std;
using namespace pcl::console;
using namespace pcl::visualization;
class SimpleHDLViewer
{
  public:
    typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    SimpleHDLViewer(pcl::Grabber& grabber, int frameSep);

    void cloud_callback (const CloudConstPtr& cloud);
    
    void save_cloud_callback (const CloudConstPtr& cloud);

    void run ();

    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;

    pcl::Grabber& grabber_;
    boost::shared_mutex cloud_mutex_;
    CloudConstPtr cloud_;
    int file_num;
    int frames;
    int frameInt;
};

#endif
