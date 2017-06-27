#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

using namespace std;
using namespace pcl::console;
using namespace pcl::visualization;

class MultiHDLViewer
{
  public:
    typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    MultiHDLViewer(pcl::Grabber& grabber1, pcl::Grabber& grabber2,
        pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler1,
        pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler2) :
        cloud_viewer_ (new pcl::visualization::PCLVisualizer ("PCL HDL Cloud")),
        grabber1_ (grabber1),
        grabber2_ (grabber2),
        handler1_ (handler1),
        handler2_ (handler2)
    {
    }

    void cloud_callback1 (const CloudConstPtr& cloud)
    {
      boost::mutex::scoped_lock lock (cloud_mutex1_);
      cloud1_ = cloud;
    }

    void cloud_callback2 (const CloudConstPtr& cloud)
    {
      boost::mutex::scoped_lock lock (cloud_mutex2_);
      cloud2_ = cloud;
    }

    void run ()
    {
      cloud_viewer_->addCoordinateSystem (3.0);
      cloud_viewer_->setBackgroundColor (0, 0, 0);
      cloud_viewer_->initCameraParameters ();
      cloud_viewer_->setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
      cloud_viewer_->setCameraClipDistances (0.0, 50.0);

      boost::function<void (const CloudConstPtr&)> cloud_cb1 = boost::bind (
          &MultiHDLViewer::cloud_callback1, this, _1);
      boost::signals2::connection cloud_connection1 = grabber1_.registerCallback (
          cloud_cb1);

      boost::function<void (const CloudConstPtr&)> cloud_cb2 = boost::bind (
          &MultiHDLViewer::cloud_callback2, this, _1);
      boost::signals2::connection cloud_connection2 = grabber2_.registerCallback (
          cloud_cb2);

      grabber1_.start ();
      grabber2_.start ();

      while (!cloud_viewer_->wasStopped ())
      {
        CloudConstPtr cloud1;

        // See if we can get a cloud
        if (cloud_mutex1_.try_lock ())
        {
          cloud1_.swap (cloud1);
          cloud_mutex1_.unlock ();
        }

        if (cloud1)
        {
          handler1_.setInputCloud (cloud1);
          if (!cloud_viewer_->updatePointCloud (cloud1, handler1_, "HDL1"))
            cloud_viewer_->addPointCloud (cloud1, handler1_, "HDL1");

          cloud_viewer_->spinOnce ();
        }

        CloudConstPtr cloud2;

        // See if we can get a cloud
        if (cloud_mutex2_.try_lock ())
        {
          cloud2_.swap (cloud2);
          cloud_mutex2_.unlock ();
        }

        if (cloud2)
        {
          handler2_.setInputCloud (cloud2);
          if (!cloud_viewer_->updatePointCloud (cloud2, handler2_, "HDL2"))
            cloud_viewer_->addPointCloud (cloud2, handler2_, "HDL2");

          cloud_viewer_->spinOnce ();
        }
        cloud_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "HDL");
        
        if (!grabber1_.isRunning () || !grabber2_.isRunning ())
          cloud_viewer_->spin ();

        boost::this_thread::sleep (boost::posix_time::microseconds (100));
      }

      grabber1_.stop ();
      grabber2_.stop ();

      cloud_connection1.disconnect ();
      cloud_connection2.disconnect ();
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;

    pcl::Grabber& grabber1_;
    pcl::Grabber& grabber2_;
    boost::mutex cloud_mutex1_;
    boost::mutex cloud_mutex2_;

    CloudConstPtr cloud1_;
    CloudConstPtr cloud2_;
    pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler1_;
    pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler2_;
};

int main (int argc, char ** argv)
{
  std::string hdlCalibration, pcapFile1, pcapFile2;

  parse_argument (argc, argv, "-calibrationFile", hdlCalibration);
  parse_argument (argc, argv, "-pcapFile1", pcapFile1);
  parse_argument (argc, argv, "-pcapFile2", pcapFile2);

  pcl::HDLGrabber grabber1 (hdlCalibration, pcapFile1);
  pcl::HDLGrabber grabber2 (hdlCalibration, pcapFile2);

  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler1 ("intensity");
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler2 ("intensity");

  MultiHDLViewer v (grabber1, grabber2, color_handler1, color_handler2);
  v.run ();
  return (0);
}
