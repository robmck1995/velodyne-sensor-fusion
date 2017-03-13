#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

using namespace std;
using namespace pcl::console;
using namespace pcl::visualization;

class SimpleHDLViewer
{
  public:
    typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
    typedef pcl::PointCloud<pcl::PointXYZRGB> RGBCloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    SimpleHDLViewer(pcl::Grabber& grabber,
        pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler) :
        cloud_viewer_ (new pcl::visualization::PCLVisualizer ("PCL HDL Cloud")),
        grabber_ (grabber),
        handler_ (handler)
    {
    }

    void cloud_callback (const CloudConstPtr& cloud)
    {
      boost::mutex::scoped_lock lock (cloud_mutex_);
      cloud_ = cloud;
    }

    void run ()
    {
      cloud_viewer_->addCoordinateSystem (3.0);
      cloud_viewer_->setBackgroundColor (0, 0, 0);
      cloud_viewer_->initCameraParameters ();
      cloud_viewer_->setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
      cloud_viewer_->setCameraClipDistances (0.0, 50.0);

      boost::function<void (const CloudConstPtr&)> cloud_cb = boost::bind (
          &SimpleHDLViewer::cloud_callback, this, _1);
      boost::signals2::connection cloud_connection = grabber_.registerCallback (
          cloud_cb);

      grabber_.start ();

      int cloudsAdded = 0;
      int colors[5][3] = {{255,0,0},{0,255,0},{0,0,255},{255,255,0},{255,0,255}};
      std::vector<CloudConstPtr> cloudArr;
      while (!cloud_viewer_->wasStopped ())
      {
        CloudConstPtr cloud;
	CloudConstPtr prevCloud;

        if(cloudsAdded >= 5) {
          break;
        }
        // See if we can get a cloud
        if (cloud_mutex_.try_lock ())
        {
          cloud_.swap (cloud);
          cloud_mutex_.unlock ();
        }

        if (cloud)
        {
          handler_.setInputCloud (cloud);
          if (cloud != prevCloud) {
            std::cout << "Gonna Copy the Cloud" << std::endl;
            std::ostringstream id;
            id << "HDL" << cloudsAdded;
            //pcl::copyPointCloud(*(pcl::PointCloud<pcl::PointXYZI>::ConstPtr) cloud, *newCloud);
            cloud_viewer_->addPointCloud (cloud, handler_, id.str());
            std::cout << "Added Point Cloud" << std::endl;
            cloudArr.push_back(cloud);
	    std::cout << "Saved Cloud" << std::endl;
          }

          /*
          if (cloud != prevCloud) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::copyPointCloud(*(pcl::PointCloud<pcl::PointXYZI>::ConstPtr &) cloud, *newCloud);
            int r = colors[cloudsAdded][0];
            int g = colors[cloudsAdded][1];
            int b = colors[cloudsAdded][2];
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb (newCloud, r, g, b); //This will display the point cloud in red (R,G,B)
            std::ostringstream id;
            id << "cloud " << cloudsAdded;
            cloud_viewer_->addPointCloud<pcl::PointXYZRGB> (newCloud, rgb, id.str());
	    cloudArr[cloudsAdded] = newCloud;
            cloud_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id.str());
	    cloudsAdded++;
          }
          */
          cloudsAdded++;
          prevCloud = cloud;
          cloud_viewer_->spinOnce ();
        }

        for(int i = 0; i < cloudArr.size(); i++) {
          RGBCloud::ConstPtr colorCloud (new RGBCloud);
        }

        if (!grabber_.isRunning ())
          cloud_viewer_->spin ();

	//prevCloud = cloud;
        boost::this_thread::sleep (boost::posix_time::microseconds (100));
      }

      grabber_.stop ();

      while (!cloud_viewer_->wasStopped ()) {
        cloud_viewer_->spinOnce(100);
      }
      cloud_connection.disconnect ();
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;

    pcl::Grabber& grabber_;
    boost::mutex cloud_mutex_;

    CloudConstPtr cloud_;
    pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler_;
};

int main (int argc, char ** argv)
{
  std::string hdlCalibration, pcapFile;

  parse_argument (argc, argv, "-calibrationFile", hdlCalibration);
  parse_argument (argc, argv, "-pcapFile", pcapFile);

  pcl::HDLGrabber grabber (hdlCalibration, pcapFile);

  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler ("intensity");

  SimpleHDLViewer v (grabber, color_handler);
  v.run ();
  return (0);
}
