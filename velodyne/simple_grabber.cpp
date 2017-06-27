#include "simple_grabber.h"
    
SimpleHDLViewer::SimpleHDLViewer(pcl::Grabber& grabber, int frameSep):
    cloud_viewer_ (new pcl::visualization::PCLVisualizer ("PCL HDL Cloud")),
    grabber_ (grabber),
    file_num (0),
    frames(0),
    frameInt(frameSep)
{
}

void SimpleHDLViewer::cloud_callback (const CloudConstPtr& cloud)
{
  boost::shared_lock<boost::shared_mutex>  lock (cloud_mutex_);
  cloud_ = cloud;
  std::ostringstream fname;
  fname << "cloud" << file_num++ << ".pcd";
  if(frames % frameInt == 0) {
    pcl::io::savePCDFile( fname.str(), *cloud, true );
  }
  frames++;
}

void SimpleHDLViewer::run ()
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
      cout << "Got Cloud" << endl;
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> handler(cloud,colors[cloudsAdded][0],colors[cloudsAdded][1],colors[cloudsAdded][2]);
      if (cloud != prevCloud) {
        if(frames % frameInt == 1) {
          std::ostringstream id;
          id << "HDL" << cloudsAdded;
          
          cloud_viewer_->addPointCloud (cloud, handler, id.str());
          std::cout << "Added Point Cloud" << std::endl;
          cloudsAdded++;
        }
      }
      prevCloud = cloud;
      cloud_viewer_->spinOnce ();
    }

    if (!grabber_.isRunning ())
      cloud_viewer_->spin ();

    boost::this_thread::sleep (boost::posix_time::microseconds (100));
  }

  grabber_.stop ();

  while (!cloud_viewer_->wasStopped ()) {
    cloud_viewer_->spinOnce(100);
  }
  cloud_connection.disconnect ();
}
