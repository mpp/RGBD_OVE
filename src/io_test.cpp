/*!

  PREREQUISITES:
    * a set of fully calibrated sensors
    * known 6DOF poses of sensors relative to a fixed position (as the center of the evaluation table)

  STEPs:
    * scene grab -> or 1 shot or take some of them and average or similar
    * scene filter (MAX - MIN)
    * statistical outlier removal
    * if more than one sensor -> registration
    * segmentation
    * bounding box / volume computation

  */

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

class SimpleOpenNIViewer
{
public:
    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
    {
        if (!viewer.wasStopped())
            viewer.showCloud (cloud);
    }
    void cloud_cb_rgba_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
        if (!viewer.wasStopped())
            viewer.showCloud (cloud);
    }

    void run ()
    {
        pcl::Grabber* interface = new pcl::OpenNIGrabber();

        boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
        boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

        boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> g =
        boost::bind (&SimpleOpenNIViewer::cloud_cb_rgba_, this, _1);

        interface->registerCallback (g);

        interface->start ();

        while (!viewer.wasStopped())
        {
            boost::this_thread::sleep (boost::posix_time::seconds (1));
        }

        interface->stop ();
    }

    pcl::visualization::CloudViewer viewer;
};

int main ()
{
    SimpleOpenNIViewer v;
    v.run ();
    return 0;
}
