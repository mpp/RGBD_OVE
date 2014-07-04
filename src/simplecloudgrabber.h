#ifndef SIMPLECLOUDGRABBER_H
#define SIMPLECLOUDGRABBER_H

#include <boost/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/io/openni_grabber.h>
//#include <openni2/OpenNI.h>
//#include <mutex>
#include <pcl/visualization/boost.h>
#include <pcl/io/openni2_grabber.h>

class SimpleCloudGrabber
{
public:
    /*SimpleCloudGrabber();
    SimpleCloudGrabber(const std::string interfaceURL);*/
    SimpleCloudGrabber(pcl::io::OpenNI2Grabber * grabber);

    ~SimpleCloudGrabber();

    void operator()();

    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud();

    void stopGrabber() {(*grab_stop_) = true;}

    bool isCloud() {return *is_cloud_;}
	
private:
    void cloudCallback (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);
	
    void copyCloud (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &source,
		pcl::PointCloud<pcl::PointXYZ>::Ptr &dest);

private:

	boost::mutex
        * grab_mutex_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr
        cloud_;

    pcl::io::OpenNI2Grabber
        * interface_;

    bool
        * grab_stop_;

    bool
        * is_cloud_;

    bool
        file_;
};

#endif // SIMPLECLOUDGRABBER_H
