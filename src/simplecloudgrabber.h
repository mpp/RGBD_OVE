#ifndef SIMPLECLOUDGRABBER_H
#define SIMPLECLOUDGRABBER_H

#include <boost/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>
#include <openni2/OpenNI.h>
#include <mutex>

class SimpleCloudGrabber
{
public:
    SimpleCloudGrabber();

    void operator()();

    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud();

    void stopGrabber() {(*grab_stop_) = true;}


private:
    void cloudCallback (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

    void copyCloud (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &source, pcl::PointCloud<pcl::PointXYZ>::Ptr &dest);

private:
    std::shared_ptr<std::mutex>
        grab_mutex_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr
        cloud_;

    pcl::Grabber
        * interface_;

    bool
        * grab_stop_;

    bool
        file_;
};

#endif // SIMPLECLOUDGRABBER_H
