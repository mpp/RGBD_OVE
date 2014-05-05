#ifndef CLOUDSGRABBER_H
#define CLOUDSGRABBER_H

#include <boost/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/openni_grabber.h>
#include <mutex>

class CloudsGrabber
{
public:
    CloudsGrabber();

    void operator()();

    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud0();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud1();

    void stopGrabber() {(*grab_stop_) = true;}

private:
    void cloudCallback0 (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

    void cloudCallback1 (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

    void startAndStop (pcl::Grabber &toStart, pcl::Grabber &toStop);

    void copyCloud (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &source, pcl::PointCloud<pcl::PointXYZ>::Ptr &dest);

private:
    std::shared_ptr<std::mutex>
        grab_mutex_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr
        cloud_0_,
        cloud_1_;

    pcl::Grabber
        * interface_0_,
        * interface_1_;

    bool
        * grab_stop_;
};

#endif // CLOUDSGRABBER_H
