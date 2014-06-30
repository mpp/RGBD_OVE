#include "simplecloudgrabber.h"

SimpleCloudGrabber::SimpleCloudGrabber()
{
    SimpleCloudGrabber("#1");
}

SimpleCloudGrabber::SimpleCloudGrabber(const std::string interfaceURL)
{
    std::shared_ptr<std::mutex> tmp(new std::mutex);
    grab_mutex_ = tmp;

    interface_ = new pcl::OpenNIGrabber(interfaceURL);

    boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
            boost::bind (&SimpleCloudGrabber::cloudCallback, this, _1);

    interface_->registerCallback (f);

    grab_stop_ = new bool(false);
    is_cloud_ = new bool(false);
    file_ = false;

    cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
}

SimpleCloudGrabber::~SimpleCloudGrabber()
{
    delete grab_stop_;
    delete is_cloud_;
    delete interface_;
}

void SimpleCloudGrabber::operator ()()
{
    interface_->start();

    while (!(*grab_stop_))
    {
       boost::this_thread::sleep (boost::posix_time::seconds (1.0));
    }

    std::cout << "grabber end" << std::endl;

    if (interface_->isRunning())
        interface_->stop ();

    return;
}

void SimpleCloudGrabber::cloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    grab_mutex_->lock();
    copyCloud(cloud, cloud_);
    grab_mutex_->unlock();

    if (file_)
    {
        pcl::io::savePCDFileASCII("cloud_.pcd", *cloud);
        file_ = false;
    }
    *is_cloud_ = true;
}

void SimpleCloudGrabber::copyCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &source,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr &dest)
{
    dest->clear();
    for (pcl::PointXYZ pt : source->points)
    {
        dest->points.push_back(pt);
    }
    dest->width = dest->points.size();
    dest->height = 1;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SimpleCloudGrabber::getCloud()
{
    grab_mutex_->lock();
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp = cloud_;
    grab_mutex_->unlock();
    return tmp;
}

