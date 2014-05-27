#include "cloudsgrabber.h"

CloudsGrabber::CloudsGrabber()
{
    std::shared_ptr<std::mutex> tmp(new std::mutex);
    grab_mutex_ = tmp;

    interface_0_ = new pcl::OpenNIGrabber("#1");
    interface_1_ = new pcl::OpenNIGrabber("#2");

    boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f0 =
            boost::bind (&CloudsGrabber::cloudCallback0, this, _1);
    boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f1 =
            boost::bind (&CloudsGrabber::cloudCallback1, this, _1);

    interface_0_->registerCallback (f0);
    interface_1_->registerCallback (f1);

    cloud_0_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_0_->points.clear();
    cloud_0_->points.push_back(pcl::PointXYZ(0,0,0));
    cloud_0_->width = 1;
    cloud_0_->height = 1;

    cloud_1_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_1_->points.clear();
    cloud_1_->points.push_back(pcl::PointXYZ(0,0,0));
    cloud_1_->width = 1;
    cloud_1_->height = 1;

    grab_stop_ = new bool(false);
    file0_ = false;
    file1_ = false;
}

void CloudsGrabber::operator ()()
{
    interface_0_->start();

    while (!(*grab_stop_))//grab_counter_ <= 10)
    {
        /*if (grab_counter_ & 1)
        {
            startAndStop(*interface_0_, *interface_1_);
        }
        else
        {
            startAndStop(*interface_1_, *interface_0_);
        }*/

        //std::cout << ((*grab_stop_)?"true":"false") << std::endl;

        boost::this_thread::sleep (boost::posix_time::seconds (0.5));

        //grab_counter_ += 1;
    }

    std::cout << "grabber end" << std::endl;

    if (interface_0_->isRunning())
        interface_0_->stop ();
    if (interface_1_->isRunning())
        interface_1_->stop ();

    return;
}

void CloudsGrabber::startAndStop(pcl::Grabber &toStart, pcl::Grabber &toStop)
{
    if (!toStart.isRunning())
        toStart.start();
    if (toStop.isRunning())
        toStop.stop();
}

void CloudsGrabber::cloudCallback0(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    //std::cout << "cb0" << std::endl;
    grab_mutex_->lock();
    //std::cout << "lock0" << std::endl;
    if (interface_1_->isRunning())
        interface_1_->stop();
    if (!interface_0_->isRunning())
        interface_0_->start();
    copyCloud(cloud, cloud_0_);
    interface_0_->stop();
    interface_1_->start();
    grab_mutex_->unlock();
    //std::cout << "unlock0" << std::endl;

    if (!file0_)
    {
        pcl::io::savePCDFileASCII("cloud_0.pcd", *cloud);
        file0_ = true;
    }
}

void CloudsGrabber::cloudCallback1(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    //std::cout << "cb1" << std::endl;
    grab_mutex_->lock();
    if (interface_0_->isRunning())
        interface_0_->stop();
    if (!interface_1_->isRunning())
        interface_1_->start();
    //std::cout << "lock1" << std::endl;
    copyCloud(cloud, cloud_1_);
    interface_1_->stop();
    interface_0_->start();
    grab_mutex_->unlock();
    //std::cout << "unlock1" << std::endl;

    if (!file1_)
    {
        pcl::io::savePCDFileASCII("cloud_1.pcd", *cloud);
        file1_ = true;
    }
}

void CloudsGrabber::copyCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &source,
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

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudsGrabber::getCloud0()
{
    grab_mutex_->lock();
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp = cloud_0_;
    grab_mutex_->unlock();
    return tmp;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudsGrabber::getCloud1()
{
    grab_mutex_->lock();
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp = cloud_1_;
    grab_mutex_->unlock();
    return tmp;
}
