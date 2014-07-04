#include "simplecloudgrabber.h"

/*SimpleCloudGrabber::SimpleCloudGrabber()
{
    SimpleCloudGrabber("#1");
}

SimpleCloudGrabber::SimpleCloudGrabber(const std::string interfaceURL)
{
	grab_mutex_ = new boost::mutex();

    boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
            boost::bind (&SimpleCloudGrabber::cloudCallback, this, _1);

	interface_ = new pcl::io::OpenNI2Grabber(interfaceURL);

    boost::signals2::connection cloud_connection = interface_->registerCallback (f);

    grab_stop_ = new bool(false);
    is_cloud_ = new bool(false);
    file_ = false;

    cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
}*/

SimpleCloudGrabber::SimpleCloudGrabber(pcl::io::OpenNI2Grabber * grabber)
	: interface_(grabber)
{
	grab_mutex_ = new boost::mutex();

    boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
            boost::bind (&SimpleCloudGrabber::cloudCallback, this, _1);

    boost::signals2::connection cloud_connection = interface_->registerCallback (f);

    grab_stop_ = new bool(false);
    is_cloud_ = new bool(false);
    file_ = false;

    cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
}

SimpleCloudGrabber::~SimpleCloudGrabber()
{
    delete grab_stop_;
    delete is_cloud_;
	delete grab_mutex_;
}

void SimpleCloudGrabber::operator ()()
{
	std::cout << "grabber start...";
	interface_->start();

	std::cout << " ok" << std::endl;

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
	std::cout << "new cloud" << std::endl;
	if (*is_cloud_)
		return;
	std::cout << "new cloud" << std::endl;
    
    boost::lock_guard<boost::mutex> guard(*grab_mutex_);
    copyCloud(cloud, cloud_);

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
	for (int k = 0; k < source->points.size(); k++)
    {
		pcl::PointXYZ pt = source->points[k];
        dest->points.push_back(pt);
    }
    dest->width = dest->points.size();
    dest->height = 1;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SimpleCloudGrabber::getCloud()
{
    boost::lock_guard<boost::mutex> guard(*grab_mutex_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp = cloud_;
    return tmp;
}

