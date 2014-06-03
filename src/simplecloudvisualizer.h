#ifndef SIMPLECLOUDVISUALIZER_H
#define SIMPLECLOUDVISUALIZER_H

#include <boost/thread.hpp>
#include <boost/make_shared.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

class SimpleCloudVisualizer
{
public:
    SimpleCloudVisualizer();

    void operator()();

    void updateCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                     std::string cloudName, int r = 255, int g = 255, int b = 255, int s = 1);

    bool wasStopped() const {return viewer_->wasStopped();}

private:
    bool
        * update_;

    boost::shared_ptr<boost::mutex>
        update_mutex_;

    boost::shared_ptr<pcl::visualization::PCLVisualizer>
        viewer_;
};

#endif // SIMPLECLOUDVISUALIZER_H
