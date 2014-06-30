#ifndef VISUALIZERTHREAD_H
#define VISUALIZERTHREAD_H

#include <boost/thread.hpp>
#include <boost/make_shared.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "cloudinteraction.h"

class VisualizerThread
{
public:

    enum VIEWPORT
    {
        OBJ0 = 0,
        OBJ1,
        COMPLETE
    };

    VisualizerThread();

    void operator()();

    void updateCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                     const VIEWPORT cloudName);

    void updateBB(const BoundingBox &bb);

    bool wasStopped() {return viewer_->wasStopped();}

    /*void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                                void* viewer_void)*/

private:
    void copyCloud (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &source, pcl::PointCloud<pcl::PointXYZ>::Ptr &dest);

private:
    bool
        * update_;

    boost::shared_ptr<boost::mutex>
        update_mutex_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr
        cloud_0_,
        cloud_1_,
        cloud_complete_;

    boost::shared_ptr<pcl::visualization::PCLVisualizer>
        viewer_;

    int
        v_0_,
        v_1_,
        v_complete_;

    std::shared_ptr<BoundingBox>
        bb_;
};


#endif // VISUALIZERTHREAD_H

