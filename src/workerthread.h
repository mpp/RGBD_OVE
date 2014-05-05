#ifndef WORKERTHREAD_H
#define WORKERTHREAD_H

#include "visualizerthread.h"

class WorkerThread
{
public:
    WorkerThread();

    void operator()();

private:
    bool
        update_;

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

    boost::shared_ptr<BoundingBox>
        bb_;
};

#endif // WORKERTHREAD_H
