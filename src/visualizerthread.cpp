#include "visualizerthread.h"

VisualizerThread::VisualizerThread()
{
    update_ = new bool(false);

    update_mutex_ = boost::shared_ptr<boost::mutex>(new boost::mutex());

    viewer_ = boost::shared_ptr<pcl::visualization::PCLVisualizer>(
        new pcl::visualization::PCLVisualizer("Object Volume Estimator"));

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

    cloud_complete_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_complete_->points.clear();
    cloud_complete_->points.push_back(pcl::PointXYZ(0,0,0));
    cloud_complete_->width = 1;
    cloud_complete_->height = 1;

    viewer_->createViewPort (0.0, 0.0, 0.5, 0.5, v_0_);
    viewer_->setBackgroundColor (0.2, 0.2, 0.2, v_0_);
    viewer_->addText ("Camera #0", 10, 10, "v0 text", v_0_);
    viewer_->addPointCloud<pcl::PointXYZ> (cloud_0_, "object0", v_0_);
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "object0", v_0_);
    viewer_->addCoordinateSystem (1.0, v_0_);

    viewer_->createViewPort (0.5, 0.0, 1.0, 0.5, v_1_);
    viewer_->setBackgroundColor (0.2, 0.2, 0.2, v_1_);
    viewer_->addText ("Camera #1", 10, 10, "v1 text", v_1_);
    viewer_->addPointCloud<pcl::PointXYZ> (cloud_1_, "object1", v_1_);
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "object1", v_1_);
    viewer_->addCoordinateSystem (1.0, v_1_);

    viewer_->createViewPort (0.0, 0.5, 1.0, 1.0, v_complete_);
    viewer_->setBackgroundColor (0.2, 0.2, 0.2, v_complete_);
    viewer_->addText ("Complete object", 10, 10, "vcomplete text", v_complete_);
    viewer_->addPointCloud<pcl::PointXYZ> (cloud_complete_, "complete", v_complete_);
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "complete", v_complete_);
    viewer_->addCoordinateSystem (1.0, v_complete_);

    viewer_->setBackgroundColor (0.2, 0.2, 0.2);
    viewer_->initCameraParameters ();
    viewer_->setCameraPosition(0.3, 0.2, -0.1, 0.0, -1.0, 0.0);

    bb_ = std::shared_ptr<BoundingBox>(new BoundingBox);
}

void VisualizerThread::updateCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                   const VIEWPORT cloudName)
{
    boost::mutex::scoped_lock updateLock(*(update_mutex_.get()));
    (*update_) = true;

    if (cloudName == COMPLETE)
    {
        copyCloud(cloud, cloud_complete_);
    }
    else if (cloudName == OBJ0)
    {
        copyCloud(cloud, cloud_0_);
    }
    else if (cloudName == OBJ1)
    {
        copyCloud(cloud, cloud_1_);
    }

    //std::cout << cloud_0_->points.size() << " - " << cloud_1_->points.size() << " - " << ((*update_)?"true":"false") << std::endl;

    updateLock.unlock();
}

void VisualizerThread::copyCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &source,
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

void VisualizerThread::updateBB(const BoundingBox &bb)
{
    bb_->max_pt = bb.max_pt;
    bb_->min_pt = bb.min_pt;
    bb_->qfinal = bb.qfinal;
    bb_->tfinal = bb.tfinal;
    //std::cout << "update bb" << bb_->max_pt.x << std::endl;
}

void VisualizerThread::operator()()
{
    // prepare visualizer named "viewer"
    while (!viewer_->wasStopped ())
    {
        viewer_->spinOnce (100);

        // Get lock on the boolean update and check if cloud was updated
        boost::mutex::scoped_lock updateLock(*(update_mutex_.get()));
        if((*update_))
        {
            //std::cout << "updating" << std::endl;
            //std::cout << cloud_0_->points.size() << " - " << cloud_1_->points.size() << " - " << ((*update_)?"true":"false") << std::endl;
            //std::cout << v_0_ << " - " << v_1_ << std::endl;

            viewer_->removeAllShapes(v_0_);
            viewer_->addText ("Camera #0", 10, 10, "v0 text", v_0_);
            if(!viewer_->updatePointCloud(cloud_0_, "object0"))
            {
                viewer_->addPointCloud<pcl::PointXYZ>(cloud_0_, "object0", v_0_);
                viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "object0", v_0_);
            }

            viewer_->removeAllShapes(v_1_);
            viewer_->addText ("Camera #1", 10, 10, "v1 text", v_1_);
            if(!viewer_->updatePointCloud(cloud_1_, "object1"))
            {
                viewer_->addPointCloud<pcl::PointXYZ>(cloud_1_, "object1", v_1_);
                viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "object1", v_1_);
            }

            viewer_->removeAllShapes(v_complete_);
            //std::cout << "try to add bb -> " << bb_->max_pt.x << std::endl;
            if (bb_)
            {
                //std::cout << "add bb" << std::endl;
                //viewer_->addCube(bb_->tfinal, bb_->qfinal,
                //            bb_->max_pt.x - bb_->min_pt.x,
                //            bb_->max_pt.y - bb_->min_pt.y,
                //            bb_->max_pt.z - bb_->min_pt.z,
                //            std::to_string(v_complete_), v_complete_);
                viewer_->addCube(bb_->min_pt.x, bb_->max_pt.x,
                                 bb_->min_pt.y, bb_->max_pt.y,
                                 bb_->min_pt.z, bb_->max_pt.z,
                                 1.0,1.0,1.0,
                                 std::to_string(v_complete_), v_complete_);
            }
            viewer_->addText ("Camera complete", 10, 10, "v complete text", v_complete_);

            if(!viewer_->updatePointCloud(cloud_complete_, "complete"))
            {
                viewer_->addPointCloud<pcl::PointXYZ>(cloud_complete_, "complete", v_complete_);
                viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "complete", v_complete_);
            }

            (*update_) = false;
        }
        updateLock.unlock();

    }
}
