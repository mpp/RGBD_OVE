#include "simplecloudvisualizer.h"

SimpleCloudVisualizer::SimpleCloudVisualizer()
{
    viewer_ = boost::shared_ptr<pcl::visualization::PCLVisualizer>(
        new pcl::visualization::PCLVisualizer("Object Volume Estimator"));

    viewer_->setBackgroundColor (0.2, 0.2, 0.2);
    viewer_->addCoordinateSystem (1.0);

    viewer_->setBackgroundColor (0.2, 0.2, 0.2);
    viewer_->initCameraParameters ();
    viewer_->setCameraPosition(0.3, 0.2, -0.1, 0.0, -1.0, 0.0);
}

void SimpleCloudVisualizer::updateCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                        std::string cloudName, int r, int g, int b, int s)
{
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, r, g, b);
    if(!viewer_->updatePointCloud(cloud, single_color, cloudName))
    {
        viewer_->addPointCloud<pcl::PointXYZ>(cloud, single_color, cloudName);
        viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, s, cloudName);
    }
}

void SimpleCloudVisualizer::operator()()
{
    // prepare visualizer named "viewer"
    while (!viewer_->wasStopped ())
    {
        viewer_->spinOnce (100);
    }
}
