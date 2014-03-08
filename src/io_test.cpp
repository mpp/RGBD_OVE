/*!

  PREREQUISITES:
    * a set of fully calibrated sensors
    * known 6DOF poses of sensors relative to a fixed position (as the center of the evaluation table)

  STEPs:
    * scene grab -> or 1 shot or take some of them and average or similar
    * scene filter (MAX - MIN)
    * statistical outlier removal
    * if more than one sensor -> registration
    * segmentation
    * bounding box / volume computation

  */

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


class SimpleOpenNIViewer
{
public:
    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
    {
        // 1 - Scene filter (MAX-MIN)
        pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> passTroughFilter;
        passTroughFilter.setInputCloud(cloud);
        passTroughFilter.setFilterFieldName("x");
        passTroughFilter.setFilterLimits(-2.0f, 2.0f);
        passTroughFilter.setFilterFieldName("y");
        passTroughFilter.setFilterLimits(0.8f, 2.0f);
        passTroughFilter.setFilterFieldName("z");
        passTroughFilter.setFilterLimits(-2.0f, 2.0f);
        passTroughFilter.filter(*filteredCloud);

        // 2 - Statistical Outliers Removal
        pcl::PointCloud<pcl::PointXYZ>::Ptr noOutliersCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (filteredCloud);
        sor.setMeanK (25);
        sor.setStddevMulThresh (1.5);
        sor.filter (*noOutliersCloud);

        // 3 - registration...

        // 4 - Segmentation
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);

        seg.setInputCloud (noOutliersCloud);
        seg.segment (*inliers, *coefficients);

        // Color the inliers for visual debug
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segmentedCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
        for (pcl::PointXYZ pt : noOutliersCloud->points)
        {
            pcl::PointXYZRGBA ptrgba;
            ptrgba.x = pt.x;
            ptrgba.y = pt.y;
            ptrgba.z = pt.z;
            ptrgba.r = 255;
            ptrgba.g = 33;
            ptrgba.b = 33;
            ptrgba.a = 255;
            segmentedCloud->points.push_back(ptrgba);
        }
        segmentedCloud->width = (int) segmentedCloud->points.size();
        segmentedCloud->height = 1;

        for (int i : inliers->indices)
        {
            segmentedCloud->points[i].r = 33;
            segmentedCloud->points[i].g = 255;
        }

        if (!viewer.wasStopped())
            viewer.showCloud (segmentedCloud);
    }

    void run ()
    {
        pcl::Grabber* interface = new pcl::OpenNIGrabber();

        boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
        boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

        interface->registerCallback (f);

        interface->start ();

        while (!viewer.wasStopped())
        {
            boost::this_thread::sleep (boost::posix_time::seconds (1));
        }

        interface->stop ();
    }

    pcl::visualization::CloudViewer viewer;
};

int main ()
{
    SimpleOpenNIViewer v;
    v.run ();
}
