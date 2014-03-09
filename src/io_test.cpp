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
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#define MIN_X -0.30f
#define MAX_X 0.30f
#define MIN_Y 0.0f
#define MAX_Y 2.0f
#define MIN_Z 0.0f
#define MAX_Z 1.5f

void passTroughFilterXYZ(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr
            filteredCloud_x (new pcl::PointCloud<pcl::PointXYZ>),
            filteredCloud_y (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> passTroughFilter;

    // PassThrough x
    passTroughFilter.setInputCloud(cloud);
    passTroughFilter.setFilterFieldName("x");
    passTroughFilter.setFilterLimits(MIN_X, MAX_X);
    passTroughFilter.filter(*filteredCloud_x);

    // PassThrough y
    passTroughFilter.setInputCloud(filteredCloud_x);
    passTroughFilter.setFilterFieldName("y");
    passTroughFilter.setFilterLimits(MIN_Y, MAX_Y);
    passTroughFilter.filter(*filteredCloud_y);

    // PassThrough z
    passTroughFilter.setInputCloud(filteredCloud_y);
    passTroughFilter.setFilterFieldName("z");
    passTroughFilter.setFilterLimits(MIN_Z, MAX_Z);
    passTroughFilter.filter(*filtered);
}

class SimpleOpenNIViewer
{
public:
    SimpleOpenNIViewer () : viewer_ ("PCL OpenNI Viewer")
    {
        cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

        cloud_->points.clear();
        cloud_->points.push_back(pcl::PointXYZ(0,0,0));
        cloud_->width = 1;
        cloud_->height = 1;

        viewer_.setBackgroundColor (0.2, 0.2, 0.2);
        viewer_.addPointCloud<pcl::PointXYZ> (cloud_, "object");
        viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "object");
        viewer_.addCoordinateSystem (1.0, 0);
        viewer_.initCameraParameters ();
    }

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
    {
        /// 1 - Scene filter (MAX-MIN)
        pcl::PointCloud<pcl::PointXYZ>::Ptr
                filteredCloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
        passTroughFilterXYZ(cloud, filteredCloud_xyz);

        /// 2 - Statistical Outliers Removal
        pcl::PointCloud<pcl::PointXYZ>::Ptr noOutliersCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (filteredCloud_xyz);
        sor.setMeanK (25);
        sor.setStddevMulThresh (1.5);
        sor.filter (*noOutliersCloud);

        /// 3 - registration...

        /// 4 - Segmentation
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
        /*pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segmentedCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
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
        }*/

        // The vector seems already sort, just in case...
        std::sort(inliers->indices.begin(), inliers->indices.end());

        pcl::PointCloud<pcl::PointXYZ>::Ptr segmentedCloud (new pcl::PointCloud<pcl::PointXYZ>);
        int currentInlier = 0;
        for (int i = 0; i < noOutliersCloud->points.size(); i++)
        {
            if (inliers->indices[currentInlier] != i)
            {
                segmentedCloud->points.push_back(noOutliersCloud->points[i]);
            }
            else
            {
                currentInlier = currentInlier + 1;
            }
        }
        segmentedCloud->width = (int) segmentedCloud->points.size();
        segmentedCloud->height = 1;

        /// 5 - Outliers Removal
        cloud_->points.clear();
        sor.setInputCloud (segmentedCloud);
        sor.setMeanK (25);
        sor.setStddevMulThresh (1.5);
        sor.filter (*cloud_);
        cloud_->width = (int) cloud_->points.size();
        cloud_->height = 1;

        /// 6 - Bounding-Box computation

        // compute principal direction
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_, centroid);
        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(*cloud_, centroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
        eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

        // move the points to the that reference frame
        Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
        p2w.block<3,3>(0,0) = eigDx.transpose();
        p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
        pcl::PointCloud<pcl::PointXYZ> cPoints;
        pcl::transformPointCloud(*cloud_, cPoints, p2w);

        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(cPoints, min_pt, max_pt);
        const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

        // final transform
        const Eigen::Quaternionf qfinal(eigDx);
        const Eigen::Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();

        if (!viewer_.wasStopped())
        {
            viewer_.updatePointCloud(cloud_, "object");
            viewer_.removeAllShapes();
            viewer_.addCube(tfinal, qfinal, max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z);
            viewer_.spinOnce(35, true);
        }
    }

    void run ()
    {
        pcl::Grabber* interface = new pcl::OpenNIGrabber();

        boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
        boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

        interface->registerCallback (f);

        interface->start ();

        while (!viewer_.wasStopped())
        {
            boost::this_thread::sleep (boost::posix_time::seconds (1));
        }

        interface->stop ();
    }

    pcl::visualization::PCLVisualizer viewer_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
};

int main ()
{
    SimpleOpenNIViewer v;
    v.run ();
}
