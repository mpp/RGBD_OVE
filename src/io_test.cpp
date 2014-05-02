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

#include <boost/make_shared.hpp>
#include <pcl/point_representation.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <mutex>

#define MIN_X -0.30f
#define MAX_X 0.30f
#define MIN_Y 0.0f
#define MAX_Y 2.0f
#define MIN_Z 0.0f
#define MAX_Z 1.5f

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

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

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
void pairAlign (const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_src,
                const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_tgt,
                pcl::PointCloud<pcl::PointXYZ>::Ptr &output,
                Eigen::Matrix4f &final_transform,
                bool downsample = false)
{
    // Downsample for consistency and speed
    // \note enable this for large datasets
    pcl::PointCloud<pcl::PointXYZ>::Ptr src (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<PointT> grid;
    if (downsample)
    {
        grid.setLeafSize (0.05, 0.05, 0.05);
        grid.setInputCloud (cloud_src);
        grid.filter (*src);

        grid.setInputCloud (cloud_tgt);
        grid.filter (*tgt);
    }
    else
    {
        src = cloud_src;
        tgt = cloud_tgt;
    }

    // Compute surface normals and curvature
    PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

    pcl::NormalEstimation<PointT, PointNormalT> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    norm_est.setSearchMethod (tree);
    norm_est.setKSearch (30);

    norm_est.setInputCloud (src);
    norm_est.compute (*points_with_normals_src);
    pcl::copyPointCloud (*src, *points_with_normals_src);

    norm_est.setInputCloud (tgt);
    norm_est.compute (*points_with_normals_tgt);
    pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

    //
    // Align
    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;

    reg.setTransformationEpsilon (1e-6);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance (0.005);

    reg.setInputSource (points_with_normals_src);
    reg.setInputTarget (points_with_normals_tgt);

    reg.setMaximumIterations (25);
    reg.align (*reg_result);

    // Get the transformation from target to source
    Eigen::Matrix4f targetToSource = reg.getFinalTransformation();

    //
    // Transform target back in source frame
    pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

    //add the source to the transformed target
    *output += *cloud_src;

    final_transform = targetToSource;
}

class SimpleOpenNIViewer
{
public:
    SimpleOpenNIViewer () : viewer_ ("PCL OpenNI Viewer")
    {
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

        viewer_.createViewPort (0.0, 0.0, 0.5, 0.5, v_0_);
        viewer_.setBackgroundColor (0.2, 0.2, 0.2, v_0_);
        viewer_.addText ("Camera #0", 10, 10, "v0 text", v_0_);
        viewer_.addPointCloud<pcl::PointXYZ> (cloud_0_, "object0", v_0_);
        viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "object0", v_0_);
        viewer_.addCoordinateSystem (1.0, v_0_);

        viewer_.createViewPort (0.5, 0.0, 1.0, 0.5, v_1_);
        viewer_.setBackgroundColor (0.2, 0.2, 0.2, v_1_);
        viewer_.addText ("Camera #1", 10, 10, "v1 text", v_1_);
        viewer_.addPointCloud<pcl::PointXYZ> (cloud_1_, "object1", v_1_);
        viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "object1", v_1_);
        viewer_.addCoordinateSystem (1.0, v_1_);

        viewer_.createViewPort (0.0, 0.5, 1.0, 1.0, v_complete_);
        viewer_.setBackgroundColor (0.2, 0.2, 0.2, v_complete_);
        viewer_.addText ("Complete object", 10, 10, "vcomplete text", v_complete_);
        viewer_.addPointCloud<pcl::PointXYZ> (cloud_complete_, "complete", v_complete_);
        viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "complete", v_complete_);
        viewer_.addCoordinateSystem (1.0, v_complete_);

//        std::cout << v_0_ << " - " << v_1_ << std::endl;

        viewer_.setBackgroundColor (0.2, 0.2, 0.2);
//        viewer_.addPointCloud<pcl::PointXYZ> (cloud_, "object");
//        viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "object");
        viewer_.initCameraParameters ();

        viewer_.setCameraPosition(0.3, 0.2, -0.1, 0.0, -1.0, 0.0);
    }

    void cloud_cb_0_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
    {
        mtx.lock();
        std::cout << "lock0" << std::endl;
        if (interface_1_->isRunning())
            interface_1_->stop();
        if (!interface_0_->isRunning())
            interface_0_->start();
        cloud_cb_(cloud, 0);
        interface_0_->stop();
        interface_1_->start();
        mtx.unlock();
        std::cout << "unlock0" << std::endl;
    }

    void cloud_cb_1_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
    {
        mtx.lock();
        std::cout << "lock1" << std::endl;
        if (interface_0_->isRunning())
            interface_0_->stop();
        if (!interface_1_->isRunning())
            interface_1_->start();
        cloud_cb_(cloud, 1);
        interface_1_->stop();
        interface_0_->start();
        mtx.unlock();
        std::cout << "unlock1" << std::endl;
    }

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, const int cameraID)
    {
        //PCL_INFO("1 - Scene filter (MAX-MIN)");
        /// 1 - Scene filter (MAX-MIN)
        pcl::PointCloud<pcl::PointXYZ>::Ptr
                filteredCloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
        passTroughFilterXYZ(cloud, filteredCloud_xyz);

        //PCL_INFO("2 - Statistical Outliers Removal");
        /// 2 - Statistical Outliers Removal
        pcl::PointCloud<pcl::PointXYZ>::Ptr noOutliersCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (filteredCloud_xyz);
        sor.setMeanK (25);
        sor.setStddevMulThresh (1.5);
        sor.filter (*noOutliersCloud);

        //PCL_INFO("4 - Segmentation");
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

        //PCL_INFO("5 - Outliers Removal");
        /// 5 - Outliers Removal
        pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZ>);
        sor.setInputCloud (segmentedCloud);
        sor.setMeanK (25);
        sor.setStddevMulThresh (1.5);
        sor.filter (*finalCloud);
        finalCloud->width = (int) finalCloud->points.size();
        finalCloud->height = 1;

        int viewport = 0;
        if (cameraID == 0)
        {
            pcl::copyPointCloud<pcl::PointXYZ, pcl::PointXYZ>(*finalCloud, *cloud_0_);

            finalCloud = cloud_0_;
            std::cout << "#0 - " << cameraID << std::endl;
            viewport = v_0_;
        }
        else
        {
            pcl::copyPointCloud<pcl::PointXYZ, pcl::PointXYZ>(*finalCloud, *cloud_1_);

            finalCloud = cloud_1_;
            std::cout << "#1 - " << cameraID << std::endl;
            viewport = v_1_;
        }


        if (cloud_0_->points.size() > 1 && cloud_1_->points.size() > 1)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr
                    registered (new pcl::PointCloud<pcl::PointXYZ>);
            //PCL_INFO("3 - registration");
            /// 3 - registration...
            Eigen::Matrix4f GlobalTransform;
            pairAlign(cloud_0_, cloud_1_, registered, GlobalTransform);


            //PCL_INFO("6 - Bounding-Box computation");
            /// 6 - Bounding-Box computation
            // compute principal direction
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*registered, centroid);
            Eigen::Matrix3f covariance;
            pcl::computeCovarianceMatrixNormalized(*registered, centroid, covariance);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
            Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
            eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

            // move the points to that reference frame
            Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
            p2w.block<3,3>(0,0) = eigDx.transpose();
            p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
            pcl::PointCloud<pcl::PointXYZ> cPoints;
            pcl::transformPointCloud(*registered, cPoints, p2w);

            pcl::PointXYZ min_pt, max_pt;
            pcl::getMinMax3D(cPoints, min_pt, max_pt);
            const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

            // final transform
            const Eigen::Quaternionf qfinal(eigDx);
            const Eigen::Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();

            if (!viewer_.wasStopped())
            {
                viewer_.updatePointCloud(registered, "complete");
                viewer_.removeAllShapes(v_complete_);
                viewer_.addText ("Camera complete", 10, 10, "v complete text", v_complete_);
                viewer_.addCube(tfinal, qfinal, max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z, std::to_string(cameraID), v_complete_);
                viewer_.spinOnce(35, true);

                std::cout << "(x, y, z) = (" << max_pt.x - min_pt.x << ", " << max_pt.y - min_pt.y << ", " << max_pt.z - min_pt.z << ")" << std::endl;
            }
        }

        if (!viewer_.wasStopped())
        {
            viewer_.updatePointCloud(finalCloud, "object"+std::to_string(cameraID));
            viewer_.removeAllShapes(viewport);
            viewer_.addText ("Camera #"+std::to_string(cameraID), 10, 10, "v"+std::to_string(cameraID)+" text", viewport);
            viewer_.spinOnce(35, true);
        }
    }

    void run ()
    {
        interface_0_ = new pcl::OpenNIGrabber("#1");
        interface_1_ = new pcl::OpenNIGrabber("#2");

        boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f0 =
                boost::bind (&SimpleOpenNIViewer::cloud_cb_0_, this, _1);
        boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f1 =
                boost::bind (&SimpleOpenNIViewer::cloud_cb_1_, this, _1);

        interface_0_->registerCallback (f0);
        interface_1_->registerCallback (f1);

        interface_0_->start ();
        interface_1_->start ();

        while (!viewer_.wasStopped())
        {
            boost::this_thread::sleep (boost::posix_time::seconds (0.5));
        }

        interface_0_->stop ();
        interface_1_->stop ();
    }

    pcl::visualization::PCLVisualizer
        viewer_;
    int
        v_0_,
        v_1_,
        v_complete_;
    pcl::Grabber
        * interface_0_,
        * interface_1_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr
        cloud_0_,
        cloud_1_,
        cloud_complete_;

    std::mutex mtx;
};

int main ()
{
    SimpleOpenNIViewer v;
    v.run ();
}
