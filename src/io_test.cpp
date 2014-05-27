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

#include <pcl/io/pcd_io.h>

#include <Eigen/Geometry>

#include <mutex>
#include <unistd.h>

#include "visualizerthread.h"
#include "cloudsgrabber.h"

#define MIN_X -0.3f
#define MAX_X 0.3f
#define MIN_Y -0.3f
#define MAX_Y 0.55f
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
        grid.setLeafSize (0.015, 0.015, 0.015);
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
    //norm_est.setKSearch (5);
    norm_est.setRadiusSearch(0.05);

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

    reg.setTransformationEpsilon (1e-1);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance (0.05);

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

void transformCloud1(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    Eigen::Matrix4f transformMatrix;

    //[0.7158866767323938, -0.6979344925893711, -0.01984213020485748;
    //  -0.5095394008595089, -0.5416529258064282, 0.6685669053558281;
    //  -0.477363451731259, -0.4685077929120851, -0.7433872361911151]
    //[0.1464000185043455; 0.04536204145848141; 0.9365197730555788]
    Eigen::Matrix3f m;

    // this is the R inverse
    m(0,0) = 0.7158866767323938;
    m(1,0) = -0.6979344925893711;
    m(2,0) = -0.01984213020485748;

    m(0,1) = -0.5095394008595089;
    m(1,1) = -0.5416529258064282;
    m(2,1) = 0.6685669053558281;

    m(0,2) = -0.477363451731259;
    m(1,2) = -0.4685077929120851;
    m(2,2) = -0.7433872361911151;

    /// R
    transformMatrix(0,0) = m(0,0);
    transformMatrix(0,1) = m(0,1);
    transformMatrix(0,2) = m(0,2);

    transformMatrix(1,0) = m(1,0);
    transformMatrix(1,1) = m(1,1);
    transformMatrix(1,2) = m(1,2);

    transformMatrix(2,0) = m(2,0);
    transformMatrix(2,1) = m(2,1);
    transformMatrix(2,2) = m(2,2);

    /// T
    Eigen::Vector3f t(0.1464000185043455, 0.04536204145848141, 0.9365197730555788);
    t = -m * t;
    transformMatrix(0,3) = t(0);
    transformMatrix(1,3) = t(1);
    transformMatrix(2,3) = t(2);

    /// Homogeneous...
    transformMatrix(3,0) = 0.0f;
    transformMatrix(3,1) = 0.0f;
    transformMatrix(3,2) = 0.0f;
    transformMatrix(3,3) = 1.0f;

    /// CAMERA 1
    //[0.8357691996146269, -0.4753603425845116, 0.2748133724429399;
    //  -0.4802294966068207, -0.3901695649885069, 0.7855872587723397;
    //  -0.2662132144809604, -0.7885431220007028, -0.5543737630706372]
    //[-0.259070065563638; -0.2356049994226622; 0.9611893197610495]

    pcl::transformPointCloud(*cloud, *cloud, transformMatrix);
}

void transformCloud0(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    Eigen::Matrix4f transformMatrix;

    Eigen::Matrix3f m;

    //[-0.6838783279023093, -0.7295779820349809, 0.005138166531651278;
    //  -0.5497475543272136, 0.5199167453092753, 0.6538074674231383;
    //  -0.4796749515419679, 0.4443000631068781, -0.7566435057452321]
    //[-0.1135204299158313; -0.03643457180122128; 0.858676674917458]

    m(0,0) = -0.6838783279023093;
    m(1,0) = -0.7295779820349809;
    m(2,0) = 0.005138166531651278;

    m(0,1) = -0.5497475543272136;
    m(1,1) = 0.5199167453092753;
    m(2,1) = 0.6538074674231383;

    m(0,2) = -0.4796749515419679;
    m(1,2) = 0.4443000631068781;
    m(2,2) = -0.7566435057452321;

    /// R
    transformMatrix(0,0) = m(0,0);
    transformMatrix(0,1) = m(0,1);
    transformMatrix(0,2) = m(0,2);

    transformMatrix(1,0) = m(1,0);
    transformMatrix(1,1) = m(1,1);
    transformMatrix(1,2) = m(1,2);

    transformMatrix(2,0) = m(2,0);
    transformMatrix(2,1) = m(2,1);
    transformMatrix(2,2) = m(2,2);

    /// T
    Eigen::Vector3f t(-0.1135204299158313, -0.03643457180122128, 0.858676674917458);
    t = -m * t;
    transformMatrix(0,3) = t(0);
    transformMatrix(1,3) = t(1);
    transformMatrix(2,3) = t(2);

    /// Homogeneous...
    transformMatrix(3,0) = 0.0f;
    transformMatrix(3,1) = 0.0f;
    transformMatrix(3,2) = 0.0f;
    transformMatrix(3,3) = 1.0f;

    pcl::transformPointCloud(*cloud, *cloud, transformMatrix);
}

void elaborateCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src, pcl::PointCloud<pcl::PointXYZ>::Ptr &dst)
{
    if (src->points.size() <= 1)
    {
        dst = src;
        return;
    }
    //PCL_INFO("1 - Scene filter (MAX-MIN)");
    /// 1 - Scene filter (MAX-MIN)
    pcl::PointCloud<pcl::PointXYZ>::Ptr
            filteredCloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    passTroughFilterXYZ(src, filteredCloud_xyz);

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
//    dst = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    int currentInlier = 0;
    for (int i = 0; i < noOutliersCloud->points.size(); i++)
    {
        if (inliers->indices[currentInlier] != i)
        {
            segmentedCloud->points.push_back(noOutliersCloud->points[i]);
//            dst->points.push_back(noOutliersCloud->points[i]);
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
    dst = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    sor.setInputCloud (segmentedCloud);
    sor.setMeanK (25);
    sor.setStddevMulThresh (1.5);
    sor.filter (*dst);
    dst->width = (int) dst->points.size();
    dst->height = 1;
}

void computeBB(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, BoundingBox &bb)
{
    //PCL_INFO("6 - Bounding-Box computation");
    /// 6 - Bounding-Box computation
    // compute principal direction
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
    eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

    // move the points to that reference frame
    Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
    p2w.block<3,3>(0,0) = eigDx.transpose();
    p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
    pcl::PointCloud<pcl::PointXYZ> cPoints;
    pcl::transformPointCloud(*cloud, cPoints, p2w);

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);
    const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

    // final transform
    const Eigen::Quaternionf qfinal(eigDx);
    const Eigen::Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();

    bb.max_pt = max_pt;
    bb.min_pt = min_pt;
    bb.qfinal = qfinal;
    bb.tfinal = tfinal;
}


int main ()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_0_a (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_a (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_0_b (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_b (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_0_c (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_c (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_0_d (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_d (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_0_e (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_e (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_0_f (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_f (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_0_a_e (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_a_e (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_0_b_e (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_b_e (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_0_c_e (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_c_e (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_0_d_e (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_d_e (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_0_e_e (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_e_e (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_0_f_e (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_f_e (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/mpp/PANOTEC/RGBD_ObjectVolumeEstimator/data/cloud_0_a.pcd", *cloud_0_a);
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/mpp/PANOTEC/RGBD_ObjectVolumeEstimator/data/cloud_1_a.pcd", *cloud_1_a);
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/mpp/PANOTEC/RGBD_ObjectVolumeEstimator/data/cloud_0_b.pcd", *cloud_0_b);
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/mpp/PANOTEC/RGBD_ObjectVolumeEstimator/data/cloud_1_b.pcd", *cloud_1_b);
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/mpp/PANOTEC/RGBD_ObjectVolumeEstimator/data/cloud_0_c.pcd", *cloud_0_c);
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/mpp/PANOTEC/RGBD_ObjectVolumeEstimator/data/cloud_1_c.pcd", *cloud_1_c);
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/mpp/PANOTEC/RGBD_ObjectVolumeEstimator/data/cloud_0_d.pcd", *cloud_0_d);
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/mpp/PANOTEC/RGBD_ObjectVolumeEstimator/data/cloud_1_d.pcd", *cloud_1_d);
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/mpp/PANOTEC/RGBD_ObjectVolumeEstimator/data/cloud_0_e.pcd", *cloud_0_e);
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/mpp/PANOTEC/RGBD_ObjectVolumeEstimator/data/cloud_1_e.pcd", *cloud_1_e);
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/mpp/PANOTEC/RGBD_ObjectVolumeEstimator/data/cloud_0_f.pcd", *cloud_0_f);
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/mpp/PANOTEC/RGBD_ObjectVolumeEstimator/data/cloud_1_f.pcd", *cloud_1_f);

    transformCloud0(cloud_0_a);
    transformCloud1(cloud_1_a);
    transformCloud0(cloud_0_b);
    transformCloud1(cloud_1_b);
    transformCloud0(cloud_0_c);
    transformCloud1(cloud_1_c);
    transformCloud0(cloud_0_d);
    transformCloud1(cloud_1_d);
    transformCloud0(cloud_0_e);
    transformCloud1(cloud_1_e);
    transformCloud0(cloud_0_f);
    transformCloud1(cloud_1_f);

    elaborateCloud(cloud_0_a, cloud_0_a_e);
    elaborateCloud(cloud_1_a, cloud_1_a_e);
    elaborateCloud(cloud_0_b, cloud_0_b_e);
    elaborateCloud(cloud_1_b, cloud_1_b_e);
    elaborateCloud(cloud_0_c, cloud_0_c_e);
    elaborateCloud(cloud_1_c, cloud_1_c_e);
    elaborateCloud(cloud_0_d, cloud_0_d_e);
    elaborateCloud(cloud_1_d, cloud_1_d_e);
    elaborateCloud(cloud_0_e, cloud_0_e_e);
    elaborateCloud(cloud_1_e, cloud_1_e_e);
    elaborateCloud(cloud_0_f, cloud_0_f_e);
    elaborateCloud(cloud_1_f, cloud_1_f_e);

    pcl::io::savePCDFileASCII("/home/mpp/PANOTEC/RGBD_ObjectVolumeEstimator/data/cloud_0_a_transformed_elaborated.pcd", *cloud_0_a_e);
    pcl::io::savePCDFileASCII("/home/mpp/PANOTEC/RGBD_ObjectVolumeEstimator/data/cloud_1_a_transformed_elaborated.pcd", *cloud_1_a_e);
    pcl::io::savePCDFileASCII("/home/mpp/PANOTEC/RGBD_ObjectVolumeEstimator/data/cloud_0_b_transformed_elaborated.pcd", *cloud_0_b_e);
    pcl::io::savePCDFileASCII("/home/mpp/PANOTEC/RGBD_ObjectVolumeEstimator/data/cloud_1_b_transformed_elaborated.pcd", *cloud_1_b_e);
    pcl::io::savePCDFileASCII("/home/mpp/PANOTEC/RGBD_ObjectVolumeEstimator/data/cloud_0_c_transformed_elaborated.pcd", *cloud_0_c_e);
    pcl::io::savePCDFileASCII("/home/mpp/PANOTEC/RGBD_ObjectVolumeEstimator/data/cloud_1_c_transformed_elaborated.pcd", *cloud_1_c_e);
    pcl::io::savePCDFileASCII("/home/mpp/PANOTEC/RGBD_ObjectVolumeEstimator/data/cloud_0_d_transformed_elaborated.pcd", *cloud_0_d_e);
    pcl::io::savePCDFileASCII("/home/mpp/PANOTEC/RGBD_ObjectVolumeEstimator/data/cloud_1_d_transformed_elaborated.pcd", *cloud_1_d_e);
    pcl::io::savePCDFileASCII("/home/mpp/PANOTEC/RGBD_ObjectVolumeEstimator/data/cloud_0_e_transformed_elaborated.pcd", *cloud_0_e_e);
    pcl::io::savePCDFileASCII("/home/mpp/PANOTEC/RGBD_ObjectVolumeEstimator/data/cloud_1_e_transformed_elaborated.pcd", *cloud_1_e_e);
    pcl::io::savePCDFileASCII("/home/mpp/PANOTEC/RGBD_ObjectVolumeEstimator/data/cloud_0_f_transformed_elaborated.pcd", *cloud_0_f_e);
    pcl::io::savePCDFileASCII("/home/mpp/PANOTEC/RGBD_ObjectVolumeEstimator/data/cloud_1_f_transformed_elaborated.pcd", *cloud_1_f_e);

    /*
    //Start visualizer thread
    VisualizerThread visualizer;
    boost::thread * visualizerThread = new boost::thread(visualizer);

    CloudsGrabber cg;
    boost::thread grabberThread(cg);

    while (!visualizer.wasStopped())
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr c0 = cg.getCloud0();
        pcl::PointCloud<pcl::PointXYZ>::Ptr c1 = cg.getCloud1();
        pcl::PointCloud<pcl::PointXYZ>::Ptr e0, e1;

        //std::cout << "wtf??" << c0->points.size() << " - " << c1->points.size() << std::endl;

        elaborateCloud(c0, e0);
        elaborateCloud(c1, e1);

        visualizer.updateCloud(e0, VisualizerThread::VIEWPORT::OBJ0);
        visualizer.updateCloud(e1, VisualizerThread::VIEWPORT::OBJ1);

        pcl::PointCloud<pcl::PointXYZ>::Ptr
                registered (new pcl::PointCloud<pcl::PointXYZ>);
        if (e0->points.size() > 1 && e1->points.size() > 1)
        {
            //PCL_INFO("3 - registration");
            /// 3 - registration...
            transformCloud1(e1);
            transformCloud0(e0);
            Eigen::Matrix4f GlobalTransform;
            pairAlign(e0, e1, registered, GlobalTransform);
            //*registered = *e0;
            //*registered += *e1;
        }
        visualizer.updateCloud(registered, VisualizerThread::VIEWPORT::COMPLETE);

        BoundingBox bb;
        computeBB(registered, bb);
        std::cout << "(x,y,z): (" << bb.max_pt.x - bb.min_pt.x
                  <<  ", " << bb.max_pt.y - bb.min_pt.y
                  <<  ", " << bb.max_pt.z - bb.min_pt.z << ")" << std::endl;

        visualizer.updateBB(bb);

        usleep(500000);
        //boost::this_thread::sleep (boost::posix_time::seconds (0.5));
    }
    cg.stopGrabber();
    std::cout << "exiting..." << std::endl;
    grabberThread.join();
    visualizerThread->join();
    return 0;
*/
}
