#include "cloudinteraction.h"

namespace clinter {

float linear_increment = 0.01;     //meters
float angular_increment = 0.01;    //rads

float checkLinStep(float step)
{
    if (0 == step)
    {
        return linear_increment;
    }
    return step;
}
float checkAngStep(float step)
{
    if (0 == step)
    {
        return angular_increment;
    }
    return step;
}

Eigen::Matrix4f identity()
{
    Eigen::Matrix4f mat;

    mat << 1,0,0,0,
            0,1,0,0,
            0,0,1,0,
            0,0,0,1;

    return mat;
}

void applyRotation(double angle, const Eigen::Vector3f &axis, Eigen::Matrix4f &transformMatrix)
{
    Eigen::Matrix3f m;
    m = Eigen::AngleAxisf(angle, axis);

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
}

/// X axis linear
void x_t_inc_callback(CloudPtr &in_out_cloud, boost::shared_ptr<Eigen::Matrix4f> &transformMatrix, float step)
{
    step = checkLinStep(step);

    (*transformMatrix) = identity();

    (*transformMatrix)(0,3) = step;

    pcl::transformPointCloud(*in_out_cloud, *in_out_cloud, (*transformMatrix));
}

void x_t_dec_callback(CloudPtr &in_out_cloud, boost::shared_ptr<Eigen::Matrix4f> &transformMatrix, float step)
{
    step = checkLinStep(step);

    (*transformMatrix) = identity();

    (*transformMatrix)(0,3) = -step;

    pcl::transformPointCloud(*in_out_cloud, *in_out_cloud, (*transformMatrix));
}

/// Y axis linear
void y_t_inc_callback(CloudPtr &in_out_cloud, boost::shared_ptr<Eigen::Matrix4f> &transformMatrix, float step)
{
    step = checkLinStep(step);

    (*transformMatrix) = identity();

    (*transformMatrix)(1,3) = step;

    pcl::transformPointCloud(*in_out_cloud, *in_out_cloud, (*transformMatrix));
}

void y_t_dec_callback(CloudPtr &in_out_cloud, boost::shared_ptr<Eigen::Matrix4f> &transformMatrix, float step)
{
    step = checkLinStep(step);

    (*transformMatrix) = identity();

    (*transformMatrix)(1,3) = -step;

    pcl::transformPointCloud(*in_out_cloud, *in_out_cloud, (*transformMatrix));
}

/// Z axis linear
void z_t_inc_callback(CloudPtr &in_out_cloud, boost::shared_ptr<Eigen::Matrix4f> &transformMatrix, float step)
{
    step = checkLinStep(step);

    (*transformMatrix) = identity();

    (*transformMatrix)(2,3) = step;

    pcl::transformPointCloud(*in_out_cloud, *in_out_cloud, (*transformMatrix));
}

void z_t_dec_callback(CloudPtr &in_out_cloud, boost::shared_ptr<Eigen::Matrix4f> &transformMatrix, float step)
{
    step = checkLinStep(step);

    (*transformMatrix) = identity();

    (*transformMatrix)(2,3) = -step;

    pcl::transformPointCloud(*in_out_cloud, *in_out_cloud, (*transformMatrix));
}

/// X axis angular
void x_r_inc_callback(CloudPtr &in_out_cloud, boost::shared_ptr<Eigen::Matrix4f> &transformMatrix, float step)
{
    step = checkAngStep(step);

    (*transformMatrix) = identity();

    applyRotation(step, Eigen::Vector3f(1.0,0,0), (*transformMatrix));

    pcl::transformPointCloud(*in_out_cloud, *in_out_cloud, (*transformMatrix));
}

void x_r_dec_callback(CloudPtr &in_out_cloud, boost::shared_ptr<Eigen::Matrix4f> &transformMatrix, float step)
{
    step = checkAngStep(step);

    (*transformMatrix) = identity();

    applyRotation(-step, Eigen::Vector3f(1.0,0,0), (*transformMatrix));

    pcl::transformPointCloud(*in_out_cloud, *in_out_cloud, (*transformMatrix));
}

/// Y axis angular
void y_r_inc_callback(CloudPtr &in_out_cloud, boost::shared_ptr<Eigen::Matrix4f> &transformMatrix, float step)
{
    step = checkAngStep(step);

    (*transformMatrix) = identity();

    applyRotation(step, Eigen::Vector3f(0,1.0,0), (*transformMatrix));

    pcl::transformPointCloud(*in_out_cloud, *in_out_cloud, (*transformMatrix));
}

void y_r_dec_callback(CloudPtr &in_out_cloud, boost::shared_ptr<Eigen::Matrix4f> &transformMatrix, float step)
{
    step = checkAngStep(step);

    (*transformMatrix) = identity();

    applyRotation(-step, Eigen::Vector3f(0,1.0,0), (*transformMatrix));

    pcl::transformPointCloud(*in_out_cloud, *in_out_cloud, (*transformMatrix));
}

/// Z axis angular
void z_r_inc_callback(CloudPtr &in_out_cloud, boost::shared_ptr<Eigen::Matrix4f> &transformMatrix, float step)
{
    step = checkAngStep(step);

    (*transformMatrix) = identity();

    applyRotation(step, Eigen::Vector3f(0,0,1.0), (*transformMatrix));

    pcl::transformPointCloud(*in_out_cloud, *in_out_cloud, (*transformMatrix));
}

void z_r_dec_callback(CloudPtr &in_out_cloud, boost::shared_ptr<Eigen::Matrix4f> &transformMatrix, float step)
{
    step = checkAngStep(step);

    (*transformMatrix) = identity();

    applyRotation(-step, Eigen::Vector3f(0,0,1.0), (*transformMatrix));

    pcl::transformPointCloud(*in_out_cloud, *in_out_cloud, (*transformMatrix));
}


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

    pcl::PointXYZ min, max;
    pcl::getMinMax3D(*cloud, min, max);

    bb.max_pt = min;
    bb.min_pt = max;
    bb.qfinal = qfinal;
    bb.tfinal = tfinal;
}

}
