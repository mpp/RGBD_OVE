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
void x_t_inc_callback(CloudPtr &in_out_cloud, float step)
{
    step = checkLinStep(step);

    Eigen::Matrix4f transformMatrix = identity();

    transformMatrix(0,3) = step;

    pcl::transformPointCloud(*in_out_cloud, *in_out_cloud, transformMatrix);
}

void x_t_dec_callback(CloudPtr &in_out_cloud, float step)
{
    step = checkLinStep(step);

    Eigen::Matrix4f transformMatrix = identity();

    transformMatrix(0,3) = -step;

    pcl::transformPointCloud(*in_out_cloud, *in_out_cloud, transformMatrix);
}

/// Y axis linear
void y_t_inc_callback(CloudPtr &in_out_cloud, float step)
{
    step = checkLinStep(step);

    Eigen::Matrix4f transformMatrix = identity();

    transformMatrix(1,3) = step;

    pcl::transformPointCloud(*in_out_cloud, *in_out_cloud, transformMatrix);
}

void y_t_dec_callback(CloudPtr &in_out_cloud, float step)
{
    step = checkLinStep(step);

    Eigen::Matrix4f transformMatrix = identity();

    transformMatrix(1,3) = -step;

    pcl::transformPointCloud(*in_out_cloud, *in_out_cloud, transformMatrix);
}

/// Z axis linear
void z_t_inc_callback(CloudPtr &in_out_cloud, float step)
{
    step = checkLinStep(step);

    Eigen::Matrix4f transformMatrix = identity();

    transformMatrix(2,3) = step;

    pcl::transformPointCloud(*in_out_cloud, *in_out_cloud, transformMatrix);
}

void z_t_dec_callback(CloudPtr &in_out_cloud, float step)
{
    step = checkLinStep(step);

    Eigen::Matrix4f transformMatrix = identity();

    transformMatrix(2,3) = -step;

    pcl::transformPointCloud(*in_out_cloud, *in_out_cloud, transformMatrix);
}

/// X axis angular
void x_r_inc_callback(CloudPtr &in_out_cloud, float step)
{
    step = checkAngStep(step);

    Eigen::Matrix4f transformMatrix = identity();

    applyRotation(step, Eigen::Vector3f(1.0,0,0), transformMatrix);

    pcl::transformPointCloud(*in_out_cloud, *in_out_cloud, transformMatrix);
}

void x_r_dec_callback(CloudPtr &in_out_cloud, float step)
{
    step = checkAngStep(step);

    Eigen::Matrix4f transformMatrix = identity();

    applyRotation(-step, Eigen::Vector3f(1.0,0,0), transformMatrix);

    pcl::transformPointCloud(*in_out_cloud, *in_out_cloud, transformMatrix);
}

/// Y axis angular
void y_r_inc_callback(CloudPtr &in_out_cloud, float step)
{
    step = checkAngStep(step);

    Eigen::Matrix4f transformMatrix = identity();

    applyRotation(step, Eigen::Vector3f(0,1.0,0), transformMatrix);

    pcl::transformPointCloud(*in_out_cloud, *in_out_cloud, transformMatrix);
}

void y_r_dec_callback(CloudPtr &in_out_cloud, float step)
{
    step = checkAngStep(step);

    Eigen::Matrix4f transformMatrix = identity();

    applyRotation(-step, Eigen::Vector3f(0,1.0,0), transformMatrix);

    pcl::transformPointCloud(*in_out_cloud, *in_out_cloud, transformMatrix);
}

/// Z axis angular
void z_r_inc_callback(CloudPtr &in_out_cloud, float step)
{
    step = checkAngStep(step);

    Eigen::Matrix4f transformMatrix = identity();

    applyRotation(step, Eigen::Vector3f(0,0,1.0), transformMatrix);

    pcl::transformPointCloud(*in_out_cloud, *in_out_cloud, transformMatrix);
}

void z_r_dec_callback(CloudPtr &in_out_cloud, float step)
{
    step = checkAngStep(step);

    Eigen::Matrix4f transformMatrix = identity();

    applyRotation(-step, Eigen::Vector3f(0,0,1.0), transformMatrix);

    pcl::transformPointCloud(*in_out_cloud, *in_out_cloud, transformMatrix);
}

}
