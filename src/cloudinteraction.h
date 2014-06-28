#ifndef CLOUDINTERACTION_H
#define CLOUDINTERACTION_H

#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>

namespace clinter {

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;

extern float linear_increment;
extern float angular_increment;

Eigen::Matrix4f identity();

void x_t_inc_callback(CloudPtr &in_out_cloud, boost::shared_ptr<Eigen::Matrix4f> &transformMatrix, float step = 0.0);
void x_t_dec_callback(CloudPtr &in_out_cloud, boost::shared_ptr<Eigen::Matrix4f> &transformMatrix, float step = 0.0);

void y_t_inc_callback(CloudPtr &in_out_cloud, boost::shared_ptr<Eigen::Matrix4f> &transformMatrix, float step = 0.0);
void y_t_dec_callback(CloudPtr &in_out_cloud, boost::shared_ptr<Eigen::Matrix4f> &transformMatrix, float step = 0.0);

void z_t_inc_callback(CloudPtr &in_out_cloud, boost::shared_ptr<Eigen::Matrix4f> &transformMatrix, float step = 0.0);
void z_t_dec_callback(CloudPtr &in_out_cloud, boost::shared_ptr<Eigen::Matrix4f> &transformMatrix, float step = 0.0);

void x_r_inc_callback(CloudPtr &in_out_cloud, boost::shared_ptr<Eigen::Matrix4f> &transformMatrix, float step = 0.0);
void x_r_dec_callback(CloudPtr &in_out_cloud, boost::shared_ptr<Eigen::Matrix4f> &transformMatrix, float step = 0.0);

void y_r_inc_callback(CloudPtr &in_out_cloud, boost::shared_ptr<Eigen::Matrix4f> &transformMatrix, float step = 0.0);
void y_r_dec_callback(CloudPtr &in_out_cloud, boost::shared_ptr<Eigen::Matrix4f> &transformMatrix, float step = 0.0);

void z_r_inc_callback(CloudPtr &in_out_cloud, boost::shared_ptr<Eigen::Matrix4f> &transformMatrix, float step = 0.0);
void z_r_dec_callback(CloudPtr &in_out_cloud, boost::shared_ptr<Eigen::Matrix4f> &transformMatrix, float step = 0.0);

}

#endif // CLOUDINTERACTION_H
