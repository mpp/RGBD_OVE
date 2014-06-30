#ifndef CLOUDINTERACTION_H
#define CLOUDINTERACTION_H

#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#define MIN_X -0.5f
#define MAX_X 0.5f
#define MIN_Y -0.26f
#define MAX_Y 0.55f
#define MIN_Z 0.0f
#define MAX_Z 1.5f

typedef struct BoundingBox_
{
    pcl::PointXYZ
        min_pt,
        max_pt;

    Eigen::Quaternionf
        qfinal;
    Eigen::Vector3f
        tfinal;

} BoundingBox;

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


void passTroughFilterXYZ(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered);

void elaborateCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src, pcl::PointCloud<pcl::PointXYZ>::Ptr &dst);

void computeBB(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, BoundingBox &bb);

}

#endif // CLOUDINTERACTION_H
