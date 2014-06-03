#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <pcl/common/distances.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "simplecloudvisualizer.h"
#include "simplecloudgrabber.h"

void elaborateCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src, pcl::PointCloud<pcl::PointXYZ>::Ptr &dst)
{
    if (src->points.size() <= 1)
    {
        dst = src;
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr
            filteredCloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    filteredCloud_xyz = src;

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


int main ()
{
    SimpleCloudVisualizer visualizer;
    boost::thread * visualizerThread = new boost::thread(visualizer);
    SimpleCloudGrabber grabber;
    boost::thread * grabberThread = new boost::thread(grabber);

    std::vector<Eigen::VectorXf> coeffVector;

    while (!visualizer.wasStopped())
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr centers(new pcl::PointCloud<pcl::PointXYZ>);
        cloud = grabber.getCloud();

        elaborateCloud(cloud, cloud);

        if (cloud->points.size() > 0)
        {
            int ballCounter = 0;
            //while (ballCounter < 3)
            {
                pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
                      model_sphere(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>
                        (cloud));
                model_sphere->setRadiusLimits(0.06, 0.07);
                pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_sphere);
                ransac.setDistanceThreshold(0.01);
                ransac.setMaxIterations(50000);
                bool fit = ransac.computeModel();
                Eigen::VectorXf coeff;
                if (fit)
                {
                    ballCounter++;
                    //std::cout << "sphere fit! " << ballCounter << std::endl;

                    ransac.refineModel();

                    std::vector<int> inliers;
                    ransac.getInliers(inliers);

                    pcl::PointCloud<pcl::PointXYZ>::Ptr finalPtr(new pcl::PointCloud<pcl::PointXYZ>());
                    pcl::copyPointCloud<pcl::PointXYZ>(*(cloud), inliers, *finalPtr);

                    visualizer.updateCloud(finalPtr, "ball" + std::to_string(ballCounter), 220, 50, 50, 3);

                    // Nei coefficienti ci sono (x,y,z,R)... speriamo bene...
                    ransac.getModelCoefficients(coeff);
                    coeffVector.push_back(coeff);

                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudBallRemoved(new pcl::PointCloud<pcl::PointXYZ>);

                    pcl::PointXYZ ballCenter(coeff.coeff(0), coeff.coeff(1), coeff.coeff(2));
                    double radius = coeff.coeff(3);
                    double epsilon = 0.01;

                    std::cout << radius << std::endl;

                    /*int inliersCounter = 0;

                    for (int k = 0; k < cloud->points.size(); k++)
                    {
                        if (k == inliers[inliersCounter])
                        {
                            inliersCounter++;
                        }
                        else if (pcl::euclideanDistance(cloud->points[k], ballCenter) >= (radius+epsilon))
                        {
                            cloudBallRemoved->points.push_back(cloud->points[k]);
                        }
                    }
                    cloudBallRemoved->width = cloudBallRemoved->points.size();
                    cloudBallRemoved->height = 1;
                    cloud = cloudBallRemoved;*/

                    centers->points.push_back(ballCenter);
                }
            }

            visualizer.updateCloud(cloud, "scene", 100, 100, 200, 1);
        }

        centers->width = centers->points.size();
        centers->height = 1;

        visualizer.updateCloud(centers, "centers", 100, 200, 100, 2);

        usleep(50000);
    }

    return 0;
}
