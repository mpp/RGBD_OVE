
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

// Opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/thread.hpp>

#include "cloudsgrabber.h"

int min_x = 50;
int max_x = 50;
int min_y = 26;
int max_y = 55;
int min_z = 0;
int max_z = 150;

/*void on_trackbar_min_x(int a, void *b)
{
    std::cout << "a: " << a << " - b: " << *b << std::endl;

    min_x = min_x_t / 100;
}*/

// PCL Visualizer to view the pointcloud
pcl::visualization::PCLVisualizer viewer ("Simple visualizing window");

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
    passTroughFilter.setFilterLimits(((float)-1*min_x)/100.0f, ((float)max_x)/100.0f);
    passTroughFilter.filter(*filteredCloud_x);

    // PassThrough y
    passTroughFilter.setInputCloud(filteredCloud_x);
    passTroughFilter.setFilterFieldName("y");
    passTroughFilter.setFilterLimits(((float)-1*min_y)/100.0f, ((float)max_y)/100.0f);
    passTroughFilter.filter(*filteredCloud_y);

    // PassThrough z
    passTroughFilter.setInputCloud(filteredCloud_y);
    passTroughFilter.setFilterFieldName("z");
    passTroughFilter.setFilterLimits(((float)-1*min_z)/100.0f, ((float)max_z)/100.0f);
    passTroughFilter.filter(*filtered);
}



int main()
{
    cv::namedWindow("trackbars");
    cv::createTrackbar("X_min_limit", "trackbars", &min_x, 150, NULL);
    cv::createTrackbar("X_max_limit", "trackbars", &max_x, 150, NULL);
    cv::createTrackbar("Y_min_limit", "trackbars", &min_y, 150, NULL);
    cv::createTrackbar("Y_max_limit", "trackbars", &max_y, 150, NULL);
    cv::createTrackbar("Z_min_limit", "trackbars", &min_z, 150, NULL);
    cv::createTrackbar("Z_max_limit", "trackbars", &max_z, 150, NULL);

    CloudsGrabber cg;
    boost::thread grabberThread(cg);
    pcl::PointCloud<pcl::PointXYZ>::Ptr c0(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr c1(new pcl::PointCloud<pcl::PointXYZ>);

    char last_c = 0;
    while(true && (last_c != 27))
    {

        c0 = cg.getCloud0();

        passTroughFilterXYZ(c0, c1);

        // Visualizing pointcloud
        viewer.addPointCloud (c1, "scene_cloud");
        viewer.spinOnce();
        viewer.removePointCloud("scene_cloud");

        last_c = cv::waitKey(33);
    }
}
