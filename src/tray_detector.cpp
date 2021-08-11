// ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
​
// OpenCV specific includes
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
​
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl_conversions/pcl_conversions.h>
​
class TrayDetector
{
private:
    /*node handle*/
    ros::NodeHandle nh;
    /*subscriber*/
    ros::Subscriber sub;
    /*publisher*/
    ros::Publisher pub;
    /*pcl objects*/
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZRGB>};
    pcl::PointCloud<pcl::PointNormal>::Ptr normals{new pcl::PointCloud<pcl::PointNormal>};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered{new pcl::PointCloud<pcl::PointXYZRGB>};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr total_of_planes{new pcl::PointCloud<pcl::PointXYZRGB>};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_cloud{new pcl::PointCloud<pcl::PointXYZRGB>};
​
    double y_max = -100.0;
​
public:
    TrayDetector();
    void Callback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void PlaneDetection(void);
    void NormalEstimation(void);
    void DeskDetection(void);
    void GetDeskCollor(void);
    void GetDeskHeigiht(void);
    void Publication(void);
};
​
TrayDetector::TrayDetector()
{
    // Create a ROS subscriber for the input point cloud
    sub = nh.subscribe("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 1, &TrayDetector::Callback, this);
    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("/tray_detector", 1);
}
​
void TrayDetector::Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    if (!ros::ok())
    {
        std::cout << "(error):not subscribe" << std::endl;
        return;
    }
    std::cout << "CALLBACK" << std::endl;
    pcl::fromROSMsg(*msg, *cloud);
    PlaneDetection();
    copyPointCloud(*total_of_planes, *normals);
    NormalEstimation();
    DeskDetection();
    Publication();
}
​
// Function to detect a plane
void TrayDetector::PlaneDetection(void)
{
    std::cout << "DETECTING PLANE." << std::endl;
    // Down sampling
    pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.03, 0.03, 0.03);
    sor.filter(*cloud_filtered);

    // Preparation for plane detection
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.1);
​
    *total_of_planes = *cloud_filtered;
​
    // Plane detection
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
​
    //3-plane statement loop processing
    for (int plane = 0; plane < 3; ++plane)
    {
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
​
        // An object that detects and stores planes.
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr one_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
​
        // Delete non-detected points from one_plane
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*one_plane);
​
        // Delete the detected point cloud from cloud_filtered.
        extract.setNegative(true);
        extract.filter(*cloud_filtered);
​
        // add planes
        *total_of_planes += *one_plane;
    }
}
​
// Function to estimate the normal
void TrayDetector::NormalEstimation(void)
{
​
    std::cout << "NORMAL ESTIMATION" << std::endl;
​
    /*remove NaN points from point cloud*/
    std::vector<int> nan_index;
    pcl::removeNaNFromPointCloud(*normals, *normals, nan_index);
​
    /*Declare the normal estimation class*/
    pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal> ne;
    /*Specify the input point cloud*/
    ne.setInputCloud(normals);
    /*kd-tree*/
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());
    /*Specify the search method for nearest neighbor point clouds.*/
    ne.setSearchMethod(tree);
    /*Specify the search radius of the nearest point cloud.*/
    ne.setRadiusSearch(0.07);
    /*Perform normal estimation*/
    ne.compute(*normals);
}
​
// Function to remove point clouds other than the desk
void TrayDetector::DeskDetection(void)
{
    pcl::PassThrough<pcl::PointNormal> pass;
    pass.setInputCloud(normals);
    pass.setFilterFieldName("normal_y");
    pass.setFilterLimits(-1.0, 0.0);
    pass.filter(*normals);
​
    for (size_t i = 0; i < normals->points.size(); i++)
    {
        if (normals->points[i].y > y_max)
        {
            y_max = normals->points[i].y;
        }
    }
    pass.setInputCloud(normals);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_max - 1.1, y_max - 0.3);
    pass.filter(*normals);
}

// Function to get the color of a desk
void TrayDetector::GetDeskCollor(void){

}

// Function to get the height of a desk
void TrayDetector::GetDeskHeigiht(void){

}
​
// pcl::PointCloud<T> to sensor_msgs::PointCloud
void TrayDetector::Publication(void)
{
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*normals, output);
    pub.publish(output);
}
​
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tray_detector");
    std::cout << "TrayDetection Start" << std::endl;
​
    TrayDetector tray_detector;
    ros::spin();
}