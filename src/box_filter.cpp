//
// Created by hvt on 19-9-1.
//

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

ros::Publisher pub;
double minX, maxX, minY, maxY, minZ, maxZ;
bool negative;
pcl::PCLPointCloud2 output;

void
cloud_cb(const pcl::PCLPointCloud2ConstPtr &cloud_msg) {
//    std::cout << minX << minZ << std::endl;
    // remove ego points
    pcl::CropBox<pcl::PCLPointCloud2> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter.setInputCloud(cloud_msg);
    boxFilter.setNegative(negative);
    boxFilter.filter(output);

    output.header.frame_id = cloud_msg->header.frame_id;
    output.header.stamp = cloud_msg->header.stamp;
    output.is_dense = cloud_msg->is_dense;

    // Publish the data.
    pub.publish(output);
}

int
main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "pcl_box_filter");
    ros::NodeHandle nh("~");

    // parameters
    nh.param<double>("x_max", maxX, 0.0);
    nh.param<double>("x_min", minX, 0.0);
    nh.param<double>("y_max", maxY, 0.0);
    nh.param<double>("y_min", minY, 0.0);
    nh.param<double>("z_max", maxZ, 0.0);
    nh.param<double>("z_min", minZ, 0.0);
    nh.param<bool>("negative", negative, false);

    ROS_INFO("Box filter tools initialized...");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

    // Spin
    ros::spin();
}