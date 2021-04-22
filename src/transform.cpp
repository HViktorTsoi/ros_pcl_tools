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
#include <pcl_ros/transforms.h>

ros::Publisher pub;
double x, y, z, roll, pitch, yaw;
Eigen::Matrix4f trans;

sensor_msgs::PointCloud2 output;

void
cloud_cb(const sensor_msgs::PointCloud2 &cloud_msg) {

    // transform point cloud
    pcl_ros::transformPointCloud(trans, cloud_msg, output);

    output.header.frame_id = cloud_msg.header.frame_id;
    output.header.stamp = cloud_msg.header.stamp;
    output.is_dense = cloud_msg.is_dense;

    // Publish the data.
    pub.publish(output);
}

int
main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "pcl_transform");
    ros::NodeHandle nh("~");
    ROS_INFO("Transform filter tools initialized...");

    // parameters
    nh.param<double>("x", x, 0.0);
    nh.param<double>("y", y, 0.0);
    nh.param<double>("z", z, 0.0);
    nh.param<double>("roll", roll, 0.0);
    nh.param<double>("pitch", pitch, 0.0);
    nh.param<double>("yaw", yaw, 0.0);

    // to transform matrix
    Eigen::Matrix3f rot = (Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
                           Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                           Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())).matrix();

    trans = Eigen::Matrix4f::Identity();
    trans(0, 3) = x;
    trans(1, 3) = y;
    trans(2, 3) = z;
    trans.block<3, 3>(0, 0) = rot;

    std::cout << "Transform matrix: \n" << trans << std::endl;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

    // Spin
    ros::spin();
}