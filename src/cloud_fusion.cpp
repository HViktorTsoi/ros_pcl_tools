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
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef
message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
        sensor_msgs::PointCloud2> SyncPolicyT;

ros::Publisher pub;
Eigen::Matrix4f trans;

sensor_msgs::PointCloud2 output;

void
cloud_cb(const sensor_msgs::PointCloud2::ConstPtr &cloud_1, const sensor_msgs::PointCloud2::ConstPtr &cloud_2) {
//    ROS_INFO("p1: %d, p2: %d", cloud_1->header.stamp.sec, cloud_2->header.stamp.sec);
    // transform point cloud
    pcl_ros::transformPointCloud(trans, *cloud_2, output);

    // fuse point cloud
    output.data.resize(cloud_1->data.size() + output.data.size());
    std::copy(cloud_1->data.begin(), cloud_1->data.end(),
              output.data.begin() + cloud_2->data.size());

    // meta info
    output.header.frame_id = cloud_1->header.frame_id;
    output.header.stamp = cloud_1->header.stamp;

    // TODO figure out how to deal with `is_dense`, `width`, `height`
    output.is_dense = false;
    output.width = output.data.size() / output.point_step;
    output.height = 1;

    // Publish the data.
    pub.publish(output);
}

int
main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "cloud_fusion");
    ros::NodeHandle nh("~");
    ROS_INFO("cloud fusion started");

    // parameters
    double x, y, z, roll, pitch, yaw;
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

    // 时间同步器
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pc1(nh, "cloud_1", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pc2(nh, "cloud_2", 1);
    SyncPolicyT policy(10);
    policy.setAgePenalty(0.05);
    message_filters::Synchronizer<SyncPolicyT> sync(static_cast<const SyncPolicyT &>(policy), sub_pc1, sub_pc2);
    sync.registerCallback(boost::bind(&cloud_cb, _1, _2));

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

    // Spin
    ros::spin();
}