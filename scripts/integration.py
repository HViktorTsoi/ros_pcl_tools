#!/usr/bin/env python2.7
# coding=utf-8
import copy

import rospy
from sensor_msgs.msg import PointCloud2

pc_integration = []
overlap_count = 10


def cb_pc(pc_msg):
    global pc_integration
    # 入队列
    pc_integration.append(pc_msg)
    # 取固定长度
    pc_integration = pc_integration[-overlap_count:]

    stacking = copy.copy(pc_msg)
    stacking.data = b''
    stacking.width *= len(pc_integration)
    stacking.row_step *= len(pc_integration)
    for pc_msg in pc_integration:
        stacking.data += pc_msg.data
    pub.publish(stacking)


if __name__ == '__main__':
    rospy.init_node('lidar_integration')
    rospy.loginfo('Init...')
    rospy.Subscriber('/livox/lidar', PointCloud2, cb_pc, queue_size=10)
    pub = rospy.Publisher('/livox/integration', PointCloud2, queue_size=10)
    # rospy.Subscriber('~input', PointCloud2, cb_pc, queue_size=10)
    # pub = rospy.Publisher('~integration', PointCloud2, queue_size=10)
    rospy.spin()
