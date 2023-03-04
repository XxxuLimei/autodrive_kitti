#!/usr/bin/env python
import cv2
import os
import numpy as np

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pcl2
from cv_bridge import CvBridge
import torch

DATA_PATH = '/home/xilm/fuxian/auto_drive_kitti/RawData/2011_09_26/2011_09_26_drive_0005_sync/'

if __name__ == '__main__':
    frame = 0
    rospy.init_node('kitti_node', anonymous=True)
    cam_pub = rospy.Publisher('kitti_cam', Image, queue_size=10)
    bridge = CvBridge()
    pcl_pub = rospy.Publisher('kitti_point_cloud', PointCloud2, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        img = cv2.imread(os.path.join(DATA_PATH, 'image_02/data/%010d.png'%frame))
        point_cloud = np.fromfile(os.path.join(DATA_PATH, 'velodyne_points/data/%010d.bin'%frame), dtype=np.float32).reshape(-1,4) # n*4 matrix, including x, y, z, reflection rate
        cam_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        pcl_pub.publish(pcl2.create_cloud_xyz32(header, point_cloud[:, :3])) # doesn't receive reflection rate parameter
        rospy.loginfo("published")
        rate.sleep()
        frame += 1
        frame %= 154
