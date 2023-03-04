#!/usr/bin/env python
from data_utils import *
from publish_utils import *
import os

DATA_PATH = '/home/xilm/fuxian/auto_drive_kitti/RawData/2011_09_26/2011_09_26_drive_0005_sync/'

if __name__ == '__main__':
    frame = 0
    rospy.init_node('kitti_node', anonymous=True)
    
    # 1. create publisher
    cam_pub = rospy.Publisher('kitti_cam', Image, queue_size=10)
    pcl_pub = rospy.Publisher('kitti_point_cloud', PointCloud2, queue_size=10)
    bridge = CvBridge()
    ego_pub = rospy.Publisher('kitti_ego_car', Marker, queue_size=10)
    model_pub = rospy.Publisher('kitti_car_model', Marker, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        # 2. read data
        image = read_camera(os.path.join(DATA_PATH, 'image_02/data/%010d.png'%frame))
        point_cloud = read_point_cloud(os.path.join(DATA_PATH, 'velodyne_points/data/%010d.bin'%frame)) # n*4 matrix, including x, y, z, reflection rate
        
        # 3. publish data
        publish_camera(cam_pub, bridge, image)
        publish_point_cloud(pcl_pub, point_cloud) # doesn't receive reflection rate parameter
        publish_ego_car(ego_pub)
        publish_car_model(model_pub)

        rospy.loginfo("published")
        rate.sleep()
        frame += 1
        frame %= 154
