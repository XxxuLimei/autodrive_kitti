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
    ego_pub = rospy.Publisher('kitti_ego_car', MarkerArray, queue_size=10)
    imu_pub = rospy.Publisher('kitti_imu', Imu, queue_size=10)
    gps_pub = rospy.Publisher('kitti_gps', NavSatFix, queue_size=10)

    rate = rospy.Rate(10)

    df_tracking = read_tracking('/home/xilm/fuxian/auto_drive_kitti/tracking/training/label_02/0000.txt')

    while not rospy.is_shutdown():

        boxes = np.array(df_tracking[df_tracking.frame==frame][['bbox_left', 'bbox_top', 'bbox_right', 'bbox_bottom']])
        types = np.array(df_tracking[df_tracking.frame==frame]['type'])
        
        # 2. read data
        image = read_camera(os.path.join(DATA_PATH, 'image_02/data/%010d.png'%frame))
        point_cloud = read_point_cloud(os.path.join(DATA_PATH, 'velodyne_points/data/%010d.bin'%frame)) # n*4 matrix, including x, y, z, reflection rate
        imu_data = read_imu(os.path.join(DATA_PATH, 'oxts/data/%010d.txt'%frame))

        # 3. publish data
        publish_camera(cam_pub, bridge, image, boxes, types)
        publish_point_cloud(pcl_pub, point_cloud) # doesn't receive reflection rate parameter
        publish_ego_car(ego_pub)
        publish_imu(imu_pub, imu_data)
        publish_gps(gps_pub, imu_data)

        rospy.loginfo("published")
        rate.sleep()
        frame += 1
        frame %= 154
