#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, Imu, NavSatFix
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import tf
import numpy as np
import cv2

FRAME_ID = 'map'
DETECTION_COLOR_DICT = {'Car':(255,255,0), 'Pedestrian':(0,226,255), 'Cyclist':(141,40,255)}
LIFETIME = 0.1
LINES = [[0, 1], [1, 2], [2, 3], [3, 0]] # lower face
LINES += [[4, 5], [5, 6], [6, 7], [7, 4]] # upper face
LINES += [[4, 0], [5, 1], [6, 2], [7, 3]] # height lines
LINES += [[4, 1], [5, 0]] #front face

def publish_camera(cam_pub, bridge, image, boxes, types, track_ids):
    for typ, box, track_id in zip(types, boxes, track_ids):
        
        top_left = int(box[0]), int(box[1]) # x and y coordinate
        bottom_right = int(box[2]), int(box[3])

        cv2.rectangle(image, top_left, bottom_right, DETECTION_COLOR_DICT[typ], 2)
        font=cv2. FONT_HERSHEY_SIMPLEX
        cv2. putText(image, str(track_id), (top_left), font, 1, DETECTION_COLOR_DICT[typ], 2, cv2.LINE_AA)
    cam_pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))

def publish_point_cloud(pcl_pub, point_cloud):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = FRAME_ID
    pcl_pub.publish(pcl2.create_cloud_xyz32(header, point_cloud[:, :3]))

def publish_ego_car(ego_car_pub):
    """
    publish left and right 45 degree FOV lines and ego car model mesh
    """
    marker_array = MarkerArray()

    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.header.stamp = rospy.Time.now()

    marker.id = 0
    marker.action = Marker.ADD
    marker.lifetime = rospy.Duration() # always here, so none
    marker.type = Marker.LINE_STRIP

    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0 # alpha
    marker.scale.x = 0.1 # line scale

    marker.points = []
    marker.points.append(Point(10, -10, 0))
    marker.points.append(Point(0, 0, 0))
    marker.points.append(Point(10, 10, 0))

    marker_array.markers.append(marker)

    mesh_marker = Marker()
    mesh_marker.header.frame_id = FRAME_ID
    mesh_marker.header.stamp = rospy.Time.now()

    mesh_marker.id = -1
    mesh_marker.lifetime = rospy.Duration()
    mesh_marker.type = Marker.MESH_RESOURCE
    mesh_marker.mesh_resource = "package://kitti_tutorial/bmw_x5/BMW X5 4.dae"

    mesh_marker.pose.position.x = 0.0
    mesh_marker.pose.position.y = 0.0
    mesh_marker.pose.position.z = -1.73

    q = tf.transformations.quaternion_from_euler(np.pi/2, 0, np.pi)
    mesh_marker.pose.orientation.x = q[0]
    mesh_marker.pose.orientation.y = q[1]
    mesh_marker.pose.orientation.z = q[2]
    mesh_marker.pose.orientation.w = q[3]

    mesh_marker.color.r = 1.0
    mesh_marker.color.g = 1.0
    mesh_marker.color.b = 1.0
    mesh_marker.color.a = 1.0

    mesh_marker.scale.x = 0.9
    mesh_marker.scale.y = 0.9
    mesh_marker.scale.z = 0.9

    marker_array.markers.append(mesh_marker)

    ego_car_pub.publish(marker_array)

def publish_imu(imu_pub, imu_data):
    imu = Imu()
    imu.header.frame_id = FRAME_ID
    imu.header.stamp = rospy.Time.now()

    q = tf.transformations.quaternion_from_euler(float(imu_data.roll), float(imu_data.pitch), float(imu_data.yaw))
    imu.orientation.x = q[0]
    imu.orientation.y = q[1]
    imu.orientation.z = q[2]
    imu.orientation.w = q[3]
    imu.linear_acceleration.x = imu_data.af
    imu.linear_acceleration.y = imu_data.al
    imu.linear_acceleration.z = imu_data.au
    imu.angular_velocity.x = imu_data.wf
    imu.angular_velocity.y = imu_data.wl
    imu.angular_velocity.z = imu_data.wu

    imu_pub.publish(imu)

def publish_gps(gps_pub, imu_data):
    gps = NavSatFix()
    gps.header.frame_id = FRAME_ID
    gps.header.stamp = rospy.Time.now()

    gps.latitude = imu_data.lat
    gps.longitude = imu_data.lon
    gps.altitude = imu_data.alt

    gps_pub.publish(gps)

def publish_3dbox(box3d_pub, corners_3d_velos, types, track_ids):
    marker_array = MarkerArray()
    for i, corners_3d_velo in enumerate(corners_3d_velos):
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.header.stamp = rospy.Time.now()

        marker.id = i
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(LIFETIME)
        marker.type = Marker.LINE_LIST

        b, g, r = DETECTION_COLOR_DICT[types[i]]
        marker.color.r = r/255.0
        marker.color.g = g/255.0
        marker.color.b = b/255.0

        marker.color.a = 1.0

        marker.scale.x = 0.1

        marker.points = []
        for l in LINES:
            p1 = corners_3d_velo[l[0]]
            marker.points.append(Point(p1[0], p1[1], p1[2]))
            p2 = corners_3d_velo[l[1]]
            marker.points.append(Point(p2[0], p2[1], p2[2]))
        marker_array.markers.append(marker)

        text_marker = Marker()
        text_marker.header.frame_id = FRAME_ID
        text_marker.header.stamp = rospy.Time.now()

        text_marker.id = i + 1000
        text_marker.action = Marker.ADD
        text_marker.lifetime = rospy.Duration(LIFETIME)
        text_marker.type = Marker.TEXT_VIEW_FACING

        p4 = np.mean(corners_3d_velo, axis=0) # upper front left corner

        text_marker.pose.position.x = p4[0]
        text_marker.pose.position.y = p4[1]
        text_marker.pose.position.z = p4[2] + 1.2

        text_marker.text = str(track_ids[i])

        text_marker.scale.x = 1
        text_marker.scale.y = 1
        text_marker.scale.z = 1

        if types is None:
            text_marker.color.r = 0.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
        else:
            b, g, r = DETECTION_COLOR_DICT[types[i]]
            text_marker.color.r = r/255.0
            text_marker.color.g = g/255.0
            text_marker.color.b = b/255.0
        text_marker.color.a = 1.0
        marker_array.markers.append(text_marker)
    box3d_pub.publish(marker_array)

def publish_loc(loc_pub, location):
    marker_array = MarkerArray()

    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.header.stamp = rospy.Time.now()

    marker.action = Marker.ADD
    marker.lifetime = rospy.Duration()
    marker.type = Marker.LINE_STRIP

    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.scale.x = 0.2

    marker.points = []
    for p in location:
        marker.points.append(Point(p[0], p[1], 0)) # x, y, and z coordinate

    marker_array.markers.append(marker)
    loc_pub.publish(marker_array)