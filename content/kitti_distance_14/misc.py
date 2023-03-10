#!/usr/bin/env python
from data_utils import *
from publish_utils import *
from kitti_util import *
from collections import deque

def compute_3d_box_cam2(h, w, l, x, y, z, yaw):
    """
    Return: 3xn in cam2 coordinate
    """
    R = np.array([[np.cos(yaw), 0, np.sin(yaw)], [0, 1, 0], [-np.sin(yaw), 0, np.cos(yaw)]])
    x_corners = [l/2, l/2,-l/2, -l/2, l/2, l/2, -l/2, -l/2]
    y_corners = [0, 0, 0, 0, -h, -h, -h, -h]
    z_corners = [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2]
    corners_3d_cam2 = np.dot(R, np.vstack([x_corners, y_corners, z_corners]))
    corners_3d_cam2 += np.vstack([x, y, z])
    return corners_3d_cam2

class Object():
    def __init__(self, center):
        self.location = deque(maxlen=20)
        self.location.appendleft(center)

    def update(self, center, displacement, yaw):
        for i in range(len(self.location)):
            x0, y0 = self.location[i]
            x1 = x0 * np.cos(yaw) + y0 * np.sin(yaw) - displacement
            y1 = -x0 * np.sin(yaw) + y0 * np.cos(yaw)
            self.location[i] = np.array([x1, y1])

        if center is not None:
            self.location.appendleft(center)

    def reset(self):
        self.location = deque(maxlen=20)

def distance_point_to_segment(P, A, B):
    """
    Calculates the min distance of a point P to a segment AB.
    Returns the point Q in AB on which the min distance is reached.
    """
    AP = P - A
    BP = P - B
    AB = B - A
    if np.dot(AB, AP) >= 0 and np.dot(-AB, BP) >= 0:
        return np.abs(np.cross(AB, AP))/np.linalg.norm(AB), np.dot(AB, AP)/np.dot(AB, AB) * AB + A
    d_PA = np.linalg.norm(AP)
    d_PB = np.linalg.norm(BP)
    if d_PA < d_PB:
        return d_PA, A
    return d_PB, B

def min_distance_cuboids(cub1, cub2):
    """
    Computes the minimun distance between two non-overlapping cuboids (3D) of shape (8, 3)
    They are projected to BEV and the minimum distance of the two rectangles are returned.
    """
    minD = 1e5
    for i in range(4): # four points
        for j in range(4): # four sides
            d, Q = distance_point_to_segment(cub1[i, :2], cub2[j, :2], cub2[j+1, :2])
            if d < minD:
                minD = d
                minP = cub1[i, :2]
                minQ = Q
    for i in range(4):
        for j in range(4):
            d, Q = distance_point_to_segment(cub2[i, :2], cub1[j, :2], cub1[j+1, :2])
            if d < minD:
                minD = d
                minP = cub2[i, :2]
                minQ = Q
    return minP, minQ, minD