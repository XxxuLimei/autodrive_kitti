#!/usr/bin/env python

import cv2
import numpy as np
import pandas as pd

IMU_COLUMN_NAMES = ['lat', 'lon', 'alt', 'roll', 'pitch', 'yaw', 'vn',
                    've', 'vf', 'vl', 'vn', 'ax', 'ay', 'az', 'af',
                    'al', 'au', 'wx', 'wy', 'wz', 'wf', 'wl', 'wu',
                    'posacc', 'velacc', 'navstat', 'numstas',
                    'posmode', 'velmode', 'orimode']

def read_camera(path):
    return cv2.imread(path)

def read_point_cloud(path):
    return np.fromfile(path, dtype=np.float32).reshape(-1, 4)

def read_imu(path):
    df = pd.read_csv(path, header=None, sep=' ')
    df.columns = IMU_COLUMN_NAMES
    return df