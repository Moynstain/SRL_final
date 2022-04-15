#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 该例程订阅两个T265设备的odometry数据，输出空间齐次变换矩阵
#
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import message_filters
from scipy.spatial.transform import Rotation as R
np.set_printoptions(suppress=True)

################################################
# 右乘变换矩，阵得到目标点在新坐标系下的位置
def tracker_to_T265(point):
    tracker_point = np.mat(point)
    # print point
    Rot_x = np.mat([[1,0,0,0], [0,0,-1,0], [0,1,0,0], [0,0,0,1]])
    Trans = np.mat([[1,0,0,0], [0,1,0,0], [0,0,1,0], [-98, 44.9, 17.25, 1]])
    T_matrix = np.matmul(Rot_x, Trans)
    T265_point = np.matmul(tracker_point.T, T_matrix)
    return T265_point  

def T265_2_to_SRL(point):  # unit: mm
    rot_y = np.mat([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])
    rot_z = np.mat([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    Trans = np.mat([[1, 0, 0, 0], [0, 1, 0, 0], [
                   0, 0, 1, 0], [0, 80.15, -5.95, 1]])
    T_matrix = rot_y * rot_z * Trans
    SRL_point = point * T_matrix
    return SRL_point

def point_trans(depth, tf):
    depth = np.around(np.vstack((depth.data[0], depth.data[1], depth.data[2], 1)), decimals=4)
    tf = np.around(np.vstack((tf.data[0:4], tf.data[4:8], tf.data[8:12], tf.data[12:16])), decimals=4)
    eye_depth = np.mat(depth)
    T265_tf = np.mat(tf)

    p_eye = eye_depth
    p_T265_1 = tracker_to_T265(p_eye)
    # print p_eye
    p_T265_2 = np.around(np.matmul(T265_tf, p_T265_1.T), decimals=5)
    p_SRL = np.around(T265_2_to_SRL(p_T265_2.T), decimals=5)
    print p_SRL

def cam_subscriber():
    rospy.init_node('SRL_exp', anonymous=True)
    eye_depth = message_filters.Subscriber("/eye_tracker/gaze_to_depth", Float32MultiArray)
    trans_tf = message_filters.Subscriber("/cam1_cam2/trans_tf", Float32MultiArray)

    sub_sync = message_filters.ApproximateTimeSynchronizer([eye_depth,trans_tf], 10, 0.1, allow_headerless=True)
    sub_sync.registerCallback(point_trans)
    rospy.spin()



if __name__ == '__main__':
    cam_subscriber()    
    print 'step 3'
    gaze_depth = message_filters.Subscriber("/eye_tracker/gaze_to_depth", Float32MultiArray)
    print str(gaze_depth)