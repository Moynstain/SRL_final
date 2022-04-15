#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 该例程订阅两个T265设备的odometry数据，输出空间齐次变换矩阵
#
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, UInt8
import message_filters
from scipy.spatial.transform import Rotation as R
np.set_printoptions(suppress=True)
from interbotix_sdk.robot_manipulation import InterbotixRobot
from interbotix_descriptions import interbotix_mr_descriptions as mrd

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
################################################

def point_trans(EMG_sim, cam_tf):
    tf = cam_tf
    EMG = EMG_sim
    print EMG.data
    tf = np.around(np.vstack((tf.data[0:4], tf.data[4:8], tf.data[8:12], tf.data[12:16])), decimals=4)
    T265_tf = np.mat(tf)

    # p_eye = np.mat([0, 300, 0, 1])
    # p_T265_1 = tracker_to_T265(p_eye.T)
    # print p_eye
    # p_T265_1 = np.mat([100, 320, 0, 1])
    # p_T265_2 = np.around(np.matmul(T265_tf, p_T265_1.T), decimals=5)
    # p_SRL = np.around(T265_2_to_SRL(p_T265_2.T), decimals=5)
    # print p_SRL
    # SRL_x = p_SRL[0][0] / 1000
    # SRL_y = p_SRL[0][1] / 1000
    # SRL_z = p_SRL[0][2] / 1000
    # print SRL_x, SRL_y, SRL_z
    # msg = raw_input("Command Next Move")
    # arm.set_ee_pose_components(x=SRL_x, y=SRL_y, z=SRL_z, roll=0, pitch=0,blocking=False)
    # arm.close_gripper()
    

    # print "while loop"
    
    # msg = raw_input("enter a command: ")    
    
    if EMG.data == 1:
        print "EMG signal arrived... "
        for i in range(0,5):
            p_eye = np.mat([0, 30*i, 500 + 30*i, 1])
            p_T265_1 = tracker_to_T265(p_eye.T)
            p_T265_2 = np.around(np.matmul(T265_tf, p_T265_1.T), decimals=5)
            p_SRL = np.around(T265_2_to_SRL(p_T265_2.T), decimals=5)
            SRL_x = p_SRL[0][0] / 1000
            SRL_y = p_SRL[0][1] / 1000
            SRL_z = p_SRL[0][2] / 1000

            # msg = raw_input("open the gripper? ")
            # if msg == "o":
            arm.open_gripper()
            arm.set_ee_pose_components(x=SRL_x-0.1, y=SRL_y, z=SRL_z-0.1, roll=0, pitch=-np.pi/4,blocking=True)
            arm.set_ee_pose_components(x=SRL_x, y=SRL_y, z=SRL_z, roll=0, pitch=-np.pi/4,blocking=True)
            arm.set_ee_pose_components(x=SRL_x-0.1, y=SRL_y, z=SRL_z-0.1, roll=0, pitch=-np.pi/4,blocking=True)
            # msg = raw_input("close the gripper? ")
            # if msg == "c":
            # arm.close_gripper()
            break






def cam_subscriber():
    

    EMG_sim = message_filters.Subscriber("/EMG_sim", UInt8)
    cam_tf = message_filters.Subscriber("/cam1_cam2/trans_tf", Float32MultiArray)
    sub_sync = message_filters.ApproximateTimeSynchronizer([EMG_sim, cam_tf], 10, 1, allow_headerless=True)
    sub_sync.registerCallback(point_trans)
    # rospy.Rate(5)

    rospy.spin()



if __name__ == '__main__':
    # rospy.init_node('SRL_follow', anonymous=True)
    arm = InterbotixRobot(robot_name="vx300", mrd=mrd, moving_time=2, accel_time=1)
    arm.go_to_home_pose()

    cam_subscriber()    
    