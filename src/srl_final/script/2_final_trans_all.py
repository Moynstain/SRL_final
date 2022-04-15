#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 该例程订阅两个T265设备的odometry数据，输出空间齐次变换矩阵
#
'''
    代码功能：
        1. 订阅2个T265设备的odometry数据，输出2个T265设备间的齐次变换矩阵
        2. T265设备
            订阅odometry数据：/cam_1/odom/sample
            数据类型：Odometry
            通过roslaunch与T265的序列号，指定眼动仪支架处的设备为1，SRL基座处的设备为2
        3. 输出
            以眼动仪支架处T265_e的坐标系为原点，计算T265_srl的坐标系原点位置
            得到T_trans，使得T265_srl = T_Trans * T265_e
            节点：/cam1_cam2/trans_tf
            数据类型：Float32MultiArray
'''

import rospy
import time
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import message_filters
from scipy.spatial.transform import Rotation as R
np.set_printoptions(suppress=True)
global call_back_idx

##########################--  new  --###########################
# Part 1
# 
# 功能：P(eye_tracker) → P(T265_1)call_back_idx = 0
# 输入：目标点眼动仪的视线坐标（u，v）
# 输出：眼动仪绑定T265下的目标点坐标
# 
# 计算表达式如下
# 式1： P(T265_1) = T(PT1_2_Pet) · P(eye_tracker)
# 式2： T(PT1_2_Pet) = Trans(dx,dy,dz) · Rot(x,90°)
# 经测量，完整安装眼动仪（右手坐标系）支架后，T265的pose center坐标系原点移动距离如下
# [dx, dy, dz] = [98, 17.25, -44.9] mm
# 即原眼动仪坐标系下的点(x, y, z)需要左乘T(PT1_2_Pet)得到T265坐标系下的值
# 
# 补充说明：经查找相关资料，T265在ROS下的包已经调整坐标系至ROS的右手坐标系
# 方向：正面向外-x正；正面向右-y正；正面向上-z正；
# 故坐标系旋转矩阵只需绕x轴逆时针转90°
################################################################

# 眼动仪视野为平面（打地鼠平面场景）时，通过视线位置获取眼动仪坐标系中的空间点坐标
def tracker_xyz_plain(u, v): 
    # 仅限打地鼠场景
    z = pow(gaze_depth**2 - center_depth**2, 0.5)
    dx = u - 0.5
    dy = v - 0.5
    dz = pow(dx**2 + dy**2, 0.5)
    x = dx * z / dz
    y = dy * z / dz
    return x, y

# 右乘变换矩，阵得到目标点在新坐标系下的位置
def tracker_to_T265(x, y, center_depth):
    tracker_point = [x, y, center_depth, 1]
    Rot_x = np.matrix([[1,0,0,0], [0,0,-1,0], [0,1,0,0], [0,0,0,1]])
    Trans = np.matrix([[1,0,0,0], [0,1,0,0], [0,0,1,0], [-98, 44.9, 17.25, 1]])
    T_matrix = np.matmul(Rot_x, Trans)
    T265_point = np.matmul(tracker_point, T_matrix)
    return T265_point  


##############################################################################
# Part 2
# 该部分功能
# 输入：两个T265订阅的Odometry数据
# 输出：两个T265空间中对应的齐次变换矩阵
# 含义：在T265_1中表示T265_2的坐标
##############################################################################

def cam_T_matrix_callback(sub1, sub2):
    global call_back_idx
    # fs = 200
    call_back_idx += 1
    call_back_idx %= 200
    #print(call_back_idx)
    if 0 == call_back_idx:
        ##############################################################################
        # 将T265的odometry数据转化为对应的矩阵
        # print sub1
        sub1 = cam_pose_callback(sub1)
        # print sub1
        # space_pos_1 = np.vstack((sub1.data[0:3], sub1.data[]))
        # print sub1.data
        # print "cam1_matrix: || ", msg1
        sub2 = cam_pose_callback(sub2)
        # print sub2
        # print sub1.data
        ##############################################################################
        # 测试cam1和cam2的空间位置(x,y,z)

        # position1 = np.hstack((sub1.data[3], sub1.data[7], sub1.data[11]))
        # position2 = np.hstack((sub2.data[3], sub2.data[7], sub2.data[11]))

        ##############################################################################
        # 初始化cam1和cam2的位置关系，均为相对世界坐标系下的点
        # 设定cam1和cam2的偏置，初始化时cam1在下，cam2在上，仅保留z轴偏置
        world_cam1_init = [0, 0, 0]
        world_cam2_init = np.mat([0, 0, 0.245, 1])
        world_cam1 = np.around(np.vstack((sub1.data[0:4], sub1.data[4:8], sub1.data[8:12], sub1.data[12:16])), decimals=4)
        world_cam2 = np.around(np.vstack((sub2.data[0:4], sub2.data[4:8], sub2.data[8:12], sub2.data[12:16])), decimals=4)
        # print world_cam1
        world_cam1 = np.mat(sub1.data)
        world_cam2 = np.mat(sub2.data)
        # print world_cam2
        # 求T(W_C1)和T(W_C2)的逆矩阵，用于求T(C1_C2)的变换关系
        # 变换关系思路
        # 已知T(W_C1)和T(W_C2)，求T(C1_C2)
        # T(C1_C2) = T(C1_W) * T(W_C2) = inverse[ T(C1_W) ] * P(W1_W2) * T(W_C2)
        inver_w_cam1 = np.around(world_cam1.I, decimals=4)
        # inver_w_cam2 = np.around(np.mat(world_cam2).I, decimals=4)    # 不需要求cam2的逆矩阵
        cam_tf = np.around(np.matmul(inver_w_cam1, world_cam2), decimals=4)
        cam_tf = np.linalg.inv(cam_tf)
        cam_tf[0:3, 3] = cam_tf[0:3, 3]  * 1000 # 单位转化成mm
        cam_tf[2,3] = cam_tf[2,3] - 30
        # print cam_tf
        tf_pub = rospy.Publisher('/cam1_cam2/trans_tf', Float32MultiArray, queue_size=2)
        pub_data = Float32MultiArray()
        # pub_data.data = cam_tf
        pub_data.data = np.around(np.hstack((cam_tf[0], cam_tf[1], cam_tf[2], cam_tf[3])), decimals = 4)
        # rospy.sleep(0.5)
        tf_pub.publish(pub_data)
        print str(cam_tf)
        ##############################################################################
        # ------------------------------------
        # 测试TF对点的转换效果 2020.10.19 基本成功
        test_point_1 = np.vstack([0, 0.5, 0, 1])
        # test_point_1 = np.vstack([0, 0, 0, 1])
        test_point_2 = np.around(np.matmul(cam_tf, test_point_1), decimals=4)
        # print str(cam_tf)
        #　print str(test_point_2), '(单位：mm)'

        # ------------------------------------
        # 测试眼动仪——SRL的转换效果 2020.10.19/20 成功
        test_point_eye = np.vstack([0, 0, 800, 1])
        x = y = 0
        center_depth = 800
        T265_1_point = tracker_to_T265(x, y, center_depth)
        t1 = T265_1_point.T
        # print t1
        # t1 = np.vstack([t1[0], t1[1], t1[2], t1[3]])
        # print T265_1_point
        T265_2_point = np.matmul(cam_tf, t1)
        SRL_point = np.around(T265_2_to_SRL(T265_2_point.T), decimals=5)
        # print SRL_point

        # ------------------------------------
        #


        # print str(cam_tf)
        # print str(np.vstack(test_point_2[:,3]))


##############################################################################
# Part 3
# 该部分功能
# 输入：
# 输出：
# 含义：T265变换到SRL坐标
##############################################################################

def T265_2_to_SRL(point):  # unit: mm
    rot_y = np.mat([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])
    rot_z = np.mat([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    Trans = np.mat([[1, 0, 0, 0], [0, 1, 0, 0], [
                   0, 0, 1, 0], [0, 80.15, -5.95, 1]])
    T_matrix = rot_y * rot_z * Trans
    SRL_point = point * T_matrix
    return SRL_point


def cam_subscriber():
    rospy.init_node('trans_subscriber', anonymous=True)
    cam1_data = message_filters.Subscriber("/cam_1/odom/sample", Odometry)
    cam2_data = message_filters.Subscriber("/cam_2/odom/sample", Odometry)
    print 'step 1'
    sub_sync = message_filters.ApproximateTimeSynchronizer([cam1_data,cam2_data], 10, 0.1, allow_headerless=True)
    global call_back_idx
    call_back_idx = 0
    sub_sync.registerCallback(cam_T_matrix_callback)
    print 'step 2'
    rospy.spin()
    # print cam1_cam2
    # return cam1_cam2

##########################################################
# 小节分割
# 以下为cam的数据订阅与处理函数
##########################################################

def cam1_subscriber():
    rospy.Subscriber("/cam_1/odom/sample", Odometry, cam_pose_callback)
    rospy.spin()
def cam2_subscriber():
    rospy.Subscriber("/cam_2/odom/sample", Odometry, cam_pose_callback)
    rospy.spin()

def cam_pose_callback(msg):
    quat = msg.pose.pose.orientation
    distance = msg.pose.pose.position
    trans_matrix = get_trans_matrix(quat, distance)
    cam_data = Float32MultiArray()
    cam_data.data = trans_matrix
    # print(trans_matrix)
    #np.hstack((trans_matrix[0], trans_matrix[1], trans_matrix[2], trans_matrix[3]))
    # rate = rospy.Rate(500)
    # rate.sleep()
    # print cam_data
    return cam_data

# 通过四元数算旋转矩阵
def get_trans_matrix(quat, distance):
    r = R.from_quat([quat.x, quat.y, quat.z, quat.w])
    trans_matrix = np.eye(4)
    trans_matrix[:3, :3] = r.as_dcm()
    distance = [round(distance.x, 4), round(distance.y, 4), round(distance.z, 4)]
    trans_matrix[:3, 3] = distance
    return trans_matrix




if __name__ == '__main__':
    cam_subscriber()
    
    # print cam1_cam2
    #pose_trans()
