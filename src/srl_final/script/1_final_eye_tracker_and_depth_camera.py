#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
    2022/4/15
    代码功能：
        1. 深度相机
            订阅奥比中光astra深度相机的深度信息，/camera/depth/points与/camera/rgb/image_rect_color
            将rgb信息与深度点云信息标定后获取某RGB像素处的深度信息
        2. 眼动仪
            获取眼动仪输出的RGB信息与视线位置信息
            经棋盘格标定，把眼动仪与深度相机的RGB坐标系对齐，获取视线位置处深度信息
        3. 输出
            眼动仪坐标系下的（x, y, depth）信息
            节点：/eye_tracker/gaze_to_depth
            数据：Float32MultiArray
'''

import rospy
import cv2
import time
import sys
import message_filters  # 同步订阅消息
import utils  # 贶恩写的utils，主要用于处理眼动仪与深度相机数据
import json  # 眼动仪与深度相机之间的变换矩阵
import numpy as np
from std_msgs.msg import Header, Float32MultiArray
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge, CvBridgeError
from tobiiglassesctrl import TobiiGlassesController
np.set_printoptions(suppress=True)

print("orbbec_frame")

''' func 1. read_dic_from_json
    用于读取include/device_assist_env/rgbd_tracker_paras.json文件
    深度相机经棋盘格标定后的校准文件
'''
def read_dic_from_json():
    with open('include/device_assist_env/rgbd_tracker_paras.json', 'r') as json_file:
        data = json.load(json_file)
        for key in data.keys():
            data[key] = np.asarray(data[key])
        return data


''' func 2. tracker_to_rgbd_xyz
    gaze 2d: u in [0, 1], v in [0, 1]
    image depth: unit mm
'''
def tracker_to_rgbd_xyz(gaze_2d, img_depth):
    uv_in_tracker = gaze_2d * np.asarray([1920, 1080])
    rgbd_tracker_paras = read_dic_from_json()
    u, v = utils.tracker_to_rgbd_without_depth(
        uv_in_tracker, img_depth, rgbd_tracker_paras)
    d = img_depth[v, u]
    return utils.uvz2xyz(u, v, d), (u, v)


# 以下为刘海原用于采集深度相机的数据，包含数据处理及订阅的节点
# prefix to the names of dummy fields we add to get byte alignment correct. this needs to not
# clash with any actual field names
DUMMY_FIELD_PREFIX = '__'

# mappings between PointField types and numpy types
type_mappings = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')), (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')), (PointField.INT32,
                                                           np.dtype('int32')), (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]
pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)

# sizes (in bytes) of PointField types
pftype_sizes = {PointField.INT8: 1, PointField.UINT8: 1, PointField.INT16: 2, PointField.UINT16: 2,
                PointField.INT32: 4, PointField.UINT32: 4, PointField.FLOAT32: 4, PointField.FLOAT64: 8}

# @converts_to_numpy(PointField, plural=True)


def fields_to_dtype(fields, point_step):
    '''Convert a list of PointFields to a numpy record datatype.
    '''
    offset = 0
    np_dtype_list = []
    for f in fields:
        while offset < f.offset:
            # might be extra padding between fields
            np_dtype_list.append(
                ('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        dtype = pftype_to_nptype[f.datatype]
        if f.count != 1:
            dtype = np.dtype((dtype, f.count))

        np_dtype_list.append((f.name, dtype))
        offset += pftype_sizes[f.datatype] * f.count

    # might be extra padding between points
    while offset < point_step:
        np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
        offset += 1

    return np_dtype_list


def pointcloud2_to_array(cloud_msg, squeeze=True):
    ''' Converts a rospy PointCloud2 message to a numpy recordarray

    Reshapes the returned array to have shape (height, width), even if the height is 1.
    The reason for using np.frombuffer rather than struct.unpack is speed... especially
    for large point clouds, this will be <much> faster.
    '''
    # construct a numpy record type equivalent to the point type of this cloud
    dtype_list = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)

    # parse the cloud into an array
    cloud_arr = np.frombuffer(cloud_msg.data, dtype_list)

    # remove the dummy fields that were added
    cloud_arr = cloud_arr[
        [fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]

    if squeeze and cloud_msg.height == 1:
        return np.reshape(cloud_arr, (cloud_msg.width,))
    else:
        return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))


def get_xyz_points(cloud_array, remove_nans=True, dtype=np.float):
    '''Pulls out x, y, and z columns from the cloud recordarray, and returns
        a 3xN matrix.
    '''
    # remove crap points
    if remove_nans:
        mask = np.isfinite(cloud_array['x']) & np.isfinite(
            cloud_array['y']) & np.isfinite(cloud_array['z'])
        cloud_array = cloud_array[mask]

    # pull out x, y, and z values
    points = np.zeros(cloud_array.shape + (3,), dtype=dtype)
    points[..., 0] = cloud_array['x']
    points[..., 1] = cloud_array['y']
    points[..., 2] = cloud_array['z']

    return points


def pointcloud2_to_xyz_array(cloud_msg, remove_nans=False):
    return get_xyz_points(pointcloud2_to_array(cloud_msg), remove_nans=remove_nans)


'''
    func. read_and_save_data
    ROS Subscriber回调函数中实现功能的部分
    将深度相机point cloud、眼动仪rgb picture、眼动仪gaze position处理，并输出gaze_position处的深度信息
    详细步骤如下描述
    已知：深度相机可输出【深度点云信息】、【rgb图像信息】；眼动仪可输出【rgb图像信息】、【基于rgb图像坐标系的视线信息】
    步骤一：预先处理深度相机的点云信息与rgb图像信息，获取深度相机任意rgb坐标处的深度数据
    步骤二：将深度相机的rgb图像信息与眼动仪的rgb图像信息标定，获取眼动仪任意rgb坐标处的深度数据
    步骤三：通过眼动仪的视线坐标信息，获取其对应的深度数据
'''
def read_and_save_data(cloud_msg, rgb_msg, bridge, cap):
    # print "read and save data"
    t = time.time()
    try:
        ret, frame = cap.read()
        height, width = frame.shape[:2]
        t = time.time()
        gaze = tobiiglasses.get_data()['gp'] # gaze为眼动仪的实际视线坐标数据header
        # print gaze
        test = gaze.has_key('gp')
        if test:
            data_gp = gaze['gp'] # 获取视线数据中的['gaze_position']数据

            rgb_image = bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            cloud = pointcloud2_to_xyz_array(cloud_msg)
            cloud[np.isnan(cloud)] = 0

            img_depth = (cloud[..., 2]*1000).astype(np.uint16)
            # print img_depth
            gaze_2d = data_gp # gaze_2d为导入的2D眼动数据，img_depth为导入的奥比中光相机深度图
            xyz_gp, uv_in_depth = tracker_to_rgbd_xyz(gaze_2d, img_depth)

            gaze_center = [0.5, 0.5] # 眼动仪坐标系原点，位于视野正中心处
            xyz_center, uv_in_depth = tracker_to_rgbd_xyz(gaze_center, img_depth)
            ##########################################################
            # 这一段用于将视线数据转化为眼动仪下的坐标
            x_gp, y_gp, z_gp = xyz_gp[0], xyz_gp[1], xyz_gp[2]
            x_center, y_center, z_center = xyz_center[0], xyz_center[1], xyz_center[2]
            dx = np.around(x_gp - x_center, decimals=4)
            dy = np.around(y_gp - y_center, decimals=4)
            dz = np.around(z_gp, decimals=4)
            xyz = [dx, dy, dz] # 获取眼动仪坐标系下，视线位置的空间坐标
            ##########################################################
            # 这一段用于将获取的“视线-距离”信息发布到ros中
            xyz_pub = rospy.Publisher('/eye_tracker/gaze_to_depth', Float32MultiArray, queue_size=10)
            pub_data = Float32MultiArray()
            pub_data.data = np.around(np.hstack((dx, dy, dz)), decimals=5)
            xyz_pub.publish(pub_data)

            print('xyz: {} mm'.format(xyz))
        # print xyz
        # print xyz, xyz_gp


    except CvBridgeError as e:
        print(e)


#########################################################################
# 主调用函数
def listener(cap):
    rospy.init_node('tracker_to_SRL')
    print "1234567890"
    cloud_msg = message_filters.Subscriber('/camera/depth/points', PointCloud2)
    print 'message subscribed'
    bridge = CvBridge()
    rgb_msg = message_filters.Subscriber('/camera/rgb/image_rect_color', Image)
    print '9876543'
    fs = 10  # Hz
    rate = rospy.Rate(fs)  # 3 Hz
    ts = message_filters.ApproximateTimeSynchronizer([cloud_msg, rgb_msg], fs, 10)

    ts.registerCallback(read_and_save_data, bridge, cap)
    print 'callback function registered'

    rospy.spin()

#########################################################################
# 眼动仪视野为平面（打地鼠平面场景）时，通过视线位置获取眼动仪坐标系中的空间点坐标


def tracker_xyz_plain(u, v, gaze_depth, center_depth):  # unit: mm
    z = center_depth
    # z = pow(gaze_depth**2 - center_depth**2, 0.5)
    dx = u - 0.5
    dy = v - 0.5
    dz = pow(dx**2 + dy**2, 0.5)
    x = dx * z / dz
    y = dy * z / dz
    return x, y


def tracker_to_T265_1(x, y, center_depth):  # unit: mm
    tracker_point = [x, y, center_depth, 1]
    Rot_x = np.matrix([[1, 0, 0, 0], [0, 0, -1, 0],
                       [0, 1, 0, 0], [0, 0, 0, 1]])
    Trans = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [
        0, 0, 1, 0], [-98, 17.25, -44.9, 1]])
    T_matrix = Rot_x * Trans
    T265_coo = np.dot(tracker_point, T_matrix)
    return T265_coo


def T265_tf_SRL(msg):  # unit: mm
    disp = np.around(
        np.vstack((msg.data[3], msg.data[7], msg.data[11], msg.data[15])), decimals=4)
    rot_r1 = np.vstack((msg.data[0], msg.data[4], msg.data[8], msg.data[12]))
    rot_r2 = np.vstack((msg.data[1], msg.data[5], msg.data[9], msg.data[13]))
    rot_r3 = np.vstack((msg.data[2], msg.data[6], msg.data[10], msg.data[14]))
    rot = np.around(np.hstack((rot_r1, rot_r2, rot_r3)), decimals=4)

    T265_tf = np.mat(np.hstack((rot, disp)))
    # T265_tf[:,3] = T265_tf[:,3] * 1000
    return T265_tf


def T265_2_to_SRL(point):  # unit: mm
    rot_y = np.mat([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])
    rot_z = np.mat([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    Trans = np.mat([[1, 0, 0, 0], [0, 1, 0, 0], [
        0, 0, 1, 0], [0, 80.15, -5.95, 1]])
    T_matrix = np.matmul(rot_y, rot_z, Trans)
    SRL_point = np.matmul(point, T_matrix)
    return SRL_point


if __name__ == '__main__':
    try:
        #########################################################################
        # 以下为眼动仪获取视线坐标，实现位置功能
        # gaze_2d = data_gp
        ipv4_address = "192.168.71.50"
        tobiiglasses = TobiiGlassesController(ipv4_address, video_scene=True)
        project_id = tobiiglasses.create_project("Test live_scene_and_gaze.py")
        participant_id = tobiiglasses.create_participant(
            project_id, "participant_test")
        calibration_id = tobiiglasses.create_calibration(
            project_id, participant_id)
        raw_input(
            "Put the calibration marker in front of the user, then press enter to calibrate")
        tobiiglasses.start_calibration(calibration_id)
        res = tobiiglasses.wait_until_calibration_is_done(calibration_id)

        if res is False:
            print("Calibration failed!")
            exit(1)

        cap = cv2.VideoCapture("rtsp://%s:8554/live/scene" % ipv4_address)
        # Check if camera opened successfully
        if (cap.isOpened() == False):
            print("Error opening video stream or file")

        # Read until video is completed
        tobiiglasses.start_streaming()
        print("Start streaming. Please wait ...")
        listener(cap)

        #########################################################################
        # listener()获取深度相机距离
        # listener()
    except rospy.ROSInterruptException:
        pass
