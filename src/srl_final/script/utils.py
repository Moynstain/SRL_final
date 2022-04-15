import numpy as np
import cv2
import open3d as o3d
import pandas as pd
import matplotlib.pyplot as plt
import numpy.matlib as npm
# import pytesseract
# import re
import glob
import os
import pywt

from shutil import copyfile
from matplotlib.backends.backend_pdf import FigureCanvasPdf as FigureCanvas


'''
This is for the current realsense camera. This parameter should be changed if the camera is changed.
'''

# default_cam_intri = np.array([[586.32717732,   0.        , 333.51347533],
#                               [  0.        , 583.33604217, 259.09226867],
#                               [  0.        ,   0.        ,   1.        ]])

# default_cam_intri = np.asarray([[609.58812653,   0.        , 332.94098057],
#                                 [  0.        , 608.71303531, 250.79214385],
#                                 [  0.        ,   0.        ,   1.        ]])

# default_cam_intri = np.asarray(
# [[508.675526  ,   0.        , 313.98135889],
# [ 0.        , 507.5633895 ,  242.97669603 ],
# [  0.        ,   0.        ,   1.        ]])

default_cam_intri = np.asarray(
[[521.85359567,   0.        , 321.18647073],
[0.        , 521.7098714 , 233.81475134],
[0.        ,   0.        ,   1.        ]])




def uvz2xyz(u, v, d, cam_intri_inv= None):
    if cam_intri_inv is None:
        cam_intri_inv = np.linalg.inv(default_cam_intri)
    uvd_vec = np.asarray([u, v, 1]) * d
    xyz = np.matmul(cam_intri_inv, uvd_vec)
    return xyz


def depth2cloud(depth, cam_intri_inv = None):
    # default unit of depth and point cloud is mm.
    if cam_intri_inv is None:
        cam_intri_inv = np.zeros((1, 1, 3, 3))
        cam_intri_inv[0, 0] = np.linalg.inv(default_cam_intri)

    uv_mat = np.ones((480, 640, 3, 1))
    uv_mat[:, :, :2, 0] = np.transpose(np.mgrid[0:480, 0:640], (1, 2, 0))[:, :, [1, 0]]
    depth = np.reshape(depth, (480, 640, 1, 1))
    uvd_mat = uv_mat * depth
    point_cloud = np.matmul(cam_intri_inv, uvd_mat)
    point_cloud = point_cloud.reshape((-1, 3))
    return point_cloud


def visualize_rgbd_point_cloud(depth_raw, color_raw):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(depth2cloud(depth_raw))
    pcd.colors = o3d.utility.Vector3dVector((color_raw.astype(np.float) / 255.0).reshape((-1, 3)))
    o3d.visualization.draw_geometries([pcd])



# Q is a Nx4 numpy matrix and contains the quaternions to average in the rows.
# The quaternions are arranged as (w,x,y,z), with w being the scalar
# The result will be the average quaternion of the input. Note that the signs
# of the output quaternion can be reversed, since q and -q describe the same orientation
def averageQuaternions(Q):
    # Number of quaternions to average
    M = Q.shape[0]
    A = np.zeros(shape=(4,4))

    for i in range(0,M):
        q = Q[i,:]
        # multiply q with its transposed version q' and add A
        A = npm.outer(q,q) + A

    # scale
    A = (1.0/M)*A
    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = np.linalg.eig(A)
    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]
    # return the real part of the largest eigenvector (has only real part)
    return np.real(eigenVectors[:,0])


# Average multiple quaternions with specific weights
# The weight vector w must be of the same length as the number of rows in the
# quaternion maxtrix Q
def weightedAverageQuaternions(Q, w):
    # Number of quaternions to average
    M = Q.shape[0]
    A = npm.zeros(shape=(4,4))
    weightSum = 0

    for i in range(0,M):
        q = Q[i,:]
        A = w[i] * np.outer(q,q) + A
        weightSum += w[i]

    # scale
    A = (1.0/weightSum) * A

    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = np.linalg.eig(A)

    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]

    # return the real part of the largest eigenvector (has only real part)
    return np.real(eigenVectors[:,0])


def draw_circles_on_img(img_rgb_tracker, imgpoints_tracker, imgpoints_est_tracker):
    img_rgb_tracker = np.copy(img_rgb_tracker)
    for r in range(imgpoints_tracker.shape[0]):
        img_rgb_tracker = cv2.circle(img_rgb_tracker, tuple(imgpoints_tracker[r].astype(np.int)),
                                     radius=10, color= (0, 255, 0),thickness = 2)
        img_rgb_tracker = cv2.circle(img_rgb_tracker, tuple(imgpoints_est_tracker[r].astype(np.int)),
                                     radius=10, color=(0, 0, 255),thickness = 2)
    return img_rgb_tracker


def interactive_img_show(img):
    fig, ax = plt.subplots()
    im = ax.imshow(img, interpolation='none')
    ax.format_coord = Formatter(im)
    plt.show()


class Formatter(object):
    def __init__(self, im):
        self.im = im

    def __call__(self, x, y):
        z = self.im.get_array()[int(y), int(x)]
        return 'x={:.01f}, y={:.01f}, z={:.01f}'.format(x, y, z)


def read_target_distance(file_name, camera_offset = 15):
    df = pd.read_csv(file_name, sep='\t', skiprows=4).values
    data = df[0:20, :].astype(float)

    eye_pos = data[:, 20:23]
    eye_pos[:, -1] -= camera_offset
    target_pos = data[:, 11:14]
    target_distance = np.mean(np.linalg.norm(eye_pos - target_pos, axis=-1))
    return target_distance


def haar_dec(array_2d):
    coeffs = pywt.dwt2(array_2d, 'haar')
    cA, (cH, cV, cD) = coeffs
    return np.r_[np.c_[cA, cH], np.c_[cV, cD]]

def haar_rec(feature_2d):
    rows, cols = feature_2d.shape
    half_row = int(rows / 2)
    half_col = int(cols / 2)
    cA = feature_2d[:half_row, :half_col]
    cH = feature_2d[:half_row, half_col:]
    cV = feature_2d[half_row:, :half_col]
    cD = feature_2d[half_row:, half_col:]
    coeffs = tuple([cA, tuple([cH, cV, cD])])
    return pywt.idwt2(coeffs, 'haar')

def haar_dec_multi(feature_2d, ite_num = 5):
    # for i in range(ite_num):
    #     array_2d = haar_dec(array_2d)
    feature_2d = haar_dec(feature_2d)
    height, width = feature_2d.shape
    for i in range(1, ite_num):
        rows, cols = int(height / 2 ** i), int(width / 2 ** i)
        feature_2d[:rows, :cols] = haar_dec(feature_2d[:rows, :cols])
    return feature_2d

def haar_rec_multi(feature_2d, ite_num = 5):
    # for i in range(ite_num):
    #     feature_2d = haar_rec(feature_2d)
    height, width = feature_2d.shape
    for i in range(ite_num):
        idx = ite_num - 1- i
        rows, cols = int(height / 2 ** idx), int(width / 2 ** idx)
        feature_2d[:rows, :cols] = haar_rec(feature_2d[:rows, :cols])
    return feature_2d

def haar_filter(array_2d, ite_num = 9, threshold = 10):
    feature_width = 2**ite_num
    height, width = array_2d.shape
    feature_2d = np.zeros((feature_width, feature_width))
    init_row = int((feature_width - height) / 2 - 1)
    init_col = int((width - feature_width) / 2 - 1)
    feature_2d[init_row:init_row+height, :] = array_2d[:, init_col:init_col+feature_width]
    feature_2d = haar_dec_multi(feature_2d, ite_num = ite_num)
    feature_2d -= threshold
    feature_2d[0, 0] += threshold
    feature_2d = haar_rec_multi(feature_2d, ite_num=ite_num)
    array_2d[:, init_col:init_col + feature_width] = feature_2d[init_row:init_row+height, :]
    return array_2d

def moving_average_within_threshold(x_t, x_t_plus_1, new_ratio = 0.5 ,threshold = 50):
    zero_indices = np.copy(x_t_plus_1 == 0)
    # valid_indices = np.copy(np.abs(x_t_plus_1-x_t) < threshold)
    # x_t_plus_1[valid_indices] = new_ratio * x_t_plus_1[valid_indices] + (1-new_ratio) * x_t[valid_indices]
    x_t_plus_1[zero_indices] = x_t[zero_indices]
    return x_t_plus_1

def edge_preserving_filter(img):
    rows, cols = img.shape
    for i in range(1):
        for r in range(1, rows):
            img[r, :] = moving_average_within_threshold(img[r-1, :], img[r, :])
        for c in range(1, cols):
            img[:, c] = moving_average_within_threshold(img[:, c-1], img[:, c])
    return img

def plane_fitting(depth_mat):
    radius = int(depth_mat.shape[0] / 2)
    uv_mat = np.ones(((2 * radius) ** 2, 3))

    uv_mat[:, :2] = np.transpose(np.mgrid[0:2 * radius, 0:2 * radius], (1, 2, 0)).reshape((-1, 2))
    depth_vec = depth_mat.reshape((-1, 1))

    plane_coeff = np.matmul(np.linalg.pinv(uv_mat), depth_vec)
    depth_mat = np.matmul(uv_mat, plane_coeff).reshape((2 * radius, 2 * radius))
    return depth_mat


def tracker_to_rgbd_without_depth(uv_tracker, img_depth, rgbd_tracker_paras):
    uv_tracker = uv_tracker.reshape([1, -1])
    cam_matrix_tracker = rgbd_tracker_paras.get('cam_matrix_tracker')
    T_tracker_in_depth = np.linalg.inv(rgbd_tracker_paras.get('T_mat_depth_in_tracker'))

    p_eye_in_depth = cam1_to_cam2_xyz(uv_tracker, np.asarray([0]),
                                      cam1_matrix=cam_matrix_tracker,
                                      T_cam1_in_cam2=T_tracker_in_depth)[:3].T
    p_virtual_point_in_depth = cam1_to_cam2_xyz(uv_tracker, np.asarray([1000]),
                                      cam1_matrix=cam_matrix_tracker,
                                      T_cam1_in_cam2=T_tracker_in_depth)[:3].T

    points_in_depth = depth2cloud(img_depth)

    distance = np.linalg.norm(np.cross(points_in_depth - p_eye_in_depth,
                                       points_in_depth-p_virtual_point_in_depth), axis=-1)
    distance /= np.linalg.norm(p_eye_in_depth - p_virtual_point_in_depth, axis=-1)

    depth_vec = np.copy(img_depth).reshape((-1))
    valid_indices = np.where(depth_vec > 100)[0]
    index_min = valid_indices[np.argmin(distance[valid_indices])]
    # index_min = np.argmin(distance)
    v = int(np.floor(index_min / img_depth.shape[1]))
    u = index_min - v * img_depth.shape[1]
    uv_est_in_depth = np.asarray([u, v])
    return uv_est_in_depth



def tracker_to_rgbd_with_depth(imgpoints_tracker, depth_points_tracker, rgbd_tracker_paras):
    T_tracker_in_depth = np.linalg.inv(rgbd_tracker_paras.get('T_mat_depth_in_tracker'))
    imgpoints_est_depth = cam1_to_cam2(
        imgpoints_tracker, depth_points_tracker,
        cam1_matrix=rgbd_tracker_paras.get('cam_matrix_tracker'),
        cam2_matrix=rgbd_tracker_paras.get('cam_matrix_depth'),
        T_cam1_in_cam2=T_tracker_in_depth)
    return imgpoints_est_depth

def tracker_to_rgbd(imgpoints_tracker, img_depth, rgbd_tracker_paras):
    imgpoints_est_depth = np.zeros(imgpoints_tracker.shape)
    for i in range(len(imgpoints_tracker)):
        imgpoints_est_depth[i] =  tracker_to_rgbd_without_depth(
            imgpoints_tracker[i], img_depth, rgbd_tracker_paras)
    return imgpoints_est_depth


def plot_rgb_cloud_from_depth(img_depth, img_rgb):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(depth2cloud(img_depth))
    pcd.colors = o3d.utility.Vector3dVector((img_rgb.astype(np.float) / 255.0).reshape((-1, 3)))
    o3d.visualization.draw_geometries([pcd])

def plot_rgb_cloud(cloud, img_rgb = None, view = True):
    pcd = o3d.geometry.PointCloud()
    z_vec = np.copy(cloud[:, 2])
    pcd.points = o3d.utility.Vector3dVector(cloud)
    if img_rgb is not None:
        pcd.colors = o3d.utility.Vector3dVector((img_rgb.astype(np.float) / 255.0).reshape((-1, 3)))
    if view:
        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=500, origin=[0, 0, 0])
        # vis = o3d.visualization.draw_geometries([pcd])
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(pcd)
        opt = vis.get_render_option()
        opt.background_color = np.asarray([0, 0, 0])
        vis.run()
    return pcd


def plot_rgb_cloud_with_gaze_vector(img_depth, img_rgb, uv_depth, uv_tracker, rgbd_tracker_paras):
    uv_depth_new = np.copy(uv_depth).reshape((1, -1))
    img_rgb_correct = draw_circles_on_img(img_rgb, uv_depth_new, uv_depth_new)

    pcd = o3d.geometry.PointCloud()
    img_rgb_correct = cv2.cvtColor(img_rgb_correct, cv2.COLOR_BGR2RGB)

    pcd.points = o3d.utility.Vector3dVector(depth2cloud(img_depth))
    pcd.colors = o3d.utility.Vector3dVector((img_rgb_correct.astype(np.float) / 255.0).reshape((-1, 3)))

    uv_depth_est = tracker_to_rgbd_without_depth(uv_tracker, img_depth, rgbd_tracker_paras)
    center = uvz2xyz(uv_depth_est[0], uv_depth_est[1], img_depth[uv_depth_est[1],uv_depth_est[0]])
    points_in_depth = center.reshape((1, 3)) + np.random.uniform(-1, 1, (100, 3))
    pcd_depth = o3d.geometry.PointCloud()
    pcd_depth.points = o3d.utility.Vector3dVector(points_in_depth)
    color_vec_depth = np.zeros(points_in_depth.shape)
    color_vec_depth[:, 2] = 1
    pcd_depth.colors = o3d.utility.Vector3dVector(color_vec_depth)

    cam_matrix_tracker = rgbd_tracker_paras.get('cam_matrix_tracker')
    T_tracker_in_depth = np.linalg.inv(rgbd_tracker_paras.get('T_mat_depth_in_tracker'))


    p_eye_in_depth = cam1_to_cam2_xyz(uv_tracker, np.asarray([0]),
                                      cam1_matrix=cam_matrix_tracker,
                                      T_cam1_in_cam2=T_tracker_in_depth)[:3].T
    p_virtual_point_in_depth = cam1_to_cam2_xyz(uv_tracker, np.asarray([8000]),
                                                cam1_matrix=cam_matrix_tracker,
                                                T_cam1_in_cam2=T_tracker_in_depth)[:3].T

    s = np.linspace(0, 1, num = 10000)
    gaze_vector = np.matmul(s.reshape((-1, 1)), p_virtual_point_in_depth - p_eye_in_depth) + p_eye_in_depth

    pcd_gaze = o3d.geometry.PointCloud()
    pcd_gaze.points = o3d.utility.Vector3dVector(gaze_vector)
    color_vec_depth = np.zeros(gaze_vector.shape)
    color_vec_depth[:, 1] = 1
    pcd_gaze.colors = o3d.utility.Vector3dVector(color_vec_depth)
    o3d.visualization.draw_geometries([pcd, pcd_depth, pcd_gaze])


def rgbd_to_tracker(imgpoints_depth, depth_points, rgbd_tracker_paras):
    imgpoints_est_tracker = cam1_to_cam2(imgpoints_depth, depth_points,
                                         cam1_matrix = rgbd_tracker_paras.get('cam_matrix_depth'),
                                         cam2_matrix = rgbd_tracker_paras.get('cam_matrix_tracker'),
                                         T_cam1_in_cam2 = rgbd_tracker_paras.get('T_mat_depth_in_tracker'))
    return imgpoints_est_tracker

def xyz1_to_xyz2(xyz1, rgbd_tracker_paras):
    p_vec_1 = np.ones((4, 1))
    p_vec_1[:3, 0] = xyz1
    T_cam1_in_cam2 = rgbd_tracker_paras.get('T_mat_depth_in_tracker')
    p_vec_2 = np.matmul(T_cam1_in_cam2, p_vec_1)
    return p_vec_2[:3, 0]

def cam1_to_cam2(uv_vec_1, z_vec_1, cam1_matrix, cam2_matrix, T_cam1_in_cam2):
    p_vec_2 = cam1_to_cam2_xyz(uv_vec_1, z_vec_1, cam1_matrix, T_cam1_in_cam2)
    uvz_vec_2 = np.matmul(cam2_matrix, p_vec_2[:3])
    uv_vec_2 = uvz_vec_2[:2] / uvz_vec_2[[2]]
    uv_vec_2 = uv_vec_2.T
    return uv_vec_2

def cam1_to_cam2_xyz(uv_vec_1, z_vec_1, cam1_matrix, T_cam1_in_cam2):
    uv_vec_1 = uv_vec_1.reshape((-1, 2))
    num_points = uv_vec_1.shape[0]
    uv_vec_1 = np.asarray(uv_vec_1)
    uvz_vec_1 = np.ones((num_points, 3))
    uvz_vec_1[:, :2] = uv_vec_1
    uvz_vec_1 *= np.reshape(z_vec_1, (-1, 1))
    p_vec_1 = np.ones((4, num_points))
    p_vec_1[:3, :] = np.matmul(np.linalg.inv(cam1_matrix), uvz_vec_1.T)
    p_vec_2 = np.matmul(T_cam1_in_cam2, p_vec_1)
    return p_vec_2

def cam1_to_cam2_without_calibration(uv_1, u_ratio, v_ratio):
    uv_2 = np.copy(uv_1)
    uv_2[:, 0] = uv_1[:, 0] * u_ratio
    uv_2[:, 1] = uv_1[:, 1] * v_ratio
    return uv_2

# def read_numbers_from_img(img_text, is_show=False):
#     # converting image into gray scale image
#     img_text = cv2.cvtColor(img_text, cv2.COLOR_BGR2GRAY)
#     # converting it to binary image by Thresholding
#     # this step is require if you have colored image because if you skip this part
#     # then tesseract won't able to detect text correctly and this will give incorrect result
#     img_text = cv2.threshold(img_text, 254.5, 255, cv2.THRESH_BINARY)[1]
#     text = pytesseract.image_to_string(img_text)
#     float_array = np.asarray(re.findall(r"[-+]?\d*\.\d+|\d+", text), dtype=np.float32)
#     if is_show:
#         cv2.imshow('threshold_img: {}, {}'.format(float_array[1:3],float_array[-3:]), img_text)
#         cv2.waitKey(100)
#     return float_array


def read_image_to_video(img_name_vec, video_name, fps=2):
    img_name_vec.sort()
    img_array = []
    for filename in img_name_vec:
        img = cv2.imread(filename)
        height, width, layers = img.shape
        size = (width, height)
        img_array.append(img)

    fourcc = cv2.VideoWriter_fourcc(*'avc1') # H.264 encoder
    out = cv2.VideoWriter(video_name, fourcc, fps, size)

    for i in range(len(img_array)):
        out.write(img_array[i])
    out.release()

def fig_to_img():
    img_name = 'temp.jpg'
    plt.savefig(img_name, bbox_inches='tight')
    img = cv2.imread(img_name, cv2.IMREAD_UNCHANGED)
    os.remove(img_name)
    return img

def split_rgbd_data_to_folder():
    data_dir = 'data/dynamic'
    color_img_name_vec = glob.glob('{}/*.jpg'.format(data_dir))
    for r in range(len(color_img_name_vec)):
        color_img_name = color_img_name_vec[r]
        depth_img_name = color_img_name.replace(".jpg", ".npy")
        # depth_img_name = depth_img_name.replace("rgb/", "depth/")
        img_dir = '{}/test{}'.format(data_dir, r)
        if not os.path.exists(img_dir):
            os.makedirs(img_dir)
        copyfile(color_img_name, '{}/rgb.jpg'.format(img_dir))
        copyfile(depth_img_name, '{}/depth.npy'.format(img_dir))

def visualize_gaze_and_environment():
    val_dir = 'data/terrain_RGBD'
    exp_index = 2
    # depth_img_name = glob.glob('data/validation/img/test{}/depth/*.npy'.format(exp_index))[0]
    color_img_name = glob.glob('{}/test{}/rgb/*.jpg'.format(val_dir, exp_index))[0]
    validation_results = np.load('{}/tracker_to_depth_validation.npy'.format(val_dir), allow_pickle=True).item()
    target_uv_in_depth_vec = validation_results.get('target_uv_in_depth_vec')
    target_uv_in_tracker_vec = validation_results.get('target_uv_in_tracker_vec')

    depth_img_name_vec = glob.glob('{}/test{}/depth/*.npy'.format(val_dir, exp_index))
    img_depth_vec = np.zeros((len(depth_img_name_vec), 480, 640))

    for r in range(len(depth_img_name_vec)):
        depth_img_name = depth_img_name_vec[r]
        img_depth = np.load(depth_img_name) * 1000.0  # convert m to mm
        # img_depth = edge_preserving_filter(img_depth)
        img_depth_vec[r] = img_depth
    img_depth = np.mean(img_depth_vec, axis=0)
    # img_depth[img_depth > 3000] = 0

    interactive_img_show(img_depth)

    img_rgb = cv2.imread(color_img_name, cv2.IMREAD_UNCHANGED)
    img_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2RGB)
    print('Max depth value: {}'.format(np.max(img_depth)))
    print(img_depth.shape, img_rgb.shape)

    rgbd_tracker_paras = np.load('paras/rgbd_tracker_paras.npy', allow_pickle=True).item()
    plot_rgb_cloud_with_gaze_vector(img_depth, img_rgb, target_uv_in_depth_vec[exp_index - 1],
                                    target_uv_in_tracker_vec[exp_index - 1], rgbd_tracker_paras)



def read_rgb_cloud(val_dir = 'data/orbbec_terrains/test1', img_idx = 50, view_cloud = False):
    rgb_img_names = glob.glob('{}/rgb/*.jpg'.format(val_dir, img_idx))
    rgb_img_names.sort()

    cloud_names = glob.glob('{}/depth/*.npy'.format(val_dir, img_idx))
    cloud_names.sort()

    rgb_name = rgb_img_names[img_idx]
    cloud_name = cloud_names[img_idx]
    print(rgb_name, cloud_name)

    point_cloud = np.load(cloud_name) * 1000.0  # convert m to mm
    # point_cloud[point_cloud[2] > 3000] = 0
    img_rgb = cv2.imread(rgb_name, cv2.IMREAD_UNCHANGED)
    img_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2RGB)
    if view_cloud:
        plot_rgb_cloud(point_cloud.reshape((-1, 3)), img_rgb)
        interactive_img_show(point_cloud[:,:, 2])

    return img_rgb, point_cloud


if __name__ == '__main__':
    split_rgbd_data_to_folder()
