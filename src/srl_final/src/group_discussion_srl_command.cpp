//
// Created by harlab on 2022/4/21.
//

#include "iostream"
#include "vector"
#include "/opt/ros/noetic/include/ros/ros.h"
#include "/opt/ros/noetic/include/std_msgs/String.h"
#include "/opt/ros/noetic/include/std_msgs/Float32MultiArray.h"
#include "/opt/ros/noetic/include/nav_msgs/Odometry.h"
#include "/home/harlab/ros_ws_1/devel/include/srl_final/Float32MultiArraySRL.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "boost/thread/thread.hpp"
// #include "duration.h"

#include "eigen3/Eigen/Core" /// 重要：eigen库可满足矩阵计算需要
#include "eigen3/Eigen/Geometry"
using namespace std;
using namespace std_msgs;
using namespace nav_msgs;
using namespace Eigen;

/// 以下部分：模拟接收t265_1与t265_2的Odometry数据，将t265_1坐标系下的数据点转换到t265_2坐标系下
/// 初始化cam1和cam2的位置关系，均为相对世界坐标系下的点
/// 设定cam1和cam2的偏置，初始化时cam1在下，cam2在上，仅保留z轴偏置

// ros::NodeHandle n;





//
//Eigen::Matrix4d t265_2_to_srl(Vector4d t265_point){
////    rot_y = np.mat([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])
////    rot_z = np.mat([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
////    Trans = np.mat([[1, 0, 0, 0], [0, 1, 0, 0], [
////    0, 0, 1, 0], [0, 80.15, -5.95, 1]])
////    T_matrix = rot_y * rot_z * Trans
//
//
//
//    return SRL_point;
//}

void srl_trans_func(const srl_final::Float32MultiArraySRLConstPtr& sub1, const srl_final::Float32MultiArraySRLConstPtr& sub2){
    auto sub1_data = sub1->data;
    auto sub2_data = sub2->data;
    Eigen::Vector4d eye_point = {sub1_data[0], sub1_data[1], sub1_data[2], sub1_data[3]};
    cout << eye_point << endl;

    Eigen::Matrix4d t265_tf;
    t265_tf << sub2_data[0], sub2_data[1], sub2_data[2], sub2_data[3],
               sub2_data[4], sub2_data[5], sub2_data[6], sub2_data[7],
               sub2_data[8], sub2_data[9], sub2_data[10], sub2_data[11],
               sub2_data[12], sub2_data[13], sub2_data[14], sub2_data[15];
    Vector4d point_t265_2 = eye_point * t265_tf; /// 得到t265_2坐标系下的点


    /// 以下部分为t265_2至srl的变换
    Matrix4d rot_y, rot_z, Trans;
    rot_y << 0, 0, 1, 0,
            0, 1, 0, 0,
            -1, 0, 0, 0,
            0, 0, 0, 1;
    rot_z << 0, -1, 0, 0,
            1, 0, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    Trans << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 80.15, -5.95, 1;
    auto T_Matrix = rot_y * rot_z * Trans;



    Matrix4d SRL_Point = point_t265_2 * T_Matrix;
    // Vector4d SRL_Point = t265_2_to_srl(point_t265_2);
    cout << "----------------------\n";
    cout << SRL_Point << endl;

}



int main(int argc, char **argv){
    // ros::NodeHandle n;
    ros::init(argc, argv, "final_trans_all_sim");
    ros::NodeHandle n;
    // ros::Publisher tf_pub = n.advertise<std_msgs::Float32MultiArray>("t265_trans_all_sim", 100);
    // ros::Subscriber sub_2 = n.subscribe("/cam_2/odom/sample", 1000, final_trans_all_callback);
//    message_filters::Subscriber<Odometry> sub1(n, "/cam_1/odom/sample", 100);
//    message_filters::Subscriber<Odometry> sub2(n, "/cam_2/odom/sample", 100);
//    message_filters::TimeSynchronizer<Odometry, Odometry> sub_sync(sub1, sub2, 100);
//    sub_sync.registerCallback(boost::bind(&final_trans_all_callback, _1, _2));
    // ros::Subscriber node_1 = n.subscribe("/cam_1/odom/sample", 1, multi_node_func);
//    node_1.


    message_filters::Subscriber<srl_final::Float32MultiArraySRL> sub1(n, "eye_tracker_xyz_sim", 100);
    message_filters::Subscriber<srl_final::Float32MultiArraySRL> sub2(n, "t265_trans_all_sim", 100);
    // typedef message_filters::sync_policies::
    typedef message_filters::sync_policies::ApproximateTime<srl_final::Float32MultiArraySRL, srl_final::Float32MultiArraySRL> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(100), sub1, sub2);
    sync.registerCallback(boost::bind(&srl_trans_func, _1, _2));


    ros::spin();
    return 0;
}