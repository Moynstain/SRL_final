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
// #include "srl_fina"
#include "boost/thread/thread.hpp"
// #include "duration.h"

#include "eigen3/Eigen/Core" /// 重要：eigen库可满足矩阵计算需要
#include "eigen3/Eigen/Geometry"
using namespace std;
using namespace nav_msgs;
using namespace Eigen;

/// 以下部分：模拟接收t265_1与t265_2的Odometry数据，将t265_1坐标系下的数据点转换到t265_2坐标系下
/// 初始化cam1和cam2的位置关系，均为相对世界坐标系下的点
/// 设定cam1和cam2的偏置，初始化时cam1在下，cam2在上，仅保留z轴偏置

// ros::NodeHandle n;

void multi_node_func(const OdometryConstPtr& sub1, const OdometryConstPtr& sub2){
    ros::NodeHandle n;
    ros::Publisher tf_pub = n.advertise<srl_final::Float32MultiArraySRL>("t265_trans_all_sim", 100);
    auto time_start = ros::Time::now();


    auto quat_data_1 = sub1->pose.pose.orientation;
    auto dist_data_1 = sub1->pose.pose.position;
    Quaterniond quat_1(quat_data_1.w, quat_data_1.x, quat_data_1.y, quat_data_1.z);
    Eigen::Matrix3d rot_1 = quat_1.toRotationMatrix();
    // cout << quat_1.x() << " " << quat_1.y() << " " << quat_1.z() << " " << quat_1.w() << endl;
    auto test = rot_1;
    // cout << rot_1 << endl;
    Matrix4d trans_1;
    trans_1 << rot_1(0, 0), rot_1(0, 1), rot_1(0, 2), dist_data_1.x,
               rot_1(1, 0), rot_1(1, 1), rot_1(1, 2), dist_data_1.y,
               rot_1(2, 0), rot_1(2, 1), rot_1(2, 2), dist_data_1.z,
               1, 1, 1, 1;


    auto quat_data_2 = sub2->pose.pose.orientation;
    auto dist_data_2 = sub2->pose.pose.position;
    Quaterniond quat_2(quat_data_2.w, quat_data_2.x, quat_data_2.y, quat_data_2.z);
    Eigen::Matrix3d rot_2 = quat_2.toRotationMatrix();
    Matrix4d trans_2;
    trans_2 << rot_2(0, 0), rot_2(0, 1), rot_2(0, 2), dist_data_2.x,
               rot_2(1, 0), rot_2(1, 1), rot_2(1, 2), dist_data_2.y,
               rot_2(2, 0), rot_2(2, 1), rot_2(2, 2), dist_data_2.z,
               1, 1, 1, 1;
//               rot_2(3, 0), rot_2(3, 1), rot_2(3, 2), 1;
    // cout << trans_2 << endl << endl;


    // Eigen::Vector4d t1_t2 = {0, 0, 0.245, 1}; /// 初始化时t265_1与t265_2的位置关系
    auto t1_inverse = trans_1.inverse();
    auto tf_temp = t1_inverse * trans_2;
    Eigen::Matrix4d t265_tf = tf_temp.inverse();
    // Eigen::Matrix4d t265_tf = tf_temp;
    // cout << t265_tf << endl;
    for (int i = 0; i <= 3; i++){
        t265_tf(i, 3) = t265_tf(i, 3) * 1000;
    }
    cout << "--------------------\n";
    cout << t265_tf << endl;
    t265_tf(2, 3) = t265_tf(2, 3) - 30.0;

    auto time_end = ros::Time::now();

    std_msgs::Float32MultiArray pub_data;
    // pub_data.data.push_back()
    // double temp_data;
    for (int row = 0; row <= 3; row++){
        for (int col = 0; col <= 3; col++){
            // temp_data = t265_tf(row, col);
            pub_data.data.push_back(t265_tf(row, col));
        }
    }
//    std_msgs::Float32MultiArray data;
//    data.data.push_back(t265_tf(0, 3)); /// DEBUG!!!

    cout << (time_start - time_end) << endl;


    tf_pub.publish(pub_data);
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
    message_filters::Subscriber<Odometry> sub1(n, "t265_no1_sim", 100, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<Odometry> sub2(n, "t265_no2_sim", 100, ros::TransportHints().tcpNoDelay());
    // typedef message_filters::sync_policies::
    typedef message_filters::sync_policies::ApproximateTime<Odometry,Odometry> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(100), sub1, sub2);
    sync.registerCallback(boost::bind(&multi_node_func, _1, _2));



    ros::spin();
    return 0;
}