//
// Created by harlab on 2022/4/18.
//

/// 本文档为 ~/script/2_final_trans_all.py的c++版本

#include "iostream"
#include "vector"
#include "/opt/ros/noetic/include/ros/ros.h"
#include "/opt/ros/noetic/include/std_msgs/String.h"
#include "/opt/ros/noetic/include/std_msgs/UInt16MultiArray.h"
#include "/opt/ros/noetic/include/nav_msgs/Odometry.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "boost/thread/thread.hpp"
// #include "duration.h"

#include "eigen3/Eigen/Core" /// 重要：eigen库可满足矩阵计算需要
#include "eigen3/Eigen/Geometry"
using namespace std;
using namespace nav_msgs;
using namespace Eigen;

Eigen::MatrixXd tracker_2_t265(double x, double y, double depth){
    // Eigen::MatrixXd tracker_point = (x, y, depth, 1);
    Eigen::Matrix<double, 1, 4> tracker_point = {x, y, depth, 1};
    Eigen::Matrix<double, 4, 4> Rot_x, Trans;
    Rot_x << 1,0,0,0,
            0,0,-1,0,
            0,1,0,0,
            0,0,0,1;
    Trans << 1,0,0,0,
            0,1,0,0,
            0,0,1,0,
            -98, 44.9, 17.25, 1;
    auto T_matrix = Rot_x * Trans;
    auto T265_point = tracker_point * T_matrix;
    return T265_point;
}

void final_trans_all_callback(const OdometryPtr& data){

    cout << "yes";
}

Matrix4d odo_2_trans_mat_func(const OdometryPtr& sub){
    auto quat_data = sub->pose.pose.orientation;
    auto dist_data = sub->pose.pose.position;
    Quaterniond quaternion(quat_data.w, quat_data.x, quat_data.y, quat_data.z);
    Eigen::Matrix3d rot_mat = quaternion.toRotationMatrix();
    Vector4d distance = {dist_data.x, dist_data.y, dist_data.z, 1};
    Matrix4d trans_mat;
//    for (int row_num = 0; row_num <= trans_mat.rows(); row_num++){
//        for (int col_num = 0; col_num <= trans_mat.cols(); col_num++){
//            trans_mat(row_num, col_num) =
//        }
//
//    }
    trans_mat << rot_mat(0, 0), rot_mat(0, 1), rot_mat(0, 2), distance(0),
            rot_mat(1, 0), rot_mat(1, 1), rot_mat(1, 2), distance(1),
            rot_mat(2, 0), rot_mat(2, 1), rot_mat(2, 2), distance(2),
            rot_mat(3, 0), rot_mat(3, 1), rot_mat(3, 2), distance(3);
    return trans_mat;
}


// ros::init(argc, argv, "/cam_1_data");
ros::NodeHandle n;
ros::Subscriber sub_1 = n.subscribe("/cam_1/odom/sample", 1, final_trans_all_callback);
ros::Subscriber sub_2 = n.subscribe("/cam_1/odom/sample", 1, final_trans_all_callback);
ros::Subscriber sub_data = n.subscribe("topic name", 10, );


int main(int argc, char **argv){
    ros::init(argc, argv, "final_trans_all");

    // ros::Subscriber sub_2 = n.subscribe("/cam_2/odom/sample", 1000, final_trans_all_callback);
//    message_filters::Subscriber<Odometry> sub1(n, "/cam_1/odom/sample", 100);
//    message_filters::Subscriber<Odometry> sub2(n, "/cam_2/odom/sample", 100);
//    message_filters::TimeSynchronizer<Odometry, Odometry> sub_sync(sub1, sub2, 100);
//    sub_sync.registerCallback(boost::bind(&final_trans_all_callback, _1, _2));
    message_filters::Subscriber<Odometry> sub1(n, "/cam_1/odom/sample", 100, ros::TransportHints());
    message_filters::Subscriber<Odometry> sub2(n, "/cam_2/odom/sample", 100, ros::TransportHints().tcpNoDelay());
    // typedef message_filters::sync_policies::
    typedef message_filters::sync_policies::ApproximateTime<Odometry,Odometry> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(100), sub1, sub2);
    sync.registerCallback(boost::bind(&final_trans_all_callback, _1, _2));
    auto trans_1 = odo_2_trans_mat_func(sub_1);


    ros::spin();
    return 0;
}

