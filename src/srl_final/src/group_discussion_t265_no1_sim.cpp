//
// Created by harlab on 2022/4/21.
//

#include "/opt/ros/noetic/include/ros/ros.h"
#include "/opt/ros/noetic/include/nav_msgs/Odometry.h"
#include "/opt/ros/noetic/include/std_msgs/UInt16MultiArray.h"
#include "/opt/ros/noetic/include/std_msgs/Float32MultiArray.h"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "boost/thread/thread.hpp"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "time.h"
#include "stdlib.h"
#include "random"
#include "iomanip"
using namespace std;

/// 以下部分：模拟T265_1随机生成的odometry数据，包含quat及pose
random_device rd;
constexpr int MIN_x = -10;
constexpr int MAX_x = 10;
default_random_engine eng_x(rd());
uniform_real_distribution<double> distr_x(MIN_x, MAX_x);

constexpr int MIN_y = -10;
constexpr int MAX_y = 10;
default_random_engine eng_y(rd());
uniform_real_distribution<double> distr_y(MIN_y, MAX_y);

constexpr int MIN_z = -100;
constexpr int MAX_z = 100;
default_random_engine eng_z(rd());
uniform_real_distribution<double> distr_z(MIN_z, MAX_z);

int main(int argc, char **argv) {
    ros::init(argc, argv, "t265_no1_sim");
    ros::NodeHandle n;
    ros::Publisher glove_pub = n.advertise<std_msgs::Float32MultiArray>("t265_no1_sim", 100);
    ros::Rate loop_rate(100);
    int count = 0;
    nav_msgs::Odometry pub_data;
    while (ros::ok()){
        auto quat_random = Eigen::Quaterniond::UnitRandom();
        Eigen::Matrix3d rot_random = quat_random.toRotationMatrix();
        // cout << rot_random << endl << endl;
        // double dist_random[] = {rand()}

        Eigen::Matrix<double, 1, 4> eye_tracker_xyz_sim = {distr_x(eng_x), distr_y(eng_y), distr_z(eng_z), 1};
        // cout << setprecision(7) << endl;
        // cout << distr_y(eng_y) << endl;
        vector<double> eye_tracker_xyz = {distr_x(eng_x), distr_y(eng_y), distr_z(eng_z), 1};

        pub_data.data.push_back(distr_x(eng_x));
        pub_data.data.push_back(distr_y(eng_y));
        pub_data.data.push_back(distr_z(eng_z));
        pub_data.data.push_back(1);
        // pub_data.data = {distr_x(eng_x), distr_y(eng_y), distr_z(eng_z), 1};
        cout << pub_data << endl;
        glove_pub.publish(pub_data); /// 发布消息

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}