//
// Created by harlab on 2022/4/21.
//

#include "/opt/ros/noetic/include/ros/ros.h"
#include "/opt/ros/noetic/include/nav_msgs/Odometry.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "time.h"
#include "stdlib.h"
#include "random"
#include "iomanip"
using namespace std;

/// 以下部分：模拟T265_1随机生成的odometry数据，包含quat及pose
random_device rd;
constexpr int MIN_x = 500;
constexpr int MAX_x = 550;
default_random_engine eng_x(rd());
uniform_real_distribution<double> distr_x(MIN_x, MAX_x);

constexpr int MIN_y = 500;
constexpr int MAX_y = 550;
default_random_engine eng_y(rd());
uniform_real_distribution<double> distr_y(MIN_y, MAX_y);

constexpr int MIN_z = -200;
constexpr int MAX_z = -150;
default_random_engine eng_z(rd());
uniform_real_distribution<double> distr_z(MIN_z, MAX_z);

int main(int argc, char **argv) {
    ros::init(argc, argv, "t265_no2_sim");
    ros::NodeHandle n;
    ros::Publisher glove_pub = n.advertise<nav_msgs::Odometry>("t265_no2_sim", 100);
    ros::Rate loop_rate(100);
    int count = 0;
    nav_msgs::Odometry pub_data;
    while (ros::ok()){
        auto quat_random = Eigen::Quaterniond::UnitRandom();
        // Eigen::Matrix3d rot_random = quat_random.toRotationMatrix();

        pub_data.pose.pose.orientation.x = quat_random.x();
        pub_data.pose.pose.orientation.y = quat_random.y();
        pub_data.pose.pose.orientation.z = quat_random.z();
        pub_data.pose.pose.orientation.w = quat_random.w();

        pub_data.pose.pose.position.x = distr_x(eng_x);
        pub_data.pose.pose.position.y = distr_y(eng_y);
        pub_data.pose.pose.position.z = distr_z(eng_z);

        cout << pub_data.pose.pose << endl;
        // ROS_INFO("%c",pub_data);
        glove_pub.publish(pub_data); /// 发布消息

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}