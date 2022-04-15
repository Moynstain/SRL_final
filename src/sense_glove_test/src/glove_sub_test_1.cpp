//
// Created by harlab on 2022/3/30.
// 本文件实现：
// 接收pub节点发布的手套参数数组，并读取所有参数

#include "/opt/ros/noetic/include/ros/ros.h"
#include "/opt/ros/noetic/include/std_msgs/String.h"
#include "/opt/ros/noetic/include/std_msgs/UInt16MultiArray.h"
#include "iostream"

using namespace std;





// const std_msgs::String::ConstPtr&
void sub_callback(const std_msgs::UInt16MultiArray& msg){
    // ROS_INFO("I heard: [%s]", msg->data.c_str());
    auto temp_data = msg.data;
    // ROS_INFO("I heard data: [%i]", temp_data.size());

}

int main(int argc, char **argv){
    ros::init(argc, argv, "glove_receive_node");
    ros::NodeHandle n;
    cout << "--- sub start --- \n";
    ros::Subscriber sub = n.subscribe("glove_test", 1000, sub_callback);
    // cout << "-- in callback --\n";
    ros::spin();
    return 0;
}
