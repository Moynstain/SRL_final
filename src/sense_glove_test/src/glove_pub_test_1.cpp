//
// Created by harlab on 2022/4/8.
//

#include "/opt/ros/noetic/include/ros/ros.h"
#include "/opt/ros/noetic/include/std_msgs/String.h"
#include "/opt/ros/noetic/include/std_msgs/UInt16MultiArray.h"
#include "sstream"
#include "iostream"
#include "vector"

using namespace std;
using namespace std_msgs;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

int glove_feedback(){

}


int main(int argc, char **argv){
    ros::init(argc, argv, "glove_pub_test_1");
    ros::NodeHandle n;
    ros::Publisher glove_pub = n.advertise<std_msgs::UInt16MultiArray>("glove_test", 1000);
    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok()){
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());

        // cout << msg.data << endl;
        // msg_pub.publish(msg); /// 发布消息


        UInt16MultiArray msg_array_test;
        int temp;
        // vector<int> temp1();
        cout << "1. insert finger parameter (1-5): " << endl;
        // glove_test->push_back(cin.get());
        cin >> temp;
        msg_array_test.data.push_back(temp);
        cout << "debug 1 \n";
        // ROS_INFO(string(msg_array_test.data[0]));
        cout << "glove finger: " << msg_array_test.data[0] << endl;

        cout << "2. insert vibration_amplitude (0-100)): " << endl;
        cin >> temp;
        msg_array_test.data.push_back(temp);
        cout << "glove vibration_amplitude: " << msg_array_test.data[1] << endl;

        cout << "3. insert vibration_repeat_number: " << endl;
        cin >> temp;
        msg_array_test.data.push_back(temp);
        cout << "glove vibration_duration: " << msg_array_test.data[2] << endl;

        cout << "4. insert vibration_duration (ms) : " << endl;
        cin >> temp;
        msg_array_test.data.push_back(temp);
        cout << "glove vibration_duration: " << msg_array_test.data[3] << endl;
        cout << "------ separate -----\n";

        // msg_array_test.data = glove_msg;
        auto test = msg_array_test;
        glove_pub.publish(msg_array_test); /// 发布消息

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}