//
// Created by harlab on 2022/4/11.
// 本文件实现：
// 接收pub节点发布的手套参数数组，并读取所有参数
// 成功根据pub的节点指定震动手指、震动力度与震动时长
// 实现时间：2022/4/14

#include "/opt/ros/noetic/include/ros/ros.h"
#include "/opt/ros/noetic/include/ros/time.h"
#include "/opt/ros/noetic/include/std_msgs/String.h"
#include "/opt/ros/noetic/include/std_msgs/UInt16MultiArray.h"
#include "iostream"

/// 以下为sense_glove导入的包
#include <thread>  //To pause the main() while vibrating
#include <chrono>  //To pause the thread for std::chrono::seconds

#include "Library.h"	// Access library details
#include "DeviceList.h" // Access devices and communication state
#include "Debugger.h"
#include "HapticGlove.h"
#include "SG_FFBCmd.h" /// 测试手套力反馈功能
using namespace std;
shared_ptr<SGCore::HapticGlove> testGlove;


int sg_initialize_func(){ /// 本函数用于系统初始化时手套的反馈
    testGlove->SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Finger::F1, 50));
    std::this_thread::sleep_for(std::chrono::milliseconds(200)); /// vibrate for 500ms 1=1ms
    testGlove->SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Finger::F2, 60));
    std::this_thread::sleep_for(std::chrono::milliseconds(200)); /// vibrate for 500ms 1=1ms
    testGlove->SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Finger::F3, 70));
    std::this_thread::sleep_for(std::chrono::milliseconds(200)); /// vibrate for 500ms 1=1ms
    testGlove->SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Finger::F4, 80));
    std::this_thread::sleep_for(std::chrono::milliseconds(200)); /// vibrate for 500ms 1=1ms
    testGlove->SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Finger::F5, 90));
    return 0;
}


int sg_finger_vibration_func(int finger_num, int buzz_level, int repeat_num, int duration_time){
    int repeat_val = 1;
    for (;repeat_val <= repeat_num; repeat_val++){
        switch (finger_num) {
            case 1:
                testGlove->SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Finger::F1, buzz_level));
                std::this_thread::sleep_for(std::chrono::milliseconds(duration_time)); /// vibrate for 500ms 1=1ms
                testGlove->SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Finger::F1, 0));
                break;
            case 2:
                testGlove->SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Finger::F2, buzz_level));
                std::this_thread::sleep_for(std::chrono::milliseconds(duration_time)); /// vibrate for 500ms 1=1ms
                testGlove->SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Finger::F2, 0));
                break;
            case 3:
                testGlove->SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Finger::F3, buzz_level));
                std::this_thread::sleep_for(std::chrono::milliseconds(duration_time)); /// vibrate for 500ms 1=1ms
                testGlove->SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Finger::F3, 0));
                break;
            case 4:
                testGlove->SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Finger::F4, buzz_level));
                std::this_thread::sleep_for(std::chrono::milliseconds(duration_time)); /// vibrate for 500ms 1=1ms
                testGlove->SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Finger::F4, 0));
                break;
            case 5:
                testGlove->SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Finger::F5, buzz_level));
                std::this_thread::sleep_for(std::chrono::milliseconds(duration_time)); /// vibrate for 500ms 1=1ms
                testGlove->SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Finger::F5, 0));
                break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200)); /// vibrate for 500ms 1=1ms
        testGlove->StopHaptics(); /// turn off vibration
    }

    return 0;
}

/// 订阅节点的回调函数，功能均在此函数中运行    const shared_ptr<SGCore::HapticGlove> glove_ptr
/// , const boost::shared_ptr<SGCore::HapticGlove>&
void sub_callback(const std_msgs::UInt16MultiArray& msg){
    ROS_INFO("received data: [%i, %i, %i, %i]", msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
    int finger_num = msg.data[0];
    int buzz_level = msg.data[1];
    int repeat_num = msg.data[2];
    int duration_time = msg.data[3];
    // shared_ptr<SGCore::HapticGlove> testGlove;
    // auto testGlove = glove_ptr;
    // auto testGlove = testGlove_func();
    cout << "-- 成功接收命令 --\n";
    testGlove->SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Finger::F2, buzz_level));
    // sg_finger_vibration_func(msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
    // testGlove->SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Finger::F1, msg.data[3]));
    // SGCore::HapticGlove::SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Finger::F1, msg.data[3]));
    // testGlove->SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Finger::F1, 50));

    std::this_thread::sleep_for(std::chrono::milliseconds(duration_time)); /// vibrate for 500ms 1=1ms
    // testGlove->SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Finger::F3, 50));
    // std::this_thread::sleep_for(std::chrono::milliseconds(200)); /// vibrate for 500ms 1=1ms

    sg_finger_vibration_func(finger_num, buzz_level, repeat_num, duration_time);

    testGlove->StopHaptics(); //turn off vibration
    // cout << "-- Error: 未接收命令 --\n";

}


int main(int argc, char **argv){
    /// 以下部分为sense_glove初始化代码
    cout << "Testing " + SGCore::Library::Version() + " compiled for " + SGCore::Library::BackEndVersion() << endl;
    cout << "=======================================" << endl;
    SGCore::Diagnostics::Debugger::debugLevel = SGCore::Diagnostics::DebugLevel::ERRORSONLY;


    // 检测sense glove的驱动程序是否在运行
    // 需先运行驱动程序，后使用本程序
    if (SGCore::DeviceList::SenseCommRunning()) {
        cout << "== 1. SenseComm成功运行 ==\n";
        // cout << "debug 1 testGlove ptr\n";
        if (SGCore::HapticGlove::GetGlove(testGlove)){
            bool rightHanded = testGlove->IsRight();
            cout << "== 2. 成功连接" << (rightHanded ? "右手" : "左手") << "手套 ==\n";
            // SGCore::DeviceType deviceType = testGlove->GetDeviceType();
        }
        else {
            cout << "== Error2. 未检测到sense glove手套，请检查USB连接。==\n" << endl;
        }
    }else{
        cout << "== Error1. SenseComm尚未运行 ==\n" << endl;
    }

    testGlove->SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Finger::F1, 50));
    std::this_thread::sleep_for(std::chrono::milliseconds(200)); /// vibrate for 500ms 1=1ms
    cout << testGlove << endl;
    testGlove->SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Finger::F2, 60));
    std::this_thread::sleep_for(std::chrono::milliseconds(200)); /// vibrate for 500ms 1=1ms
    cout << testGlove << endl;
    testGlove->SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Finger::F3, 70));
    std::this_thread::sleep_for(std::chrono::milliseconds(200)); /// vibrate for 500ms 1=1ms
    testGlove->SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Finger::F4, 80));
    std::this_thread::sleep_for(std::chrono::milliseconds(200)); /// vibrate for 500ms 1=1ms
    testGlove->SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Finger::F5, 90));

    testGlove->StopHaptics(); //turn off vibration

    ros::init(argc, argv, "glove_receive_node");
    ros::NodeHandle n;
    cout << "== 3. ROS Node初始化，等待接收命令 ==\n";

    ros::Subscriber sub = n.subscribe("glove_test", 1000, sub_callback);
    ros::spin();
    return 0;
}
//  sub1 = n.subscribe("/navsat/odom", 10, boost::bind(&multiThreadListener::chatterCallback1, this, _1, name1));