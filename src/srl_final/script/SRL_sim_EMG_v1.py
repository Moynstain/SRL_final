#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import numpy as np
from std_msgs.msg import Header, Float32MultiArray, UInt8

rospy.init_node('EMG_sim_node')

EMG_pub = rospy.Publisher('/EMG_sim', UInt8, queue_size=10)
while True:
    msg = raw_input("EMG sim signal: ")
    if msg == "1":
        pub_data = 1
        EMG_pub.publish(pub_data)
    if msg == "q":
        break


        