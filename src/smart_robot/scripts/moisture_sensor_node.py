#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import random
from std_msgs.msg import Float32

def moisture_sensor_node():
    rospy.init_node('moisture_sensor_node')
    pub = rospy.Publisher('/moisture', Float32, queue_size=10)
    rate = rospy.Rate(0.5) # 每2秒發布一次數據
    rospy.loginfo("Moisture sensor simulator started.")

    while not rospy.is_shutdown():
        # 模擬一個 20% 到 80% 之間的濕度值
        moisture_level = random.uniform(20.0, 80.0)
        rospy.loginfo(f"Current moisture: {moisture_level:.2f}%")
        pub.publish(moisture_level)
        rate.sleep()

if __name__ == '__main__':
    try:
        moisture_sensor_node()
    except rospy.ROSInterruptException:
        pass
