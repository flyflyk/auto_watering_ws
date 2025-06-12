#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist  # 用於發布速度指令
from sensor_msgs.msg import LaserScan # 用於訂閱雷射掃描數據

class NavigatorNode:
    def __init__(self):
        rospy.init_node('navigator_node', anonymous=True)

        # 建立一個 Publisher，發布 Twist 型別的訊息到 /cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # 建立一個 Subscriber，訂閱 LaserScan 型別的訊息從 /scan topic
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        rospy.loginfo("Navigator Node is ready.")

    def scan_callback(self, scan_data):
        front_dist = scan_data.ranges[0]
        rospy.loginfo("Distance to front obstacle: %.2f meters", front_dist)
        move_cmd = Twist()

        # --- 核心避障邏輯 ---
        # 如果正前方的距離大於 0.8 公尺，就直走
        if front_dist > 0.8:
            move_cmd.linear.x = 0.2  # 前進速度 0.2 m/s
            move_cmd.angular.z = 0.0 # 不轉彎
        else:
            # 否則，如果太近，就停止前進並原地左轉
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.5  # 左轉速度 0.5 rad/s

        # 發布速度指令
        self.cmd_vel_pub.publish(move_cmd)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        navigator = NavigatorNode()
        navigator.run()
    except rospy.ROSInterruptException:
        pass