#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import math
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from smart_robot.msg import MoveToPlantAction, MoveToPlantResult, MoveToPlantFeedback
from tf.transformations import euler_from_quaternion

class NavigatorNode:
    def __init__(self):
        rospy.init_node('navigator_node')

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.server = actionlib.SimpleActionServer('move_to_plant', MoveToPlantAction, self.execute, False)
        self.server.start()

        self.current_pos = Point()
        self.current_yaw = 0.0
        self.front_dist = 999.0
        
        rospy.loginfo("Navigator Action Server is ready.")

    def odom_callback(self, msg):
        self.current_pos = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def scan_callback(self, msg):
        ranges_in_front = msg.ranges[0:15] + msg.ranges[345:360]
        valid_ranges = [r for r in ranges_in_front if r > 0.0 and not math.isinf(r)]
        
        if valid_ranges:
            self.front_dist = min(valid_ranges)
        else:
            self.front_dist = 999.0

    def execute(self, goal):
        target_pos = goal.target_plant_position
        rospy.loginfo(f"Received goal to move to {target_pos.x, target_pos.y}")
        
        rate = rospy.Rate(10)
        feedback = MoveToPlantFeedback()
        result = MoveToPlantResult()

        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                result.success = False
                break
            
            # --- 核心導航邏輯 ---
            # 1. 避障優先
            if self.front_dist < 0.5:
                self.stop_and_turn()
                rate.sleep()
                continue

            # 2. 計算到目標的距離和角度
            dist_to_goal = math.sqrt((target_pos.x - self.current_pos.x)**2 + (target_pos.y - self.current_pos.y)**2)
            
            # 如果已到達
            if dist_to_goal < 0.2:
                rospy.loginfo("Goal reached.")
                result.success = True
                break

            # 3. 朝向目標
            angle_to_goal = math.atan2(target_pos.y - self.current_pos.y, target_pos.x - self.current_pos.x)
            angle_error = angle_to_goal - self.current_yaw
            
            # 修正角度，使其在-pi到pi之間
            if angle_error > math.pi: angle_error -= 2 * math.pi
            if angle_error < -math.pi: angle_error += 2 * math.pi

            # 4. 發布移動指令
            move_cmd = Twist()
            if abs(angle_error) > 0.1: # 如果角度偏差大，先轉向
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 0.4 if angle_error > 0 else -0.4
            else: # 角度差不多了，前進
                move_cmd.linear.x = 0.2
                move_cmd.angular.z = 0.0
            
            self.cmd_vel_pub.publish(move_cmd)

            # 發布回饋
            feedback.current_robot_position = self.current_pos
            self.server.publish_feedback(feedback)
            
            rate.sleep()

        # 停止機器人
        self.cmd_vel_pub.publish(Twist())
        if result.success:
            self.server.set_succeeded(result)

    def stop_and_turn(self):
        rospy.logwarn("Obstacle detected! Stopping and turning.")
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.5 # 左轉
        self.cmd_vel_pub.publish(move_cmd)

if __name__ == '__main__':
    try:
        NavigatorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass